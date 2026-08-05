#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <ctime>
#include <format>
#include <fstream>
#include <iterator>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>
#include <memory>
#include <mutex>
#include <ompl/base/Path.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/check.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/pcl_macros.h>
#include <queue>
#include <rerun.hpp>
#include <rerun/archetypes/image.hpp>
#include <rerun/archetypes/text_log.hpp>
#include <rerun/recording_stream.hpp>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <taskflow/taskflow.hpp>
#include <thread>
#include <tuple>
#include <unordered_set>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "nav.hpp"
#include "pid.hpp"
#include "serial.hpp"
#include "slam.hpp"
#include "utils.hpp"
#include "yolo.hpp"

namespace po = boost::program_options;

#define EARTH_RADIUS 6378137.0

/* zmq topic names */
const std::string topic_color = "color_frame";
const std::string topic_depth = "depth_frame";
const std::string topic_timestamp = "timestamp";
const std::string topic_pointcloud = "pointcloud";

struct mapReadySignal {
  std::mutex mtx;
  std::condition_variable cv;
  bool flag = false;
} map_sync;

/* function to capture & publish realsense frames */
auto capture_frame(struct utils::rs_handler *rs_ptr, zmq::socket_t &pub)
    -> void {

  rs2::frame frame;

  while (true) {
    int ret;

    frame = rs_ptr->frame_q.wait_for_frame();

    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      auto aligned_frames = rs_ptr->align.process(fs);

      rs2::video_frame colorFrame = aligned_frames.first(RS2_STREAM_COLOR);
      rs2::video_frame depthFrame = aligned_frames.get_depth_frame();

      const void *raw_colorFrame = colorFrame.get_data();
      const void *raw_depthFrame = depthFrame.get_data();

      size_t colorFrame_len = colorFrame.get_data_size();
      size_t depthFrame_len = depthFrame.get_data_size();

      double timestamp = fs.get_timestamp();

      rs2::points points = rs_ptr->pc.calculate(depthFrame);
      const rs2::vertex *vertices = points.get_vertices();
      std::vector<float> vertices_vector;
      vertices_vector.reserve(points.size() * 3);
      for (size_t i = 0; i < points.size(); i++) {
        /* realsense encodes "no depth here" as an exact (0,0,0) vertex. Those
           are finite, so no downstream PCL filter drops them — left in, they
           collapse onto the camera origin and become a phantom obstacle under
           the rover. */
        if (vertices[i].z <= 0.0f)
          continue;
        vertices_vector.push_back(vertices[i].x);
        vertices_vector.push_back(vertices[i].y);
        vertices_vector.push_back(vertices[i].z);
      }

      ret = utils::publish_msg(
          pub, topic_color, [raw_colorFrame, colorFrame_len]() {
            zmq::message_t msg(colorFrame_len);
            memcpy(msg.data(), raw_colorFrame, colorFrame_len);
            return msg;
          });
      if (!ret)
        spdlog::error("Error Publishing: topic_color");

      ret = utils::publish_msg(
          pub, topic_depth, [raw_depthFrame, depthFrame_len]() {
            zmq::message_t msg(depthFrame_len);
            memcpy(msg.data(), raw_depthFrame, depthFrame_len);
            return msg;
          });
      if (!ret)
        spdlog::error("Error Publishing: topic_depth");

      ret = utils::publish_msg(pub, topic_timestamp, [timestamp]() {
        std::string timestamp_string = std::to_string(timestamp);
        zmq::message_t msg(timestamp_string.size());
        memcpy(msg.data(), timestamp_string.data(), timestamp_string.size());
        return msg;
      });
      if (!ret)
        spdlog::error("Error Publishing: topic_timestamp");

      ret = utils::publish_msg(pub, topic_pointcloud, [vertices_vector]() {
        zmq::message_t msg(vertices_vector);
        return msg;
      });
      if (!ret)
        spdlog::error("Error Publishing: topic_pointcloud");
    }
  }
}

/* function to get slam pose */
auto localize(struct slam::slamHandle *slam_handler,
              utils::SharedLatest<struct slam::slamPose> &poseState,
              zmq::socket_t &sub, const rerun::RecordingStream &rec,
              utils::rs_config realsense_config) -> void {

  Eigen::Matrix<double, 4, 4> current_pose;
  Eigen::Matrix<double, 4, 4> res;
  std::vector<zmq::message_t> colorFrameMsg;
  std::vector<zmq::message_t> depthFrameMsg;
  std::vector<zmq::message_t> timestampMsg;
  zmq::recv_result_t result_color;
  zmq::recv_result_t result_depth;
  zmq::recv_result_t result_timestamp;
  struct slam::rawColorDepthPair *frame_raw = new slam::rawColorDepthPair;
  struct slam::RGBDFrame *frame_cv;

  while (true) {
    result_color = zmq::recv_multipart(sub, std::back_inserter(colorFrameMsg));
    result_depth = zmq::recv_multipart(sub, std::back_inserter(depthFrameMsg));
    result_timestamp =
        zmq::recv_multipart(sub, std::back_inserter(timestampMsg));

    if (!result_color.has_value() || !result_depth.has_value() ||
        !result_timestamp.has_value()) {
      colorFrameMsg.clear();
      depthFrameMsg.clear();
      timestampMsg.clear();
      continue;
    }

    frame_raw->colorFrame = colorFrameMsg[1].data();
    frame_raw->depthFrame = depthFrameMsg[1].data();
    frame_raw->timestamp = std::stod(timestampMsg[1].to_string());

    frame_cv = slam::getColorDepthPair(frame_raw);

    res = slam::runLocalization(frame_cv, slam_handler);

    current_pose = utils::T_camera_base * res;
    auto translations = current_pose.col(3);

    float x = translations.x();
    float y = translations.y();
    float z = translations.z();
    float yaw = slam::yawfromPose(current_pose);

    struct slam::slamPose pose = {.x = x, .y = y, .z = z, .yaw = yaw};
    poseState.set(pose);

    std::string coordinates = std::format("x: {} y: {} yaw: {}", x, y, yaw);
    rec.log("SlamPose", rerun::TextLog(coordinates));

    colorFrameMsg.clear();
    depthFrameMsg.clear();
    timestampMsg.clear();
  }
}

/* function to create gridmap  */
auto mapping(nav::navContext *nav_ctx,
             utils::SharedLatest<struct slam::slamPose> &poseState,
             zmq::socket_t &sub, const rerun::RecordingStream &rec,
             utils::rs_config realsense_config) -> void {

  struct slam::slamPose pose;
  std::vector<zmq::message_t> pointcloud_msg;
  zmq::recv_result_t result_pointcloud;
  std::vector<float> points_buffer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Affine3d T_pc;

  while (true) {

    pointcloud_msg.clear();

    result_pointcloud =
        zmq::recv_multipart(sub, std::back_inserter(pointcloud_msg));

    if (!result_pointcloud.has_value() || pointcloud_msg.size() < 2)
      continue;

    points_buffer.resize(pointcloud_msg[1].size() / sizeof(float));
    std::memcpy(points_buffer.data(), pointcloud_msg[1].data(),
                pointcloud_msg[1].size());

    int buffer_size = static_cast<int>(points_buffer.size() / 3);

    cloud->width = buffer_size;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(buffer_size);

    for (int i = 0; i < buffer_size; i++) {
      cloud->points[i].x = points_buffer[i * 3];
      cloud->points[i].y = points_buffer[i * 3 + 1];
      cloud->points[i].z = points_buffer[i * 3 + 2];
    }

    if (!poseState.get(pose)) {
      spdlog::warn("GridMap : No pose available yet, skipping cloud");
      continue;
    }

    /* cloud is in the camera optical frame; lift it into the SLAM world frame
       via camera->base, then the rover's own heading. Dropping the yaw term
       stamps obstacles into the map as if the rover never turned. */
    const Eigen::Matrix3d R_yaw =
        Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();

    T_pc.setIdentity();
    T_pc.linear() = R_yaw * utils::T_camera_base.block<3, 3>(0, 0);
    T_pc.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);

    nav::preProcessPointCloud(nav_ctx, cloud, T_pc.matrix());
    nav::processGridMapCells(nav_ctx, cloud);
    nav::log_gridmap(nav_ctx, rec);

    std::lock_guard<std::mutex> lock(map_sync.mtx);
    map_sync.flag = true;
    map_sync.cv.notify_all();
  }
}

class StateMachine {
public:
  enum class State {
    BOOT,
    WAIT_MAP_READY,
    LOAD_WAYPOINT,
    PLAN_PATH,
    TRAVERSE_PATH,
    SEARCH_TARGET,
    APPROACH_TARGET,
    WAYPOINT_REACHED,
    RECOVER_SLAM,
    RECOVER_OBSTACLE,
    FAULT_SERIAL,
    MISSION_DONE,
    MISSION_ABORT
  };

  enum class TraverseResult {
    REACHED,
    REPLAN_TIMEOUT,
    REPLAN_OBSTACLE,
    FAULT_SLAM,
    FAULT_SERIAL
  };

  enum class PlanResult { PATH_FOUND, AT_GOAL, FAULT };

  enum class ApproachResult { ARRIVED, LOST_TARGET, FAULT_SERIAL };

  enum class WaypointType { GPS_ONLY, GPS_ARUCO, GPS_OBJECT };

  enum class LedColor { RED, BLUE, GREEN };

  struct Waypoint {
    double lat = 0.0;
    double lon = 0.0;
    WaypointType type = WaypointType::GPS_ONLY;
    int aruco_id = -1;
  };

private:
  auto get_local_goal(struct tarzan::geodetic &current_gps,
                      double target_latitude, double target_longitude,
                      slam::slamPose &pose) -> std::tuple<float, float> {
    double dLat = DEG2RAD(target_latitude - current_gps.lat);
    double dLon = DEG2RAD(target_longitude - current_gps.lon);

    double x_east = dLon * std::cos(DEG2RAD(current_gps.lat)) * EARTH_RADIUS;
    double y_north = dLat * EARTH_RADIUS;

    double total_distance = std::sqrt((x_east * x_east) + (y_north * y_north));

    double target_angle_global = std::atan2(y_north, x_east);
    double rover_angle_global = DEG2RAD(90.0 - current_gps.head);
    double relative_angle =
        utils::normalize_angle(target_angle_global - rover_angle_global);

    double local_goal_dist =
        std::min(total_distance, (double)nav_ctx->params.grid_map_dim[1]);

    double target_x = local_goal_dist * std::cos(relative_angle);
    double target_y = local_goal_dist * std::sin(relative_angle);

    float local_goal_x = pose.x + (target_x * std::cos(pose.yaw));
    float local_goal_y = pose.y + (target_y * std::sin(pose.yaw));

    return std::make_tuple(local_goal_x, local_goal_y);
  }

  static auto state_name(State s) -> const char * {
    switch (s) {
    case State::BOOT: return "BOOT";
    case State::WAIT_MAP_READY: return "WAIT_MAP_READY";
    case State::LOAD_WAYPOINT: return "LOAD_WAYPOINT";
    case State::PLAN_PATH: return "PLAN_PATH";
    case State::TRAVERSE_PATH: return "TRAVERSE_PATH";
    case State::SEARCH_TARGET: return "SEARCH_TARGET";
    case State::APPROACH_TARGET: return "APPROACH_TARGET";
    case State::WAYPOINT_REACHED: return "WAYPOINT_REACHED";
    case State::RECOVER_SLAM: return "RECOVER_SLAM";
    case State::RECOVER_OBSTACLE: return "RECOVER_OBSTACLE";
    case State::FAULT_SERIAL: return "FAULT_SERIAL";
    case State::MISSION_DONE: return "MISSION_DONE";
    case State::MISSION_ABORT: return "MISSION_ABORT";
    }
    return "?";
  }

  auto log_transition(State from, State to) -> void {
    if (from != to)
      spdlog::info(
          std::format("State: {} -> {}", state_name(from), state_name(to)));
    current_state = to;
  }

  auto stop_motors() -> void {
    tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(0.0, 0.0);
    serial::write_msg<struct tarzan::tarzan_msg>(serial, msg,
                                                 tarzan::TARZAN_MSG_LEN);
  }

  /* LED protocol on Nucleo TBD — log only for now */
  auto signal_led(LedColor color) -> void {
    const char *name = color == LedColor::RED     ? "RED"
                       : color == LedColor::GREEN ? "GREEN"
                                                  : "BLUE";
    spdlog::info(std::format("LED: {}", name));
  }

  auto traverse() -> TraverseResult {
    auto geo_path =
        std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(current_path);
    if (!geo_path || geo_path->getStateCount() == 0) {
      spdlog::warn("State Machine: Path is empty or invalid");
      return TraverseResult::REACHED;
    }
    const auto states = geo_path->getStates();

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);

    uint64_t previous = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    int state_idx = 1;
    double linear_x = drive_cmd.linear_x, angular_z = drive_cmd.angular_z;

    while (state_idx < (int)states.size()) {
      if (std::chrono::steady_clock::now() > deadline)
        return TraverseResult::REPLAN_TIMEOUT;

      if (slam::getStatus(slam_handler) != "Tracking")
        return TraverseResult::FAULT_SLAM;

      slam::slamPose pose;
      if (!poseState.get(pose)) {
        spdlog::error("State Machine : No pose available yet");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      /* the pose read no longer blocks on a queue, so pace the control loop
         here instead — otherwise this spins and floods the serial port */
      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      grid_map::Position gm_pos(pose.x, pose.y);
      grid_map::Index gm_idx;
      if (nav_ctx->map->getIndex(gm_pos, gm_idx)) {
        for (auto &obs : nav_ctx->occupancy_list) {
          double d = std::sqrt(std::pow(gm_idx(0) - obs(0), 2) +
                               std::pow(gm_idx(1) - obs(1), 2));
          if (d < 3.0)
            return TraverseResult::REPLAN_OBSTACLE;
        }
      }

      auto state = states[state_idx]->as<ob::RealVectorStateSpace::StateType>();
      double target_x = state->values[0];
      double target_y = state->values[1];

      double dx = target_x - pose.x;
      double dy = target_y - pose.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance < nav_ctx->params.grid_map_res) {
        state_idx++;
        continue;
      }

      double target_yaw = std::atan2(dy, dx);
      double angular_error = utils::normalize_angle(target_yaw - pose.yaw);

      uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();

      if (std::abs(angular_error) < 0.01)
        angular_z = 0.0;
      else
        angular_z = std::clamp(
            control::computeCommand(pid_ctx, angular_error, now - previous),
            (angular_z * -1), angular_z);
      previous = now;

      tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(linear_x, angular_z);
      serial::Error err = serial::write_msg<struct tarzan::tarzan_msg>(
          serial, msg, tarzan::TARZAN_MSG_LEN);
      if (err != serial::Error::WriteSuccess)
        return TraverseResult::FAULT_SERIAL;
    }
    return TraverseResult::REACHED;
  }

  auto search_object() -> bool {
    std::vector<zmq::message_t> msgs;
    auto result = zmq::recv_multipart(color_sub, std::back_inserter(msgs));
    if (!result.has_value() || msgs.size() < 2)
      return false;

    cv::Mat frame(480, 640, CV_8UC3, msgs[1].data());
    if (frame.empty())
      return false;

    auto detections = detector->detect(frame);
    if (!detections.empty()) {
      detector->drawBoundingBox(frame, detections);
      spdlog::info(std::format("search: {} detections", detections.size()));
    }

    auto img_data = reinterpret_cast<const uint8_t *>(frame.data);
    rec.log("search/frame",
            rerun::Image::from_rgb24({img_data, img_data + frame.total() * 3},
                                     {640, 480}));
    return !detections.empty();
  }

  auto search_aruco() -> bool {
    std::vector<zmq::message_t> msgs;
    auto result = zmq::recv_multipart(color_sub, std::back_inserter(msgs));
    if (!result.has_value() || msgs.size() < 2)
      return false;

    cv::Mat frame(480, 640, CV_8UC3, msgs[1].data());
    if (frame.empty())
      return false;

    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, dict, corners, ids);

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(frame, corners, ids);
      spdlog::info(std::format("aruco: {} markers", ids.size()));
    }

    auto img_data = reinterpret_cast<const uint8_t *>(frame.data);
    rec.log("search/frame",
            rerun::Image::from_rgb24({img_data, img_data + frame.total() * 3},
                                     {640, 480}));

    if (ids.empty())
      return false;
    if (current_waypoint.aruco_id < 0)
      return true;
    for (int id : ids)
      if (id == current_waypoint.aruco_id)
        return true;
    return false;
  }

  auto approach() -> ApproachResult {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    int lost_frames = 0;
    uint64_t previous = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();

    while (std::chrono::steady_clock::now() < deadline) {
      std::vector<zmq::message_t> msgs;
      auto result = zmq::recv_multipart(color_sub, std::back_inserter(msgs));
      if (!result.has_value() || msgs.size() < 2)
        continue;

      cv::Mat frame(480, 640, CV_8UC3, msgs[1].data());
      if (frame.empty())
        continue;

      float center_x = -1.0f;
      float bbox_area = 0.0f;

      if (current_waypoint.type == WaypointType::GPS_ARUCO) {
        auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dict, corners, ids);
        for (size_t i = 0; i < ids.size(); i++) {
          if (current_waypoint.aruco_id < 0 ||
              ids[i] == current_waypoint.aruco_id) {
            float min_x = corners[i][0].x, max_x = corners[i][0].x;
            float min_y = corners[i][0].y, max_y = corners[i][0].y;
            for (auto &p : corners[i]) {
              min_x = std::min(min_x, p.x);
              max_x = std::max(max_x, p.x);
              min_y = std::min(min_y, p.y);
              max_y = std::max(max_y, p.y);
            }
            center_x = (min_x + max_x) / 2.0f;
            bbox_area = (max_x - min_x) * (max_y - min_y);
            break;
          }
        }
      } else {
        auto detections = detector->detect(frame);
        if (!detections.empty()) {
          auto &b = detections[0].box;
          center_x = b.x + b.width / 2.0f;
          bbox_area = b.area();
        }
      }

      if (center_x < 0) {
        if (++lost_frames > 30)
          return ApproachResult::LOST_TARGET;
        continue;
      }
      lost_frames = 0;

      /* bbox > 25% of frame ~ within 2m */
      if (bbox_area > 640.0f * 480.0f * 0.25f)
        return ApproachResult::ARRIVED;

      float error = (center_x - 320.0f) / 320.0f;
      uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
      double angular_z =
          std::clamp(control::computeCommand(pid_ctx, error, now - previous),
                     (double)(-drive_cmd.angular_z),
                     (double)drive_cmd.angular_z);
      previous = now;

      tarzan::tarzan_msg msg =
          tarzan::get_tarzan_msg(drive_cmd.linear_x * 0.5, angular_z);
      serial::Error err = serial::write_msg<struct tarzan::tarzan_msg>(
          serial, msg, tarzan::TARZAN_MSG_LEN);
      if (err != serial::Error::WriteSuccess)
        return ApproachResult::FAULT_SERIAL;
    }
    return ApproachResult::LOST_TARGET;
  }

  auto plan() -> PlanResult {
    slam::slamPose pose;
    if (!poseState.get(pose)) {
      spdlog::error("plan: unable to fetch pose");
      return PlanResult::FAULT;
    }

    struct tarzan::geodetic_msg geo_msg;
    serial::Error err = serial::read_msg<struct tarzan::geodetic_msg>(
        serial, &geo_msg, tarzan::GEODETIC_MSG_LEN);
    if (err == serial::AsioReadError || err == serial::CobsDecodeError) {
      spdlog::error(serial::get_error(err));
      return PlanResult::FAULT;
    }

    double target_latitude = current_waypoint.lat;
    double target_longitude = current_waypoint.lon;

    double dLat = DEG2RAD(target_latitude - geo_msg.geo_data.lat);
    double dLon = DEG2RAD(target_longitude - geo_msg.geo_data.lon);
    double x_east =
        dLon * std::cos(DEG2RAD(geo_msg.geo_data.lat)) * EARTH_RADIUS;
    double y_north = dLat * EARTH_RADIUS;
    double total_distance = std::sqrt((x_east * x_east) + (y_north * y_north));

    if (total_distance < 2.0)
      return PlanResult::AT_GOAL;

    auto [local_x, local_y] = get_local_goal(geo_msg.geo_data, target_latitude,
                                             target_longitude, pose);

    ob::ScopedState<> start(nav_ctx->space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = pose.x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = pose.y;

    ob::ScopedState<> goal(nav_ctx->space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = local_x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = local_y;

    current_path = nav::get_path(nav_ctx, start, goal);
    if (!current_path) {
      spdlog::error("plan: planner failed");
      return PlanResult::FAULT;
    }
    return PlanResult::PATH_FOUND;
  }

public:
  struct nav::navContext *nav_ctx;
  struct control::Pid *pid_ctx;
  boost::asio::serial_port *serial;
  utils::SharedLatest<struct slam::slamPose> &poseState;
  tarzan::DiffDriveTwist drive_cmd;
  slam::slamHandle *slam_handler;
  ob::PathPtr current_path;
  std::queue<Waypoint> waypoints;
  zmq::socket_t &color_sub;
  const rerun::RecordingStream &rec;
  YOLO8Detector *detector;

  State current_state = State::BOOT;
  Waypoint current_waypoint{};
  std::chrono::steady_clock::time_point search_started_at{};
  bool search_active = false;
  int serial_retry_count = 0;

  StateMachine(struct nav::navContext *nav, struct control::Pid *pid,
               boost::asio::serial_port *ser,
               utils::SharedLatest<struct slam::slamPose> &pose_state,
               tarzan::DiffDriveTwist cmd, slam::slamHandle *sh,
               zmq::socket_t &csub, const rerun::RecordingStream &r,
               YOLO8Detector *det)
      : nav_ctx(nav), pid_ctx(pid), serial(ser), poseState(pose_state),
        drive_cmd(cmd), slam_handler(sh), color_sub(csub), rec(r),
        detector(det) {};

  auto run() -> int {
    tf::Executor executor(1);
    tf::Taskflow taskflow;

    signal_led(LedColor::RED);
    current_state = State::BOOT;

    auto t_boot = taskflow.emplace([this]() -> int {
                            log_transition(current_state, State::WAIT_MAP_READY);
                            return 0;
                          })
                      .name("BOOT");

    auto t_wait_map = taskflow.emplace([this]() -> int {
                                std::unique_lock<std::mutex> lock(map_sync.mtx);
                                map_sync.cv.wait(
                                    lock, [] { return map_sync.flag; });
                                log_transition(current_state,
                                               State::LOAD_WAYPOINT);
                                return 0;
                              })
                          .name("WAIT_MAP_READY");

    auto t_load_wp = taskflow.emplace([this]() -> int {
                               if (waypoints.empty()) {
                                 log_transition(current_state,
                                                State::MISSION_DONE);
                                 return 0;
                               }
                               current_waypoint = waypoints.front();
                               waypoints.pop();
                               spdlog::info(std::format(
                                   "Waypoint: lat={} lon={} type={}",
                                   current_waypoint.lat, current_waypoint.lon,
                                   (int)current_waypoint.type));
                               log_transition(current_state, State::PLAN_PATH);
                               return 1;
                             })
                         .name("LOAD_WAYPOINT");

    auto t_plan_path = taskflow.emplace([this]() -> int {
                                 PlanResult pr = plan();
                                 if (pr == PlanResult::PATH_FOUND) {
                                   serial_retry_count = 0;
                                   log_transition(current_state,
                                                  State::TRAVERSE_PATH);
                                   return 0;
                                 }
                                 if (pr == PlanResult::FAULT) {
                                   log_transition(current_state,
                                                  State::FAULT_SERIAL);
                                   return 3;
                                 }
                                 if (current_waypoint.type ==
                                     WaypointType::GPS_ONLY) {
                                   log_transition(current_state,
                                                  State::WAYPOINT_REACHED);
                                   return 1;
                                 }
                                 log_transition(current_state,
                                                State::SEARCH_TARGET);
                                 return 2;
                               })
                           .name("PLAN_PATH");

    auto t_traverse_path =
        taskflow.emplace([this]() -> int {
                  TraverseResult tr = traverse();
                  switch (tr) {
                  case TraverseResult::REACHED:
                  case TraverseResult::REPLAN_TIMEOUT:
                    log_transition(current_state, State::PLAN_PATH);
                    return 0;
                  case TraverseResult::REPLAN_OBSTACLE:
                    log_transition(current_state, State::RECOVER_OBSTACLE);
                    return 1;
                  case TraverseResult::FAULT_SLAM:
                    log_transition(current_state, State::RECOVER_SLAM);
                    return 2;
                  case TraverseResult::FAULT_SERIAL:
                    log_transition(current_state, State::FAULT_SERIAL);
                    return 3;
                  }
                  return 0;
                })
            .name("TRAVERSE_PATH");

    auto t_search = taskflow.emplace([this]() -> int {
                              if (!search_active) {
                                search_started_at =
                                    std::chrono::steady_clock::now();
                                search_active = true;
                              }
                              if (std::chrono::steady_clock::now() -
                                      search_started_at >
                                  std::chrono::seconds(60)) {
                                search_active = false;
                                spdlog::warn(
                                    "Search: timeout, claiming partial credit");
                                log_transition(current_state,
                                               State::WAYPOINT_REACHED);
                                return 1;
                              }
                              bool found =
                                  current_waypoint.type ==
                                          WaypointType::GPS_ARUCO
                                      ? search_aruco()
                                      : search_object();
                              if (found) {
                                search_active = false;
                                log_transition(current_state,
                                               State::APPROACH_TARGET);
                                return 0;
                              }
                              tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(
                                  0.0, drive_cmd.angular_z * 0.3);
                              serial::Error err = serial::write_msg<
                                  struct tarzan::tarzan_msg>(
                                  serial, msg, tarzan::TARZAN_MSG_LEN);
                              if (err != serial::Error::WriteSuccess) {
                                log_transition(current_state,
                                               State::FAULT_SERIAL);
                                return 3;
                              }
                              return 2;
                            })
                        .name("SEARCH_TARGET");

    auto t_approach = taskflow.emplace([this]() -> int {
                                ApproachResult ar = approach();
                                if (ar == ApproachResult::ARRIVED) {
                                  log_transition(current_state,
                                                 State::WAYPOINT_REACHED);
                                  return 0;
                                }
                                if (ar == ApproachResult::LOST_TARGET) {
                                  log_transition(current_state,
                                                 State::SEARCH_TARGET);
                                  return 1;
                                }
                                log_transition(current_state,
                                               State::FAULT_SERIAL);
                                return 2;
                              })
                          .name("APPROACH_TARGET");

    auto t_wp_reached = taskflow.emplace([this]() -> int {
                                  stop_motors();
                                  signal_led(LedColor::GREEN);
                                  std::this_thread::sleep_for(
                                      std::chrono::seconds(2));
                                  log_transition(current_state,
                                                 State::LOAD_WAYPOINT);
                                  return 0;
                                })
                            .name("WAYPOINT_REACHED");

    auto t_recover_slam =
        taskflow.emplace([this]() -> int {
                  stop_motors();
                  auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::seconds(10);
                  while (std::chrono::steady_clock::now() < deadline) {
                    if (slam::getStatus(slam_handler) == "Tracking") {
                      log_transition(current_state, State::PLAN_PATH);
                      return 0;
                    }
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(200));
                  }
                  log_transition(current_state, State::MISSION_ABORT);
                  return 1;
                })
            .name("RECOVER_SLAM");

    auto t_recover_obs = taskflow.emplace([this]() -> int {
                                   stop_motors();
                                   current_path.reset();
                                   std::this_thread::sleep_for(
                                       std::chrono::milliseconds(500));
                                   log_transition(current_state,
                                                  State::PLAN_PATH);
                                   return 0;
                                 })
                             .name("RECOVER_OBSTACLE");

    auto t_fault_serial = taskflow.emplace([this]() -> int {
                                    stop_motors();
                                    if (++serial_retry_count > 5) {
                                      log_transition(current_state,
                                                     State::MISSION_ABORT);
                                      return 1;
                                    }
                                    std::this_thread::sleep_for(
                                        std::chrono::seconds(1));
                                    log_transition(current_state,
                                                   State::PLAN_PATH);
                                    return 0;
                                  })
                              .name("FAULT_SERIAL");

    auto t_mission_done = taskflow.emplace([this]() -> void {
                                    signal_led(LedColor::GREEN);
                                    spdlog::info(
                                        "State Machine: mission complete");
                                  })
                              .name("MISSION_DONE");

    auto t_mission_abort = taskflow.emplace([this]() -> void {
                                     stop_motors();
                                     signal_led(LedColor::RED);
                                     spdlog::error(std::format(
                                         "State Machine: aborted in {}",
                                         state_name(current_state)));
                                   })
                               .name("MISSION_ABORT");

    t_boot.precede(t_wait_map);
    t_wait_map.precede(t_load_wp);
    t_load_wp.precede(t_mission_done, t_plan_path);
    t_plan_path.precede(t_traverse_path, t_wp_reached, t_search,
                        t_fault_serial);
    t_traverse_path.precede(t_plan_path, t_recover_obs, t_recover_slam,
                            t_fault_serial);
    t_search.precede(t_approach, t_wp_reached, t_search, t_fault_serial);
    t_approach.precede(t_wp_reached, t_search, t_fault_serial);
    t_wp_reached.precede(t_load_wp);
    t_recover_slam.precede(t_plan_path, t_mission_abort);
    t_recover_obs.precede(t_plan_path);
    t_fault_serial.precede(t_plan_path, t_mission_abort);

    executor.run(taskflow).wait();

    return current_state == State::MISSION_DONE ? 1 : 0;
  }
};

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::info);

  po::options_description desc("Allowed Options");
  desc.add_options()("help", "produce help message")(
      "serial", po::value<std::string>(), "serial port")(
      "br", po::value<int>(), "baudrate")(
      "rerun_ip", po::value<std::string>(), "rerun viewer ip")(
      "gridmap_config", po::value<std::string>(), "gridmap parameters file")(
      "p", po::value<double>(), "proportional gain")(
      "i", po::value<double>(), "integral gain")(
      "d", po::value<double>(), "differential gain")(
      "gnss", po::value<std::string>(), "file path of gnss targets")(
      "linear", po::value<float>(), "max linear velocity")(
      "angular", po::value<float>(), "max angular velocity")(
      "yolo_model", po::value<std::string>(), "path to YOLO ONNX model")(
      "yolo_labels", po::value<std::string>(), "path to YOLO class labels")(
      "sim", po::bool_switch(),
      "simulation mode: frames come from an external publisher (the Webots "
      "mario_bridge controller) instead of an attached RealSense")(
      "zmq_endpoint", po::value<std::string>()->default_value("inproc://realsense"),
      "frame transport endpoint; must not be inproc:// when --sim is set")(
      "slam_config", po::value<std::string>()->default_value("stellaconf.yaml"),
      "stella_vslam camera config")(
      "slam_vocab", po::value<std::string>()->default_value("orb_vocab.fbow"),
      "stella_vslam ORB vocabulary");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  /* In sim the Webots mario_bridge controller owns the PUB socket, so frames
     have to cross a process boundary -- inproc:// cannot reach it. */
  const bool sim_mode = vm["sim"].as<bool>();
  const std::string zmq_endpoint = vm["zmq_endpoint"].as<std::string>();
  if (sim_mode && zmq_endpoint.rfind("inproc://", 0) == 0) {
    spdlog::error("--sim needs an inter-process --zmq_endpoint "
                  "(e.g. tcp://127.0.0.1:5599), got {}",
                  zmq_endpoint);
    return -1;
  }

  /* zmq vars */
  zmq::context_t ctx(1);
  zmq::socket_t pub(ctx, ZMQ_PUB);
  zmq::socket_t slam_sub(ctx, ZMQ_SUB);
  zmq::socket_t mapping_sub(ctx, ZMQ_SUB);
  zmq::socket_t color_sub(ctx, ZMQ_SUB);

  /* realsense vars */
  struct utils::rs_config realsense_config{
      .height = 640, .width = 480, .fps = 30, .enable_imu = false};
  struct utils::rs_handler *rs_ptr = nullptr;

  /* slam vars */
  struct slam::slamHandle *slam_handler = new slam::slamHandle(
      vm["slam_config"].as<std::string>(), vm["slam_vocab"].as<std::string>());
  utils::SharedLatest<struct slam::slamPose> poseState;

  /* path planning vars */
  struct nav::navContext *nav_ctx =
      nav::setupNav(vm["gridmap_config"].as<std::string>());

  /* rerun vars */
  const auto rec = rerun::RecordingStream("TEAM RUDRA AUTONOMOUS - mario");
  const std::string rerun_url =
      std::format("rerun+http://{}/proxy", vm["rerun_ip"].as<std::string>());

  /* pid vars */
  struct control::Pid *pid_ctx =
      control::initPid(vm["p"].as<double>(), vm["i"].as<double>(),
                       vm["d"].as<double>());

  /* yolo detector */
  YOLO8Detector *detector =
      new YOLO8Detector(vm["yolo_model"].as<std::string>(),
                        vm["yolo_labels"].as<std::string>());

  /* CONFIGURING PERIPHERALS */
  spdlog::info("Configuring Rover Peripherals...");

  /* CONFIGURING REALSENSE */
  if (sim_mode) {
    spdlog::info("Sim mode: skipping Realsense, expecting frames on {}",
                 zmq_endpoint);
  } else {
    rs_ptr = utils::setupRealsense(realsense_config);
    if (not rs_ptr) {
      spdlog::error("Unable to setup Realsense");
      return -1;
    }
    spdlog::info("Successful setup of Realsense");
    rs2::frame frame;
    for (int i = 0; i < 100; i++) {
      frame = rs_ptr->frame_q.wait_for_frame();
    }
  }

  /* CONFIGURING NUCLEO COM */
  std::string SERIAL_PORT = vm["serial"].as<std::string>();
  int BAUDRATE = vm["br"].as<int>();
  boost::asio::io_context io;
  boost::asio::serial_port *serial = serial::open(io, SERIAL_PORT, BAUDRATE);
  if (not serial) {
    spdlog::error("Unable to setup serial port");
    return -1;
  }
  spdlog::info(std::format("Successful connection to {}", SERIAL_PORT));

  StateMachine sm(nav_ctx, pid_ctx, serial, poseState,
                  tarzan::DiffDriveTwist{vm["linear"].as<float>(),
                                         vm["angular"].as<float>()},
                  slam_handler, color_sub, rec, detector);

  /* CONFIGURING ZMQ SOCKETS */
  /* Only bind when we are the publisher. In sim the bridge has already bound
     this endpoint and binding it again would fail. */
  if (!sim_mode) {
    try {
      pub.bind(zmq_endpoint);
    } catch (zmq::error_t &e) {
      spdlog::error(e.what());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  try {
    slam_sub.connect(zmq_endpoint);
    slam_sub.set(zmq::sockopt::subscribe, topic_color);
    slam_sub.set(zmq::sockopt::subscribe, topic_depth);
    slam_sub.set(zmq::sockopt::subscribe, topic_timestamp);
  } catch (zmq::error_t &e) {
    spdlog::error(e.what());
  }

  try {
    mapping_sub.connect(zmq_endpoint);
    mapping_sub.set(zmq::sockopt::subscribe, topic_pointcloud);
  } catch (zmq::error_t &e) {
    spdlog::error(e.what());
  }

  try {
    color_sub.connect(zmq_endpoint);
    color_sub.set(zmq::sockopt::subscribe, topic_color);
  } catch (zmq::error_t &e) {
    spdlog::error(e.what());
  }

  /* CONFIGURING RERUN */
  rec.connect_grpc(rerun_url).exit_on_failure();

  /* LAUNCHING BACKGROUND THREADS */
  /* In sim the bridge publishes the frames, so there is nothing to capture. */
  std::thread capture_thread;
  if (!sim_mode)
    capture_thread = std::thread(capture_frame, rs_ptr, std::ref(pub));
  std::thread localize_thread(localize, slam_handler, std::ref(poseState),
                              std::ref(slam_sub), std::ref(rec),
                              realsense_config);
  std::thread mapping_thread(mapping, nav_ctx, std::ref(poseState),
                             std::ref(mapping_sub), std::ref(rec),
                             realsense_config);

  /* PARSE GNSS WAYPOINTS */
  {
    std::ifstream gnss_file(vm["gnss"].as<std::string>());
    if (!gnss_file.is_open()) {
      spdlog::error("Unable to open GNSS file");
      return -1;
    }
    std::string line;
    while (std::getline(gnss_file, line)) {
      if (line.empty())
        continue;
      std::istringstream ss(line);
      double lat, lon;
      std::string type_str = "gps";
      int aruco_id = -1;
      if (!(ss >> lat >> lon)) {
        spdlog::warn(std::format("Skipping malformed GNSS line: {}", line));
        continue;
      }
      ss >> type_str;
      ss >> aruco_id;
      StateMachine::WaypointType type = StateMachine::WaypointType::GPS_ONLY;
      if (type_str == "aruco")
        type = StateMachine::WaypointType::GPS_ARUCO;
      else if (type_str == "object")
        type = StateMachine::WaypointType::GPS_OBJECT;
      sm.waypoints.push({lat, lon, type, aruco_id});
    }
    gnss_file.close();
  }
  spdlog::info(std::format("Loaded {} GNSS waypoints", sm.waypoints.size()));

  sm.run();

  /* CLEANUP */
  if (capture_thread.joinable())
    capture_thread.join();
  localize_thread.join();
  mapping_thread.join();

  delete slam_handler;
  delete nav_ctx;
  delete pid_ctx;
  delete detector;
  if (rs_ptr)
    utils::destroyHandle(rs_ptr);
  serial::close(serial);

  return 0;
}
