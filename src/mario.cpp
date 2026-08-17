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
#include <opencv2/aruco.hpp>
#include <opencv2/core/check.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
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
#include <thread>
#include <tuple>
#include <unordered_set>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "fsm.hpp"
#include "nav/occupancy_map.hpp"
#include "nav/planner_astar.hpp"
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
auto mapping(nav::OccupancyMap &occupancy_map,
             utils::SharedLatest<struct slam::slamPose> &poseState,
             zmq::socket_t &sub, const rerun::RecordingStream &rec,
             utils::rs_config realsense_config) -> void {

  struct slam::slamPose pose;
  std::vector<zmq::message_t> pointcloud_msg;
  zmq::recv_result_t result_pointcloud;
  std::vector<float> points_buffer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  /* Identity, not default-constructed: an Affine3d leaves its bottom row
     uninitialised, and transformPointCloud reads the whole 4x4. */
  Eigen::Affine3d T_pc = Eigen::Affine3d::Identity();

  while (true) {

    pointcloud_msg.clear();

    result_pointcloud =
        zmq::recv_multipart(sub, std::back_inserter(pointcloud_msg));

    if (!result_pointcloud.has_value() || pointcloud_msg.size() < 2)
      continue;

    points_buffer.resize(pointcloud_msg[1].size() / sizeof(float));
    std::memcpy(points_buffer.data(), pointcloud_msg[1].data(),
                pointcloud_msg[1].size());

    const size_t num_points = points_buffer.size() / 3;
    cloud->width = static_cast<uint32_t>(num_points);
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(num_points);
    for (size_t i = 0; i < num_points; ++i) {
      cloud->points[i].x = points_buffer[3 * i + 0];
      cloud->points[i].y = points_buffer[3 * i + 1];
      cloud->points[i].z = points_buffer[3 * i + 2];
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

    /* Follow the rover before folding the cloud in, so the points land on a
       grid that still covers it. The map used to be a fixed box around the
       SLAM origin, which put the rover off its own map once it got further
       than dim/2 -- 10 m with the sim config, against a first waypoint 14 m
       out. */
    occupancy_map.recenter(pose.x, pose.y);

    occupancy_map.integrate(cloud, T_pc.matrix());
    occupancy_map.log(rec);

    /* Only the first grid is a milestone -- the FSM waits on it once before
       leaving BOOT. Signalling every iteration would be harmless but taking
       the lock every frame is not. */
    if (!map_sync.flag) {
      std::lock_guard<std::mutex> lock(map_sync.mtx);
      map_sync.flag = true;
      map_sync.cv.notify_all();
    }
  }
}

/* The transition graph lives in include/fsm.hpp so that test/fsm_test.cpp can
   drive the same table with scripted results. This class supplies the actions
   that graph calls -- everything below is the rover-side implementation. */
class StateMachine {
public:
  using State = fsm::State;
  using TraverseResult = fsm::TraverseResult;
  using PlanResult = fsm::PlanResult;
  using ApproachResult = fsm::ApproachResult;
  using WaypointType = fsm::WaypointType;
  using LedColor = fsm::LedColor;
  using Waypoint = fsm::Waypoint;

private:
  static constexpr auto state_name(State s) -> const char * {
    return fsm::state_name(s);
  }

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

    /* The map reaches dim/2 from the rover, not dim, and a goal planted on the
       very edge has no room for the safety margin either. Clamp to the
       half-extent of the shorter side, less a margin, so the goal is always
       somewhere the planner can actually reach. */
    const double half_extent =
        std::min(map_.params().dim[0], map_.params().dim[1]) / 2.0;
    const double reachable = std::max(
        half_extent - 4.0 * planner_params_.safety_margin, map_.resolution());
    double local_goal_dist = std::min(total_distance, reachable);

    double target_x = local_goal_dist * std::cos(relative_angle);
    double target_y = local_goal_dist * std::sin(relative_angle);

    /* Full 2D rotation into the world frame. The cross terms were dropped
       here, so the goal only landed correctly when the rover happened to be
       pointing along an axis -- a leg could look planned and still set off at
       an angle. */
    float local_goal_x =
        pose.x + (target_x * std::cos(pose.yaw) - target_y * std::sin(pose.yaw));
    float local_goal_y =
        pose.y + (target_x * std::sin(pose.yaw) + target_y * std::cos(pose.yaw));

    return std::make_tuple(local_goal_x, local_goal_y);
  }

public:
  /* ---- the actions fsm::run() calls ---------------------------------- */

  auto on_transition(State from, State to) -> void {
    if (from != to)
      spdlog::info(
          std::format("State: {} -> {}", state_name(from), state_name(to)));
    current_state = to;
  }

  auto wait_map_ready() -> void {
    std::unique_lock<std::mutex> lock(map_sync.mtx);
    map_sync.cv.wait(lock, [] { return map_sync.flag; });
  }

  auto load_waypoint() -> bool {
    if (waypoints.empty())
      return false;
    current_waypoint = waypoints.front();
    waypoints.pop();
    spdlog::info(std::format("Waypoint: lat={} lon={} type={}",
                             current_waypoint.lat, current_waypoint.lon,
                             (int)current_waypoint.type));
    return true;
  }

  auto current_waypoint_type() -> WaypointType { return current_waypoint.type; }

  /* One scan step: nudge the rover round and report whether the link held. */
  auto search_scan_step() -> bool {
    tarzan::tarzan_msg msg =
        tarzan::get_tarzan_msg(0.0, drive_cmd.angular_z * 0.3);
    return serial::write_msg<struct tarzan::tarzan_msg>(
               serial, msg, tarzan::TARZAN_MSG_LEN) ==
           serial::Error::WriteSuccess;
  }

  auto slam_tracking() -> bool {
    return slam::getStatus(slam_handler) == "Tracking";
  }

  auto clear_path() -> void { current_path.reset(); }

  auto on_mission_done() -> void {
    spdlog::info("State Machine: mission complete");
  }

  auto on_mission_abort(State in) -> void {
    spdlog::error(std::format("State Machine: aborted in {}", state_name(in)));
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
    if (!current_path || current_path->empty()) {
      spdlog::warn("State Machine: Path is empty or invalid");
      return TraverseResult::REACHED;
    }
    const nav::Path &path = *current_path;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);

    uint64_t previous = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    size_t waypoint_idx = 1;
    double linear_x = drive_cmd.linear_x, angular_z = 0.0;
    /* The configured maximum, held separately. Clamping against angular_z
       itself uses the previous output as this iteration's bound, which
       ratchets the command down to zero over a few waypoints. */
    const double max_angular = drive_cmd.angular_z;

    while (waypoint_idx < path.size()) {
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

      /* Same three-cell trip wire as before, asked in metres now that the
         map answers in them. */
      if (map_.clearance(pose.x, pose.y) < 3.0 * map_.resolution())
        return TraverseResult::REPLAN_OBSTACLE;

      double dx = path[waypoint_idx].x - pose.x;
      double dy = path[waypoint_idx].y - pose.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance < map_.resolution()) {
        waypoint_idx++;
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
            -max_angular, max_angular);
      previous = now;

      tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(linear_x, angular_z);
      serial::Error err = serial::write_msg<struct tarzan::tarzan_msg>(
          serial, msg, tarzan::TARZAN_MSG_LEN);
      if (err != serial::Error::WriteSuccess)
        return TraverseResult::FAULT_SERIAL;
      spdlog::debug("State Machine: writing drive cmd");
    }

    /* The last twist written is still in effect, so stop before handing the
       FSM back -- otherwise the rover coasts through the replan. */
    stop_motors();
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

    auto path = planner_.plan({pose.x, pose.y}, {local_x, local_y});
    if (!path) {
      spdlog::warn("State Machine: no path to local goal");
      return PlanResult::FAULT;
    }

    current_path = std::move(path);
    return PlanResult::PATH_FOUND;
  }

public:
  nav::OccupancyMap &map_;
  nav::Planner &planner_;
  /* Kept alongside the planner because get_local_goal has to size the local
     goal against the same margin the planner will demand of the path. */
  nav::PlannerParams planner_params_;
  struct control::Pid *pid_ctx;
  boost::asio::serial_port *serial;
  utils::SharedLatest<struct slam::slamPose> &poseState;
  tarzan::DiffDriveTwist drive_cmd;
  slam::slamHandle *slam_handler;
  std::optional<nav::Path> current_path;
  std::queue<Waypoint> waypoints;
  zmq::socket_t &color_sub;
  const rerun::RecordingStream &rec;
  YOLO8Detector *detector;

  /* Retry counts and search deadlines are fsm::run()'s bookkeeping now; this
     only tracks the current state so aborts can name where they happened. */
  State current_state = State::BOOT;
  Waypoint current_waypoint{};

  StateMachine(nav::OccupancyMap &map, nav::Planner &planner,
               const nav::PlannerParams &planner_params,
               struct control::Pid *pid, boost::asio::serial_port *ser,
               utils::SharedLatest<struct slam::slamPose> &pose_state,
               tarzan::DiffDriveTwist cmd, slam::slamHandle *sh,
               zmq::socket_t &csub, const rerun::RecordingStream &r,
               YOLO8Detector *det)
      : map_(map), planner_(planner), planner_params_(planner_params),
        pid_ctx(pid), serial(ser), poseState(pose_state), drive_cmd(cmd),
        slam_handler(sh), color_sub(csub), rec(r), detector(det) {};

  /* The graph itself is in include/fsm.hpp, shared with test/fsm_test.cpp. */
  auto run() -> int { return fsm::run(*this); }
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

  /* Everything below reads these with vm[...].as<>(), which throws on a
     missing option rather than telling you which one you forgot. */
  for (const char *opt :
       {"serial", "br", "rerun_ip", "gridmap_config", "p", "i", "d", "gnss",
        "linear", "angular", "yolo_model", "yolo_labels"}) {
    if (!vm.count(opt)) {
      spdlog::error("Missing required option: --{}", opt);
      return -1;
    }
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

  /* mapping & path planning vars */
  const std::string nav_cfg = vm["gridmap_config"].as<std::string>();
  nav::MapParams map_params = nav::loadMapParams(nav_cfg);
  nav::PlannerParams planner_params = nav::loadPlannerParams(nav_cfg);

  nav::OccupancyMap occupancy_map(map_params);
  std::unique_ptr<nav::Planner> planner =
      std::make_unique<nav::AStarPlanner>(occupancy_map, planner_params);

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

  StateMachine sm(occupancy_map, *planner, planner_params, pid_ctx, serial,
                  poseState,
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
  std::thread mapping_thread(mapping, std::ref(occupancy_map),
                             std::ref(poseState), std::ref(mapping_sub),
                             std::ref(rec), realsense_config);

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
  delete pid_ctx;
  delete detector;
  if (rs_ptr)
    utils::destroyHandle(rs_ptr);
  serial::close(serial);

  return 0;
}
