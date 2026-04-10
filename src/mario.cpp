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
#include <opencv2/core/check.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/pcl_macros.h>
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
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "nav.hpp"
#include "pid.hpp"
#include "serial.hpp"
#include "slam.hpp"
#include "utils.hpp"

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
void capture_frame(struct utils::rs_handler *rs_ptr, zmq::socket_t &pub) {

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
      for (size_t i = 0; i < points.size(); i++) {
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
void localize(struct slam::slamHandle *slam_handler,
              utils::SafeQueue<struct slam::slamPose> &poseQueue,
              zmq::socket_t &sub, const rerun::RecordingStream &rec,
              utils::rs_config realsense_config) {

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
    poseQueue.produce(std::move(pose));

    std::string coordinates = std::format("x: {} y: {} yaw: {}", x, y, yaw);
    rec.log("SlamPose", rerun::TextLog(coordinates));

    colorFrameMsg.clear();
    depthFrameMsg.clear();
    timestampMsg.clear();
  }
}

/* function to create gridmap  */
void mapping(nav::navContext *nav_ctx,
             utils::SafeQueue<struct slam::slamPose> &poseQueue,
             zmq::socket_t &sub, const rerun::RecordingStream &rec,
             utils::rs_config realsense_config) {

  struct slam::slamPose pose;
  std::vector<zmq::message_t> pointcloud_msg;
  zmq::recv_result_t result_pointcloud;
  std::vector<float> points_buffer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Affine3d T_pc;

  while (!map_sync.flag) {

    result_pointcloud =
        zmq::recv_multipart(sub, std::back_inserter(pointcloud_msg));

    if (!result_pointcloud.has_value())
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

    if (!poseQueue.consume(pose)) {
      spdlog::error("GridMap : Unable to fetch data from Pose Queue");
      continue;
    }

    T_pc.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
    T_pc.linear() = utils::T_camera_base.block<3, 3>(0, 0);

    nav::preProcessPointCloud(nav_ctx, cloud, T_pc.matrix());
    nav::processGridMapCells(nav_ctx, cloud);
    nav::log_gridmap(nav_ctx, rec);

    std::lock_guard<std::mutex> lock(map_sync.mtx);
    map_sync.flag = true;
    map_sync.cv.notify_all();
  }
}

enum class TraverseResult {
  REACHED,
  REPLAN_TIMEOUT,
  REPLAN_OBSTACLE,
  FAULT_SLAM,
  FAULT_SERIAL
};

enum class PlanResult { PATH_FOUND, AT_GOAL, FAULT };

class StateMachine {
private:
  std::tuple<float, float> get_local_goal(struct tarzan::geodetic &current_gps,
                                          double target_latitude,
                                          double target_longitude,
                                          slam::slamPose &pose) {
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

  TraverseResult traverse() {
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
      if (!poseQueue.consume(pose)) {
        spdlog::error("State Machine : Unable to fetch data from Pose Queue");
        continue;
      }

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

public:
  struct nav::navContext *nav_ctx;
  struct control::Pid *pid_ctx;
  boost::asio::serial_port *serial;
  utils::SafeQueue<struct slam::slamPose> &poseQueue;
  const struct tarzan::DiffDriveTwist &drive_cmd;
  slam::slamHandle *slam_handler;
  ob::PathPtr current_path;
  std::queue<std::pair<double, double>> target_gnss;

  StateMachine(struct nav::navContext *nav, struct control::Pid *pid,
               boost::asio::serial_port *ser,
               utils::SafeQueue<struct slam::slamPose> &queue,
               const struct tarzan::DiffDriveTwist &cmd,
               slam::slamHandle *sh)
      : nav_ctx(nav), pid_ctx(pid), serial(ser), poseQueue(queue),
        drive_cmd(cmd), slam_handler(sh) {};

  bool init() {
    std::unique_lock<std::mutex> lk(map_sync.mtx);
    map_sync.cv.wait(lk, [] { return map_sync.flag; });
    lk.unlock();
    if (slam::getStatus(slam_handler) != "Tracking") {
      spdlog::error("FSM init: SLAM not tracking");
      return false;
    }
    if (!serial->is_open()) {
      spdlog::error("FSM init: serial not open");
      return false;
    }
    spdlog::info("FSM init: all checks passed");
    return true;
  }

  void stop() {
    tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(0.0f, 0.0f);
    serial::Error err = serial::write_msg<tarzan::tarzan_msg>(
        serial, msg, tarzan::TARZAN_MSG_LEN);
    if (err != serial::Error::WriteSuccess)
      spdlog::error("stop: {}", serial::get_error(err));
  }

  PlanResult plan() {
    slam::slamPose pose;
    if (!poseQueue.consume(pose)) {
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

    if (target_gnss.empty())
      return PlanResult::AT_GOAL;

    double target_latitude = target_gnss.front().first;
    double target_longitude = target_gnss.front().second;

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

  int navGPS() {
    spdlog::info("State Machine : Executing navGPS");

    std::unique_lock<std::mutex> lock(map_sync.mtx);
    map_sync.cv.wait(lock, [] { return map_sync.flag; });
    lock.unlock();

    while (!target_gnss.empty()) {
      PlanResult pr = plan();
      if (pr == PlanResult::AT_GOAL) {
        target_gnss.pop();
        continue;
      }
      if (pr == PlanResult::FAULT)
        return 0;
      traverse();
    }
    spdlog::info("State Machine: navGPS complete");
    return 1;
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
      "angular", po::value<float>(), "max angular velocity");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  /* zmq vars */
  zmq::context_t ctx(1);
  zmq::socket_t pub(ctx, ZMQ_PUB);
  zmq::socket_t slam_sub(ctx, ZMQ_SUB);
  zmq::socket_t mapping_sub(ctx, ZMQ_SUB);

  /* realsense vars */
  struct utils::rs_config realsense_config{
      .height = 640, .width = 480, .fps = 30, .enable_imu = false};
  struct utils::rs_handler *rs_ptr;

  /* slam vars */
  struct slam::slamHandle *slam_handler = new slam::slamHandle();
  utils::SafeQueue<struct slam::slamPose> poseQueue;

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

  /* CONFIGURING PERIPHERALS */
  spdlog::info("Configuring Rover Peripherals...");

  /* CONFIGURING REALSENSE */
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

  StateMachine sm(
      nav_ctx, pid_ctx, serial, poseQueue,
      tarzan::DiffDriveTwist{vm["linear"].as<float>(), vm["angular"].as<float>()},
      slam_handler);

  /* CONFIGURING ZMQ SOCKETS */
  try {
    pub.bind("inproc://realsense");
  } catch (zmq::error_t &e) {
    spdlog::error(e.what());
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  try {
    slam_sub.connect("inproc://realsense");
    slam_sub.set(zmq::sockopt::subscribe, topic_color);
    slam_sub.set(zmq::sockopt::subscribe, topic_depth);
    slam_sub.set(zmq::sockopt::subscribe, topic_timestamp);
  } catch (zmq::error_t &e) {
    spdlog::error(e.what());
  }

  try {
    mapping_sub.connect("inproc://realsense");
    mapping_sub.set(zmq::sockopt::subscribe, topic_pointcloud);
  } catch (zmq::error_t &e) {
    spdlog::error(e.what());
  }

  /* CONFIGURING RERUN */
  rec.connect_grpc(rerun_url).exit_on_failure();

  /* LAUNCHING BACKGROUND THREADS */
  std::thread capture_thread(capture_frame, rs_ptr, std::ref(pub));
  std::thread localize_thread(localize, slam_handler, std::ref(poseQueue),
                              std::ref(slam_sub), std::ref(rec),
                              realsense_config);
  std::thread mapping_thread(mapping, nav_ctx, std::ref(poseQueue),
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
      if (line.empty()) continue;
      std::istringstream ss(line);
      std::string lat_str, lon_str;
      if (std::getline(ss, lat_str, ' ') && std::getline(ss, lon_str)) {
        try {
          double lat = std::stod(lat_str);
          double lon = std::stod(lon_str);
          sm.target_gnss.push({lat, lon});
        } catch (const std::exception &e) {
          spdlog::warn(std::format("Skipping malformed GNSS line: {}", line));
        }
      }
    }
    gnss_file.close();
  }
  spdlog::info(std::format("Loaded {} GNSS waypoints", sm.target_gnss.size()));

  sm.navGPS();

  /* CLEANUP */
  capture_thread.join();
  localize_thread.join();
  mapping_thread.join();

  delete slam_handler;
  delete nav_ctx;
  delete pid_ctx;
  utils::destroyHandle(rs_ptr);
  serial::close(serial);

  return 0;
}
