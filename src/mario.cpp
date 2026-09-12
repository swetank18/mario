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
#include "slam/backend.hpp"
#include "slam/stella_backend.hpp"
#include "utils.hpp"
#include "yolo.hpp"

namespace po = boost::program_options;

#define EARTH_RADIUS 6378137.0

/* zmq topic names */
const std::string topic_color = "color_frame";
const std::string topic_depth = "depth_frame";
const std::string topic_timestamp = "timestamp";
const std::string topic_pointcloud = "pointcloud";
const std::string topic_lidar = "lidar_pointcloud";

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
auto localize(slam::Backend &backend,
              utils::SharedLatest<struct slam::Pose> &poseState,
              zmq::socket_t &sub, const rerun::RecordingStream &rec,
              utils::rs_config realsense_config) -> void {

  std::vector<zmq::message_t> colorFrameMsg;
  std::vector<zmq::message_t> depthFrameMsg;
  std::vector<zmq::message_t> timestampMsg;
  zmq::recv_result_t result_color;
  zmq::recv_result_t result_depth;
  zmq::recv_result_t result_timestamp;

  slam::Frame frame;
  /* rs_config's `height` and `width` are swapped relative to how they are
     handed to enable_stream -- a known bug, on the out-of-scope list in
     tweaks/REFACTOR_NAV.md. `height` is the field holding 640, so it is the
     image width. This reproduces the size slam.cpp used to hardcode. */
  frame.width = realsense_config.height;
  frame.height = realsense_config.width;

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

    /* Borrowed for the duration of track(), not owned. The old path
       heap-allocated a rawColorDepthPair and an RGBDFrame every iteration and
       freed neither. */
    frame.first = colorFrameMsg[1].data();
    frame.second = depthFrameMsg[1].data();
    frame.timestamp = std::stod(timestampMsg[1].to_string());

    struct slam::Pose pose;
    if (backend.track(frame, pose)) {
      poseState.set(pose);

      std::string coordinates =
          std::format("x: {} y: {} yaw: {}", pose.x, pose.y, pose.yaw);
      rec.log("SlamPose", rerun::TextLog(coordinates));
    }

    colorFrameMsg.clear();
    depthFrameMsg.clear();
    timestampMsg.clear();
  }
}

/* function to create gridmap  */
auto mapping(nav::OccupancyMap &occupancy_map,
             utils::SharedLatest<struct slam::Pose> &poseState,
             zmq::socket_t &sub, const rerun::RecordingStream &rec,
             utils::rs_config realsense_config, bool use_lidar) -> void {

  struct slam::Pose pose;
  std::vector<zmq::message_t> pointcloud_msg;
  zmq::recv_result_t result_pointcloud;
  std::vector<float> points_buffer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  /* Where the depth camera sits on the rover, in the base FLU frame. Constant
     for the life of the run, so it is built once out here.

     Identity first, not default-constructed: an Affine3d leaves its bottom row
     uninitialised, and transformPointCloud reads the whole 4x4.

     The translation is the part that used to be missing. T_camera_base only
     ever carried the optical->FLU rotation, so the cloud was folded in as
     though the camera were bolted to the ground at the base origin: flat
     ground came out at z = -0.62 m against a -0.25 m ditch threshold, and
     every square metre the camera could see was mapped as a negative
     obstacle. The 0.40 m of forward offset was missing too, which put every
     rock 40 cm nearer than it really was. */
  Eigen::Affine3d T_base_sensor = Eigen::Affine3d::Identity();
  if (use_lidar) {
    /* A lidar cloud is already in the base FLU convention -- x forward, y
       left, z up -- so the mount is the whole transform and the rotation
       stays identity. Applying the optical->FLU rotation here as well would
       turn the scan on its side. */
    T_base_sensor.translation() =
        Eigen::Vector3d(occupancy_map.params().lidar_offset[0],
                        occupancy_map.params().lidar_offset[1],
                        occupancy_map.params().lidar_offset[2]);
    spdlog::info("GridMap: building from the lidar, mount ({:.2f}, {:.2f}, "
                 "{:.2f})",
                 T_base_sensor.translation().x(),
                 T_base_sensor.translation().y(),
                 T_base_sensor.translation().z());
  } else {
    T_base_sensor.linear() = utils::T_camera_base.block<3, 3>(0, 0);
    T_base_sensor.translation() =
        Eigen::Vector3d(occupancy_map.params().sensor_offset[0],
                        occupancy_map.params().sensor_offset[1],
                        occupancy_map.params().sensor_offset[2]);
    spdlog::info("GridMap: building from the depth camera, mount ({:.2f}, "
                 "{:.2f}, {:.2f})",
                 T_base_sensor.translation().x(),
                 T_base_sensor.translation().y(),
                 T_base_sensor.translation().z());
  }

  Eigen::Affine3d T_world_base = Eigen::Affine3d::Identity();

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

    /* Only the rover's own pose changes per frame. Dropping the yaw term
       stamps obstacles into the map as if the rover never turned. */
    T_world_base.linear() =
        Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    T_world_base.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);

    /* Follow the rover before folding the cloud in, so the points land on a
       grid that still covers it. The map used to be a fixed box around the
       SLAM origin, which put the rover off its own map once it got further
       than dim/2 -- 10 m with the sim config, against a first waypoint 14 m
       out. */
    occupancy_map.recenter(pose.x, pose.y);

    /* Kept apart on purpose: the map applies the config's rover-relative
       passthrough limits in the base frame, between the two. */
    occupancy_map.integrate(cloud, T_world_base.matrix(),
                            T_base_sensor.matrix());
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
                      slam::Pose &pose) -> std::tuple<float, float> {
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
    return backend_.status() == slam::Tracking::Tracking;
  }

  auto clear_path() -> void { current_path.reset(); }

  auto on_mission_done() -> void {
    spdlog::info("State Machine: mission complete");
  }

  auto on_waypoint_skipped() -> void {
    spdlog::error(std::format("State Machine: giving up on waypoint lat={} "
                              "lon={} -- no path after every recovery",
                              current_waypoint.lat, current_waypoint.lon));
    plan_failures_ = 0;
  }

  /* One step of the SLAM recovery sweep: a slow turn on the spot, so the
     camera passes over views the backend may recognise. Slow, because
     relocalisation needs sharp frames and the rover may be next to whatever
     it lost tracking against. */
  auto slam_recover_step() -> bool {
    tarzan::tarzan_msg msg =
        tarzan::get_tarzan_msg(0.0f, (float)(drive_cmd.angular_z * 0.25));
    return serial::write_msg<struct tarzan::tarzan_msg>(
               serial, msg, tarzan::TARZAN_MSG_LEN) ==
           serial::Error::WriteSuccess;
  }

  /* Throw away the SLAM map and the occupancy map together: the new pose
     frame has a new origin, so a map built in the old one is not merely
     stale. The mission survives this because every plan() re-projects its
     goal from the GPS fix and compass heading of the moment; nothing
     upstream of here remembers the old frame. */
  auto reset_slam() -> void {
    spdlog::warn("State Machine: SLAM did not relocalise -- resetting the "
                 "backend and clearing the map");
    poseState.invalidate();
    backend_.reset();
    map_.clear();
    current_path.reset();
    obstacle_trips_ = 0;
  }

  /* Change something before plan() is asked again. Attempt 1 gives the map
     a moment to fill in; after that, back out if the rover is boxed in, and
     otherwise turn so the camera puts new ground on the map. plan() also
     brings the local goal in by half for every failure in a row, so a goal
     that landed in a pocket of rocks is retried nearer and nearer. */
  auto recover_plan(int attempt) -> bool {
    slam::Pose pose;
    const bool boxed_in =
        poseState.get(pose) &&
        map_.clearance(pose.x, pose.y) < planner_params_.safety_margin;
    spdlog::warn("State Machine: plan recovery {} ({})", attempt,
                 boxed_in ? "boxed in, backing out"
                 : attempt == 1 ? "waiting for the map"
                                : "turning to look around");
    if (attempt == 1 && !boxed_in) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      return true;
    }
    if (boxed_in)
      return escape_stall();

    const auto turn_until =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
    while (std::chrono::steady_clock::now() < turn_until) {
      tarzan::tarzan_msg msg =
          tarzan::get_tarzan_msg(0.0f, (float)(drive_cmd.angular_z * 0.5));
      if (serial::write_msg<struct tarzan::tarzan_msg>(
              serial, msg, tarzan::TARZAN_MSG_LEN) !=
          serial::Error::WriteSuccess)
        return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    stop_motors();
    return true;
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

  /* Escape manoeuvre for a rover that has stopped making progress. Backing
     off and turning is the only thing that changes what the camera can see,
     and therefore the only thing that can change the plan -- RECOVER_OBSTACLE
     pauses for half a second and replans from the same pose against the same
     map, which is how the rover spent 200 s replanning the same path into the
     same rock. */
  auto escape_stall() -> bool {
    spdlog::warn("State Machine: no progress -- backing off");
    const auto reverse_until =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
    while (std::chrono::steady_clock::now() < reverse_until) {
      tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(-0.25f, 0.0f);
      if (serial::write_msg<struct tarzan::tarzan_msg>(
              serial, msg, tarzan::TARZAN_MSG_LEN) !=
          serial::Error::WriteSuccess)
        return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto turn_until =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(900);
    while (std::chrono::steady_clock::now() < turn_until) {
      tarzan::tarzan_msg msg =
          tarzan::get_tarzan_msg(0.0f, (float)(drive_cmd.angular_z * 0.6));
      if (serial::write_msg<struct tarzan::tarzan_msg>(
              serial, msg, tarzan::TARZAN_MSG_LEN) !=
          serial::Error::WriteSuccess)
        return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    stop_motors();
    return true;
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
    const double max_linear = drive_cmd.linear_x;
    double angular_z = 0.0;
    /* The configured maximum, held separately. Clamping against angular_z
       itself uses the previous output as this iteration's bound, which
       ratchets the command down to zero over a few waypoints. */
    const double max_angular = drive_cmd.angular_z;

    /* A waypoint counts as captured inside this, rather than inside one cell.
       The pose updates at 10 Hz and the rover covers 6 cm between updates at
       0.6 m/s, so a 10 cm capture radius is a coin toss -- miss it and the
       rover turns round to fetch a waypoint it has already driven through.
       Observed: waypoint 4 overshot by 0.5 m, a U-turn, and the rest of the
       leg driven backwards. */
    const double capture_radius = std::max(2.5 * map_.resolution(), 0.25);
    /* Beyond the capture radius, a waypoint behind the rover's beam is still
       done with: chasing it means turning round. This is the test the capture
       radius cannot do on its own, and it is what actually stops the U-turn. */
    const double behind_slack = 1.5;
    /* Stop driving forward and turn on the spot past this heading error.
       Driving and turning at once is what turns a heading error into an arc
       that overshoots the next waypoint. */
    const double turn_in_place_error = 0.6;
    /* Ease off within this distance of the waypoint being chased. */
    const double slow_radius = 0.6;

    /* Stall detection. The rover sat at one position to the centimetre for
       200 s across seven plan/traverse cycles, each ending in a 30 s
       REPLAN_TIMEOUT that mapped straight back to PLAN_PATH -- a stuck rover
       looked exactly like a slow one, forever. */
    const double stall_distance = 0.08;
    const auto stall_window = std::chrono::seconds(4);
    auto stall_epoch = std::chrono::steady_clock::now();
    double stall_x = 0.0, stall_y = 0.0;
    bool stall_ref_valid = false;

    while (waypoint_idx < path.size()) {
      if (std::chrono::steady_clock::now() > deadline)
        return TraverseResult::REPLAN_TIMEOUT;

      if (backend_.status() != slam::Tracking::Tracking)
        return TraverseResult::FAULT_SLAM;

      slam::Pose pose;
      if (!poseState.get(pose)) {
        spdlog::error("State Machine : No pose available yet");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      /* the pose read no longer blocks on a queue, so pace the control loop
         here instead — otherwise this spins and floods the serial port */
      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      /* The trip wire, in metres, from the same measured rover radius the
         planner keeps its paths clear by. It used to be three grid cells --
         0.30 m against a rover 0.53 m in radius -- so the rover could be in
         contact with a rock and still be told it had clearance. */
      if (map_.clearance(pose.x, pose.y) < planner_params_.rover_radius) {
        /* Tripping the wire leaves traverse(), and RECOVER_OBSTACLE pauses for
           half a second and plans again from the same pose against the same
           map -- which produces the same path, which trips the wire again.
           51 recoveries in one 5-minute run, none of them going anywhere.

           The stall detector below cannot see it, because every trip resets
           the loop it lives in. So the count is kept on the object instead:
           three trips inside the same 0.4 m and the rover backs out rather
           than replanning into the same rock a fourth time. */
        if (std::hypot(pose.x - last_trip_x_, pose.y - last_trip_y_) < 0.4)
          obstacle_trips_++;
        else
          obstacle_trips_ = 1;
        last_trip_x_ = pose.x;
        last_trip_y_ = pose.y;

        if (obstacle_trips_ >= 3) {
          spdlog::warn("State Machine: {} obstacle trips at ({:.2f}, {:.2f}) "
                       "-- backing out instead of replanning",
                       obstacle_trips_, pose.x, pose.y);
          obstacle_trips_ = 0;
          if (!escape_stall())
            return TraverseResult::FAULT_SERIAL;
        }
        return TraverseResult::REPLAN_OBSTACLE;
      }

      /* Has the rover actually moved? Anything that stops it -- wedged on a
         rock, a wheel dug in, a command that never reached the motors -- looks
         the same from here, and all of them need the same answer. */
      if (!stall_ref_valid) {
        stall_x = pose.x;
        stall_y = pose.y;
        stall_epoch = std::chrono::steady_clock::now();
        stall_ref_valid = true;
      } else if (std::hypot(pose.x - stall_x, pose.y - stall_y) >
                 stall_distance) {
        stall_x = pose.x;
        stall_y = pose.y;
        stall_epoch = std::chrono::steady_clock::now();
        /* Moving again, so whatever the last trip was about is behind us. */
        obstacle_trips_ = 0;
      } else if (std::chrono::steady_clock::now() - stall_epoch >
                 stall_window) {
        if (!escape_stall())
          return TraverseResult::FAULT_SERIAL;
        return TraverseResult::REPLAN_OBSTACLE;
      }

      double dx = path[waypoint_idx].x - pose.x;
      double dy = path[waypoint_idx].y - pose.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      /* Captured, or already passed. The dot product is the rover's heading
         against the direction to the waypoint: negative means it is behind
         the beam, and no amount of driving forward will reach it. */
      const double ahead = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
      if (distance < capture_radius ||
          (ahead < 0.0 && distance < behind_slack)) {
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

      /* Speed follows the heading error and the distance left: turn on the
         spot while badly aimed, ease off approaching the waypoint, full speed
         only when pointed at it with room ahead. A constant forward command
         is what made every heading correction an overshooting arc. */
      double linear_x = 0.0;
      if (std::abs(angular_error) < turn_in_place_error) {
        const double aim = 1.0 - std::abs(angular_error) / turn_in_place_error;
        const double approach = std::clamp(distance / slow_radius, 0.3, 1.0);
        linear_x = max_linear * aim * approach;
      }

      tarzan::tarzan_msg msg =
          tarzan::get_tarzan_msg((float)linear_x, (float)angular_z);
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
            rerun::Image::from_rgb24(
                rerun::Collection<uint8_t>::borrow(img_data,
                                                   frame.total() * 3),
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
            rerun::Image::from_rgb24(
                rerun::Collection<uint8_t>::borrow(img_data,
                                                   frame.total() * 3),
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
    slam::Pose pose;
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

    /* Every failure in a row halves the hop. A goal that landed behind a
       boulder field is retried a little nearer each time until the planner
       can see a way to it; a success puts the full reach back. */
    if (plan_failures_ > 0) {
      const double scale = std::pow(0.5, std::min(plan_failures_, 3));
      local_x = pose.x + (local_x - pose.x) * scale;
      local_y = pose.y + (local_y - pose.y) * scale;
    }

    auto path = planner_.plan({pose.x, pose.y}, {local_x, local_y});
    if (!path) {
      plan_failures_++;
      /* Which end refused matters and the old line did not say. A* will not
         leave a start cell inside safety_margin of an obstacle, and it will
         not accept a goal in one either, and the two want completely
         different answers -- back out, versus pick a different goal. */
      spdlog::warn("State Machine: no path from ({:.2f}, {:.2f}) "
                   "[clearance {:.2f} m, occupied={}] to ({:.2f}, {:.2f}) "
                   "[clearance {:.2f} m, occupied={}], margin {:.2f} m",
                   pose.x, pose.y, map_.clearance(pose.x, pose.y),
                   map_.occupied(pose.x, pose.y), local_x, local_y,
                   map_.clearance(local_x, local_y),
                   map_.occupied(local_x, local_y),
                   planner_params_.safety_margin);
      return PlanResult::NO_PATH;
    }

    plan_failures_ = 0;
    current_path = std::move(path);
    return PlanResult::PATH_FOUND;
  }

  /* Obstacle trips that produced no movement, and where the last one was.
     traverse() returns on every trip, so this cannot live in its loop. */
  int obstacle_trips_ = 0;
  double last_trip_x_ = 1e9, last_trip_y_ = 1e9;
  /* Consecutive plan() calls that found nothing; scales the next hop. */
  int plan_failures_ = 0;

public:
  nav::OccupancyMap &map_;
  nav::Planner &planner_;
  /* Kept alongside the planner because get_local_goal has to size the local
     goal against the same margin the planner will demand of the path. */
  nav::PlannerParams planner_params_;
  struct control::Pid *pid_ctx;
  boost::asio::serial_port *serial;
  utils::SharedLatest<struct slam::Pose> &poseState;
  tarzan::DiffDriveTwist drive_cmd;
  slam::Backend &backend_;
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
               utils::SharedLatest<struct slam::Pose> &pose_state,
               tarzan::DiffDriveTwist cmd, slam::Backend &backend,
               zmq::socket_t &csub, const rerun::RecordingStream &r,
               YOLO8Detector *det)
      : map_(map), planner_(planner), planner_params_(planner_params),
        pid_ctx(pid), serial(ser), poseState(pose_state), drive_cmd(cmd),
        backend_(backend), color_sub(csub), rec(r), detector(det) {};

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
      "stella_vslam ORB vocabulary")(
      "cloud_source", po::value<std::string>()->default_value("depth"),
      "which cloud the occupancy map is built from: 'depth' (the RGB-D "
      "camera's 55-degree wedge) or 'lidar' (the 360-degree scanner). The "
      "lidar needs sensor.lidar_offset in the gridmap config.");

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

  const std::string cloud_source = vm["cloud_source"].as<std::string>();
  if (cloud_source != "depth" && cloud_source != "lidar") {
    spdlog::error("--cloud_source must be 'depth' or 'lidar', got {}",
                  cloud_source);
    return -1;
  }
  const bool use_lidar = cloud_source == "lidar";

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

  /* slam vars.

     Held as a Backend, so which SLAM system this is stops being visible past
     this line. Swapping in AirSlamBackend is a change to this construction
     and to how the RealSense is configured -- backend->wants() says which
     pair of images it needs -- and to nothing else. See
     docs/AIRSLAM_INTEGRATION.md for what that costs today. */
  std::unique_ptr<slam::Backend> backend = std::make_unique<slam::StellaBackend>(
      vm["slam_config"].as<std::string>(), vm["slam_vocab"].as<std::string>());
  utils::SharedLatest<struct slam::Pose> poseState;

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
                  *backend, color_sub, rec, detector);

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
    mapping_sub.set(zmq::sockopt::subscribe,
                    use_lidar ? topic_lidar : topic_pointcloud);
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
  std::thread localize_thread(localize, std::ref(*backend), std::ref(poseState),
                              std::ref(slam_sub), std::ref(rec),
                              realsense_config);
  std::thread mapping_thread(mapping, std::ref(occupancy_map),
                             std::ref(poseState), std::ref(mapping_sub),
                             std::ref(rec), realsense_config, use_lidar);

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

  delete pid_ctx;
  delete detector;
  if (rs_ptr)
    utils::destroyHandle(rs_ptr);
  serial::close(serial);

  return 0;
}
