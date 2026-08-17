#include "pid.hpp"
#include <chrono>
#include <cmath>

namespace control {

struct Pid *initPid(double p, double i, double d) {
  struct Pid *pid = new struct Pid();

  pid->gains = new struct Pid::Gains();
  pid->errors = new struct Pid::Errors();

  pid->gains->p = p;
  pid->gains->i = i;
  pid->gains->d = d;

  pid->errors->p_error_last = 0;
  pid->errors->p_error = 0;
  pid->errors->i_error = 0;
  pid->errors->d_error = 0;
  pid->errors->cmd = 0;
  pid->errors->error_dot = 0;

  return pid;
}

void resetPid(struct Pid *pid) {
  pid->errors->p_error_last = 0.0;
  pid->errors->p_error = 0.0;
  pid->errors->i_error = 0.0;
  pid->errors->d_error = 0.0;
  pid->errors->cmd = 0.0;
}

double computeCommandErrorDot(struct Pid *pid, double error, double error_dot,
                              uint64_t dt) {

  double p_term, d_term, i_term;
  pid->errors->p_error = error; // this is error = target - state
  pid->errors->d_error = error_dot;

  if (dt == 0 || std::isnan(error) || std::isinf(error) ||
      std::isnan(error_dot) || std::isinf(error_dot)) {
    return 0.0;
  }

  // Calculate proportional contribution to command
  p_term = pid->gains->p * pid->errors->p_error;

  // Calculate the integral of the position error
  pid->errors->i_error +=
      (static_cast<double>(dt) / 1e9) * pid->errors->p_error;

  // Calculate integral contribution to command
  i_term = pid->gains->i * pid->errors->i_error;

  // Calculate derivative contribution to command
  d_term = pid->gains->d * pid->errors->d_error;

  // Compute the command
  pid->errors->cmd = p_term + i_term + d_term;

  return pid->errors->cmd;
}

double computeCommand(struct Pid *pid, double error, uint64_t dt) {
  if (dt == 0 || std::isnan(error) || std::isinf(error)) {
    return 0.0;
  }

  pid->errors->error_dot = pid->errors->d_error;

  // Calculate the derivative error
  pid->errors->error_dot =
      (error - pid->errors->p_error_last) / (static_cast<double>(dt) / 1e9);
  pid->errors->p_error_last = error;

  return computeCommandErrorDot(pid, error, pid->errors->error_dot, dt);
}
} // namespace control

#ifdef PID_TEST_CPP
#include <Eigen/Dense>
#include <iostream>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>

#include "pid.hpp"
#include "serial.hpp"
#include "slam.hpp"
#include "utils.hpp"

int main(int argc, char *argv[]) {

  // slam vars
  struct slam::slamHandle *slam_handler = new slam::slamHandle();
  Eigen::Matrix<double, 4, 4> current_pose;
  Eigen::Matrix<double, 4, 4> res;
  struct slam::rawColorDepthPair *frame_raw = new slam::rawColorDepthPair();

  // rs vars
  struct utils::rs_config realsense_config{
      .height = 640, .width = 480, .fps = 30, .enable_imu = false};
  struct utils::rs_handler *rs_ptr;
  rs2::frame frame;
  rs_ptr = utils::setupRealsense(realsense_config);
  if (not rs_ptr) {
    std::cout << " Unable to setup Realsense ";
    return -1;
  }

  // com vars
  std::string port = argv[1];
  boost::asio::io_context io;
  boost::asio::serial_port *nucleo = serial::open(io, port, 9600);

  // pid vars
  double p = std::stod(argv[2]);
  double i = std::stod(argv[3]);
  double d = std::stod(argv[4]);
  struct control::Pid *pid = control::initPid(p, i, d);

  float target = std::stod(argv[5]);
  float linear_x = 0;
  float angular_z;

  uint64_t previous = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
  while (true) {
    // fetch frames from realsense
    frame = rs_ptr->frame_q.wait_for_frame();

    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      auto aligned_frames = rs_ptr->align.process(fs);

      rs2::video_frame colorFrame = aligned_frames.first(RS2_STREAM_COLOR);
      rs2::video_frame depthFrame = aligned_frames.get_depth_frame();

      // get raw frame data
      frame_raw->colorFrame = colorFrame.get_data();
      frame_raw->depthFrame = depthFrame.get_data();
      frame_raw->timestamp = fs.get_timestamp();

      // get current pose
      struct slam::RGBDFrame *frame_cv = slam::getColorDepthPair(frame_raw);
      res = slam::runLocalization(frame_cv, slam_handler);
      current_pose = utils::T_camera_base * res;
      auto translations = current_pose.col(3);
      double yaw = (double)slam::yawfromPose(current_pose);

      uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
      if (std::abs(target - yaw) < 0.01) { // tolerance = 0.05
        angular_z = 0.0;
      } else {
        angular_z = std::clamp(
            control::computeCommand(pid, target - yaw, now - previous), -5.5,
            5.5);
      }
      std::cout << "Yaw : " << yaw << " Angular_Vel : " << angular_z
                << std::endl;
      previous = now;
      tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(linear_x, angular_z);

      std::string err =
          serial::get_error(serial::write_msg<struct tarzan::tarzan_msg>(
              nucleo, msg, tarzan::TARZAN_MSG_LEN));
      std::cout << "Serial Port : " << err << std::endl;
    }
  }
  serial::close(nucleo);
}
#endif
