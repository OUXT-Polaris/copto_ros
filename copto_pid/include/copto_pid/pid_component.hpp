// Copyright (c) 2022 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COPTO_PID__PID_COMPONENT_HPP_
#define COPTO_PID__PID_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COPTO_PID_PID_COMPONENT_EXPORT __attribute__((dllexport))
#define COPTO_PID_PID_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define COPTO_PID_PID_COMPONENT_EXPORT __declspec(dllexport)
#define COPTO_PID_PID_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef COPTO_PID_PID_COMPONENT_BUILDING_DLL
#define COPTO_PID_PID_COMPONENT_PUBLIC COPTO_PID__PID_COMPONENT_EXPORT
#else
#define COPTO_PID_PID_COMPONENT_PUBLIC COPTO_PID__PID_COMPONENT_IMPORT
#endif
#define COPTO_PID__PID_COMPONENT_PUBLIC_TYPE COPTO_PID__PID_COMPONENT_PUBLIC
#define COPTO_PID_PID_COMPONENT_LOCAL
#else
#define COPTO_PID_PID_COMPONENT_EXPORT __attribute__((visibility("default")))
#define COPTO_PID_PID_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define COPTO_PID_PID_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define COPTO_PID_PID_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define COPTO_PID_PID_COMPONENT_PUBLIC
#define COPTO_PID_PID_COMPONENT_LOCAL
#endif
#define COPTO_PID_PID_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace copto_pid
{
class PIDComponent : public rclcpp::Node
{
public:
  COPTO_PID_PID_COMPONENT_PUBLIC
  explicit PIDComponent(const rclcpp::NodeOptions & options);
  double roll_ = 0;
  double pitch_ = 0;
  double yaw_ = 0;
  double roll_old = 0;
  double pitch_old = 0;
  double yaw_old = 0;

  double rollrate_ = 0;
  double pitchrate_ = 0;
  double yawrate_ = 0;

  double THROTT_RANGE = 10 * 4;  //N
  double MIN_THROTT = 0.2 * 4;
  double MAX_YAWRATE = 20 * 3.14 / 180;  //rad/s
  double MAX_ROLL = 20 * 3.14 / 180;    // rad
  double MAX_PITCH = 20 * 3.14 / 180;   // rad

  double e_pitch_old;
  double e_roll_old;
  double e_yaw_old;

  double ctl_pitch, ctl_roll, ctl_thrott, ctl_yaw;

private:
  void POSEtopic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void JOYtopic_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void update();
  void getEulerRPY(
    const geometry_msgs::msg::Quaternion q, double & roll, double & pitch, double & yaw);
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr POSEsubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr JOYsubscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr CTLpublisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  double dt = 0.01;
  //pid gain
  double Kp_y = 0.18;
  double Kd_y = 0.0016;

  double Kp_r = 0.135;
  double Kd_r = 0.0016;

  double Kp_p = 0.135;
  double Kd_p = 0.0016;
};
}  // namespace copto_pid

#endif  // COPTO_PID__PID_COMPONENT_HPP_