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

#ifndef COPTO_MIX__MIX_COMPONENT_HPP_
#define COPTO_MIX__MIX_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COPTO_MIX_MIX_COMPONENT_EXPORT __attribute__((dllexport))
#define COPTO_MIX_MIX_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define COPTO_MIX_MIX_COMPONENT_EXPORT __declspec(dllexport)
#define COPTO_MIX_MIX_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef COPTO_MIX_MIX_COMPONENT_BUILDING_DLL
#define COPTO_MIX_MIX_COMPONENT_PUBLIC COPTO_MIX__MIX_COMPONENT_EXPORT
#else
#define COPTO_MIX_MIX_COMPONENT_PUBLIC COPTO_MIX__MIX_COMPONENT_IMPORT
#endif
#define COPTO_MIX__MIX_COMPONENT_PUBLIC_TYPE COPTO_MIX__MIX_COMPONENT_PUBLIC
#define COPTO_MIX_MIX_COMPONENT_LOCAL
#else
#define COPTO_MIX_MIX_COMPONENT_EXPORT __attribute__((visibility("default")))
#define COPTO_MIX_MIX_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define COPTO_MIX_MIX_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define COPTO_MIX_MIX_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define COPTO_MIX_MIX_COMPONENT_PUBLIC
#define COPTO_MIX_MIX_COMPONENT_LOCAL
#endif
#define COPTO_MIX_MIX_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <Eigen/Dense>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float32_multi_array.hpp"

namespace copto_mix
{
class MIXComponent : public rclcpp::Node
{
public:
  COPTO_MIX_MIX_COMPONENT_PUBLIC
  explicit MIXComponent(const rclcpp::NodeOptions & options);
  bool initialized = false;

  Eigen::MatrixXd M;
  Eigen::MatrixXd A;

  Eigen::VectorXd T;
  Eigen::VectorXd u;

private:
  void CTLtopic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void init_M();
  void update();

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr CTLsubscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr PWMpublisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double l = 0.15;
  double c = 0.001;

  double a_r = 100;
  double a_p = 100;

  double a_y = 100;
  double a_th = 5;
};
}  // namespace copto_mix

#endif  // COPTO_MIX__MIX_COMPONENT_HPP_