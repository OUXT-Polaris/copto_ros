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

#include <copto_mix/mix_component.hpp>
#include <copto_pid/pid_component.hpp>
#include <copto_quat/quat_component.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto copto_pid = std::make_shared<copto_pid::PIDComponent>(options);
  auto copto_mix = std::make_shared<copto_mix::MIXComponent>(options);
  ;
  auto copto_quat = std::make_shared<copto_quat::QUATComponent>(options);

  exec.add_node(copto_pid);
  exec.add_node(copto_mix);
  exec.add_node(copto_quat);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}