#include <copto_mix/mix_component.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<copto_mix::MIXComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}