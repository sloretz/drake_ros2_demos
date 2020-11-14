#ifndef DRAKE_ROS_SYSTEMS__DRAKE_ROS_HPP_
#define DRAKE_ROS_SYSTEMS__DRAKE_ROS_HPP_

#include <rosidl_runtime_c/message_type_support_struct.h>

#include <drake_ros_systems/drake_ros_interface.hpp>

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/qos.hpp>
#include <rclcpp/serialized_message.hpp>

namespace drake_ros_systems
{
/// PIMPL forward declarations
class DrakeRosPrivate;
class Publisher;
class Subscription;

/// System that abstracts working with ROS
class DrakeRos final: public DrakeRosInterface
{
public:
  DrakeRos();

  virtual ~DrakeRos();

  std::unique_ptr<Publisher>
  create_publisher(
    const rosidl_message_type_support_t & ts,
    const std::string & topic_name,
    const rclcpp::QoS & qos) final;

  virtual
  std::shared_ptr<Subscription>
  create_subscription(
    const rosidl_message_type_support_t & ts,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback) final;

  void
  spin(
    int timeout_millis) final;

private:
  std::unique_ptr<DrakeRosPrivate> impl_;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__DRAKE_ROS_HPP_
