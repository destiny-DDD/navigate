#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "mysystem/odom_to_tf.hpp"

class OdomToTf : public rclcpp::Node
{
public:
  OdomToTf()
  : Node("odom_to_tf"), broadcaster_(this)
  {
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr odometry) {
        broadcaster_.sendTransform(mysystem::odometry_to_transform(*odometry));
      });
  }

private:
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTf>());
  rclcpp::shutdown();
  return 0;
}
