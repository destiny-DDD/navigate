#include "mysystem/odom_to_tf.hpp"

namespace mysystem
{

geometry_msgs::msg::TransformStamped odometry_to_transform(
  const nav_msgs::msg::Odometry & odometry)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = odometry.header.stamp;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "imu";
  transform.transform.translation.x = odometry.pose.pose.position.x;
  transform.transform.translation.y = odometry.pose.pose.position.y;
  transform.transform.translation.z = odometry.pose.pose.position.z;
  transform.transform.rotation = odometry.pose.pose.orientation;
  return transform;
}

}  // namespace mysystem
