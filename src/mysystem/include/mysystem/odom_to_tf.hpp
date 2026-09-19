#ifndef MYSYSTEM__ODOM_TO_TF_HPP_
#define MYSYSTEM__ODOM_TO_TF_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace mysystem
{

geometry_msgs::msg::TransformStamped odometry_to_transform(
  const nav_msgs::msg::Odometry & odometry);

}  // namespace mysystem

#endif  // MYSYSTEM__ODOM_TO_TF_HPP_
