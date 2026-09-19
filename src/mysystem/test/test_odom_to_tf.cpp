#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "mysystem/odom_to_tf.hpp"

TEST(OdomToTf, ConvertsOdometryIntoOdomToImuTransform)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp.sec = 12;
  odometry.header.stamp.nanosec = 34;
  odometry.pose.pose.position.x = 1.2;
  odometry.pose.pose.position.y = -0.4;
  odometry.pose.pose.position.z = 0.1;
  odometry.pose.pose.orientation.z = 0.5;
  odometry.pose.pose.orientation.w = 0.866;

  const auto transform = mysystem::odometry_to_transform(odometry);

  EXPECT_EQ(transform.header.stamp, odometry.header.stamp);
  EXPECT_EQ(transform.header.frame_id, "odom");
  EXPECT_EQ(transform.child_frame_id, "imu");
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, 1.2);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, -0.4);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, 0.1);
  EXPECT_DOUBLE_EQ(transform.transform.rotation.z, 0.5);
  EXPECT_DOUBLE_EQ(transform.transform.rotation.w, 0.866);
}
