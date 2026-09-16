import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
	package_share = get_package_share_directory("mysystem")
	urdf_file = os.path.join(package_share, "urdf", "robot.urdf")

	use_gui = LaunchConfiguration("use_gui")
	use_foxglove = LaunchConfiguration("use_foxglove")
	use_sim_time = LaunchConfiguration("use_sim_time")

	robot_description = ParameterValue(
		Command(["cat ", urdf_file]),
		value_type=str,
	)

	declare_use_gui = DeclareLaunchArgument(
		"use_gui",
		default_value="false",
		description="Start the joint state publisher GUI",
	)
	declare_use_foxglove = DeclareLaunchArgument(
		"use_foxglove",
		default_value="true",
		description="Start Foxglove Bridge",
	)
	declare_use_sim_time = DeclareLaunchArgument(
		"use_sim_time",
		default_value="false",
		description="Use simulation time",
	)

	robot_state_publisher = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		name="robot_state_publisher",
		output="screen",
		parameters=[
			{
				"robot_description": robot_description,
				"use_sim_time": use_sim_time,
			}
		],
	)

	joint_state_publisher = Node(
		package="joint_state_publisher",
		executable="joint_state_publisher",
		name="joint_state_publisher",
		output="screen",
		condition=UnlessCondition(use_gui),
		parameters=[
			{
				"robot_description": robot_description,
				"use_sim_time": use_sim_time,
			}
		],
	)

	joint_state_publisher_gui = Node(
		package="joint_state_publisher_gui",
		executable="joint_state_publisher_gui",
		name="joint_state_publisher_gui",
		output="screen",
		condition=IfCondition(use_gui),
		parameters=[
			{
				"robot_description": robot_description,
				"use_sim_time": use_sim_time,
			}
		],
	)

	foxglove_bridge = Node(
		package="foxglove_bridge",
		executable="foxglove_bridge",
		name="foxglove_bridge",
		output="screen",
		condition=IfCondition(use_foxglove),
		parameters=[
			{
				"port": 8765,
				"use_sim_time": use_sim_time,
			}
		],
	)

	ld = LaunchDescription()
	ld.add_action(declare_use_gui)
	ld.add_action(declare_use_foxglove)
	ld.add_action(declare_use_sim_time)
	ld.add_action(robot_state_publisher)
	ld.add_action(joint_state_publisher)
	ld.add_action(joint_state_publisher_gui)
	ld.add_action(foxglove_bridge)

	return ld
