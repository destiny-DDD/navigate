import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'world.world'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'sim_worlds_launch.py'
PACKAGE_XML_PATH = PACKAGE_ROOT / 'package.xml'
URDF_PATH = PACKAGE_ROOT / 'urdf' / 'robot.urdf'
BRIDGE_CONFIG_PATH = PACKAGE_ROOT / 'config' / 'bridge.yaml'


class TestSimWorldAssets(unittest.TestCase):
    def test_world_has_realtime_physics_ground_and_walls(self):
        root = ET.parse(WORLD_PATH).getroot()
        world = root.find('world')

        self.assertIsNotNone(world)
        self.assertEqual(world.get('name'), 'test_office')
        self.assertEqual(
            world.findtext('physics/real_time_update_rate'), '500'
        )
        self.assertIn('sun', {light.get('name') for light in world.findall('light')})
        self.assertEqual(
            {model.get('name') for model in world.findall('model')},
            {'ground_plane', 'north_wall', 'south_wall', 'east_wall', 'west_wall'},
        )

    def test_launch_description_includes_gazebo_simulator(self):
        spec = importlib.util.spec_from_file_location('sim_worlds_launch', LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        description = module.generate_launch_description()

        self.assertTrue(
            any(
                isinstance(action, IncludeLaunchDescription)
                for action in description.entities
            )
        )

    def test_launch_description_spawns_robot_from_urdf(self):
        spec = importlib.util.spec_from_file_location('sim_worlds_launch', LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        description = module.generate_launch_description()

        self.assertTrue(
            any(
                isinstance(action, Node)
                and action.node_package == 'ros_gz_sim'
                and action.node_executable == 'create'
                for action in description.entities
            )
        )

    def test_launch_description_bridges_control_and_starts_odom_tf(self):
        spec = importlib.util.spec_from_file_location('sim_worlds_launch', LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        description = module.generate_launch_description()

        self.assertTrue(
            any(
                isinstance(action, Node)
                and action.node_package == 'ros_gz_bridge'
                and action.node_executable == 'parameter_bridge'
                for action in description.entities
            )
        )

    def test_bridge_config_declares_clock_control_and_odometry(self):
        bridge_config = yaml.safe_load(BRIDGE_CONFIG_PATH.read_text())

        self.assertEqual(
            {
                (entry['ros_topic_name'], entry['gz_topic_name'], entry['direction'])
                for entry in bridge_config
            },
            {
                ('/clock', '/clock', 'GZ_TO_ROS'),
                ('/cmd_vel', '/model/robot/cmd_vel', 'ROS_TO_GZ'),
                ('/odom', '/model/robot/odometry', 'GZ_TO_ROS'),
            },
        )

    def test_launch_loads_bridge_config_file(self):
        spec = importlib.util.spec_from_file_location('sim_worlds_launch', LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        description = module.generate_launch_description()
        bridge = next(
            action
            for action in description.entities
            if isinstance(action, Node)
            and action.node_package == 'ros_gz_bridge'
        )

        parameter_mappings = bridge._Node__parameters
        self.assertEqual(len(parameter_mappings), 1)
        config_file = next(iter(parameter_mappings[0].items()))
        self.assertEqual(
            perform_substitutions(LaunchContext(), config_file[0]), 'config_file'
        )
        self.assertIn(
            '/config/bridge.yaml',
            perform_substitutions(LaunchContext(), config_file[1]),
        )
        self.assertTrue(
            any(
                isinstance(action, Node)
                and action.node_package == 'mysystem'
                and action.node_executable == 'odom_to_tf'
                for action in description.entities
            )
        )

    def test_package_exports_mesh_resource_path_for_gazebo(self):
        root = ET.parse(PACKAGE_XML_PATH).getroot()
        gazebo_export = root.find('export/gazebo_ros')

        self.assertIsNotNone(gazebo_export)
        self.assertEqual(gazebo_export.get('gazebo_model_path'), '${prefix}/..')

    def test_robot_urdf_configures_mecanum_drive(self):
        root = ET.parse(URDF_PATH).getroot()
        plugin = root.find('gazebo/plugin')

        self.assertIsNotNone(plugin)
        if plugin is None:
            return

        self.assertEqual(plugin.get('filename'), 'gz-sim-mecanum-drive-system')
        self.assertEqual(plugin.get('name'), 'gz::sim::systems::MecanumDrive')
        self.assertEqual(plugin.findtext('front_left_joint'), 'joint_wheel_1')
        self.assertEqual(plugin.findtext('front_right_joint'), 'joint_wheel_4')
        self.assertEqual(plugin.findtext('back_left_joint'), 'joint_wheel_2')
        self.assertEqual(plugin.findtext('back_right_joint'), 'joint_wheel_3')
        self.assertEqual(plugin.findtext('wheel_radius'), '0.07618')
        self.assertEqual(plugin.findtext('wheel_separation'), '0.321')
        self.assertEqual(plugin.findtext('wheelbase'), '0.22')
        self.assertEqual(plugin.findtext('topic'), '/model/robot/cmd_vel')
        self.assertEqual(plugin.findtext('odom_topic'), '/model/robot/odometry')
        self.assertEqual(plugin.findtext('frame_id'), 'odom')
        self.assertEqual(plugin.findtext('child_frame_id'), 'imu')

    def test_package_declares_mecanum_runtime_dependencies(self):
        root = ET.parse(PACKAGE_XML_PATH).getroot()
        dependencies = {dependency.text for dependency in root.findall('depend')}

        self.assertTrue(
            {
                'geometry_msgs',
                'nav_msgs',
                'tf2_ros',
            }.issubset(dependencies)
        )


if __name__ == '__main__':
    unittest.main()
