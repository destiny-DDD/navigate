import importlib.util
import math
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

    def test_ground_is_a_thick_box_for_gpu_lidar_occlusion(self):
        root = ET.parse(WORLD_PATH).getroot()
        ground = root.find("world/model[@name='ground_plane']")
        self.assertIsNotNone(ground)
        if ground is None:
            return

        self.assertEqual(ground.findtext('pose'), '0 0 -0.05 0 0 0')
        self.assertEqual(
            ground.findtext('link/collision/geometry/box/size'), '10 10 0.1'
        )
        self.assertEqual(
            ground.findtext('link/visual/geometry/box/size'), '10 10 0.1'
        )

    def test_world_enables_gazebo_sensor_system(self):
        root = ET.parse(WORLD_PATH).getroot()
        plugins = root.findall('world/plugin')

        self.assertTrue(
            any(
                plugin.get('filename') == 'gz-sim-sensors-system'
                and plugin.get('name') == 'gz::sim::systems::Sensors'
                for plugin in plugins
            )
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
                (
                    '/joint_states',
                    '/world/test_office/model/robot/joint_state',
                    'GZ_TO_ROS',
                ),
                (
                    '/mid360/points',
                    '/model/robot/mid360/points/points',
                    'GZ_TO_ROS',
                ),
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

    def test_robot_urdf_publishes_wheel_joint_states(self):
        root = ET.parse(URDF_PATH).getroot()
        plugin = next(
            (
                gazebo_plugin
                for gazebo_plugin in root.findall('gazebo/plugin')
                if gazebo_plugin.get('name') == 'gz::sim::systems::JointStatePublisher'
            ),
            None,
        )

        self.assertIsNotNone(plugin)
        if plugin is None:
            return

        self.assertEqual(
            plugin.get('filename'), 'gz-sim-joint-state-publisher-system'
        )
        self.assertEqual(
            [joint.text for joint in plugin.findall('joint_name')],
            [
                'joint_wheel_1',
                'joint_wheel_2',
                'joint_wheel_3',
                'joint_wheel_4',
            ],
        )
        self.assertEqual(plugin.findtext('update_rate'), '50')

    def test_mecanum_wheels_use_directional_roller_friction(self):
        root = ET.parse(URDF_PATH).getroot()
        expected_fdir1 = {
            'wheel_1': '1 -1 0',
            'wheel_2': '1 1 0',
            'wheel_3': '1 -1 0',
            'wheel_4': '1 1 0',
        }

        for wheel_name, fdir1 in expected_fdir1.items():
            link = root.find(f"link[@name='{wheel_name}']")
            self.assertIsNotNone(link)
            if link is None:
                continue
            self.assertEqual(
                link.find('collision/geometry/sphere').get('radius'), '0.07618'
            )

            gazebo = root.find(f"gazebo[@reference='{wheel_name}']")
            self.assertIsNotNone(gazebo)
            if gazebo is None:
                continue
            friction = gazebo.find('collision/surface/friction/ode')
            self.assertIsNotNone(friction)
            if friction is None:
                continue
            self.assertEqual(friction.findtext('mu'), '1.0')
            self.assertEqual(friction.findtext('mu2'), '0.0')
            fdir1_element = friction.find('fdir1')
            self.assertIsNotNone(fdir1_element)
            if fdir1_element is None:
                continue
            self.assertEqual(fdir1_element.text, fdir1)
            self.assertEqual(
                fdir1_element.get('{http://gazebosim.org/schema}expressed_in'),
                'imu',
            )

    def test_mecanum_wheel_drive_axes_have_the_same_chassis_direction(self):
        """避免右侧轮因镜像安装而与左侧轮反向驱动。"""
        root = ET.parse(URDF_PATH).getroot()
        expected_axis_y = {
            'joint_wheel_1': 1.0,
            'joint_wheel_2': 1.0,
            'joint_wheel_3': 1.0,
            'joint_wheel_4': 1.0,
        }

        for joint_name, expected_y in expected_axis_y.items():
            joint = root.find(f"joint[@name='{joint_name}']")
            self.assertIsNotNone(joint)
            if joint is None:
                continue

            roll = float(joint.find('origin').get('rpy').split()[0])
            local_axis_z = float(joint.find('axis').get('xyz').split()[2])
            chassis_axis_y = -math.sin(roll) * local_axis_z
            self.assertAlmostEqual(chassis_axis_y, expected_y, places=6)

    def test_robot_urdf_fixes_upper_structure_to_the_base(self):
        root = ET.parse(URDF_PATH).getroot()
        upper_joint = root.find("joint[@name='joint_up']")

        self.assertIsNotNone(upper_joint)
        if upper_joint is None:
            return

        self.assertEqual(upper_joint.get('type'), 'fixed')
        self.assertEqual(upper_joint.find('parent').get('link'), 'up')
        self.assertEqual(upper_joint.find('child').get('link'), 'base_link')

    def test_robot_urdf_mounts_mid360_gpu_lidar_on_existing_sensor_link(self):
        root = ET.parse(URDF_PATH).getroot()
        self.assertIsNotNone(root.find("link[@name='livox_frame']"))
        sensors = {
            (gazebo.get('reference'), sensor.get('name')): sensor
            for gazebo in root.findall('gazebo')
            for sensor in gazebo.findall('sensor')
        }

        self.assertIn(('livox_frame', 'mid360_lidar'), sensors)
        self.assertEqual(sensors[('livox_frame', 'mid360_lidar')].get('type'), 'gpu_lidar')
        self.assertEqual(
            sensors[('livox_frame', 'mid360_lidar')].findtext('visualize'),
            'false',
        )
        self.assertEqual(
            sensors[('livox_frame', 'mid360_lidar')].findtext('topic'),
            '/model/robot/mid360/points',
        )
        self.assertEqual(
            sensors[('livox_frame', 'mid360_lidar')].findtext('frame_id'),
            'livox_frame',
        )
        self.assertEqual(
            sensors[('livox_frame', 'mid360_lidar')].findtext('lidar/range/min'),
            '0.1',
        )
        self.assertEqual(
            sensors[('livox_frame', 'mid360_lidar')].findtext('lidar/range/max'),
            '5.0',
        )
        self.assertEqual(
            sensors[('livox_frame', 'mid360_lidar')].findtext(
                'lidar/scan/vertical/min_angle'
            ),
            '0.52359878',
        )
        self.assertEqual(
            sensors[('livox_frame', 'mid360_lidar')].findtext(
                'lidar/scan/vertical/max_angle'
            ),
            '1.57079633',
        )

    def test_bridge_config_declares_mid360_simulated_3d_lidar(self):
        bridge_config = yaml.safe_load(BRIDGE_CONFIG_PATH.read_text())
        lidar_bridges = {
            (
                entry['ros_topic_name'],
                entry['gz_topic_name'],
                entry['ros_type_name'],
                entry['gz_type_name'],
            )
            for entry in bridge_config
            if entry['ros_topic_name'] == '/mid360/points'
        }

        self.assertEqual(
            lidar_bridges,
            {
                (
                    '/mid360/points',
                    '/model/robot/mid360/points/points',
                    'sensor_msgs/msg/PointCloud2',
                    'gz.msgs.PointCloudPacked',
                ),
            },
        )
        self.assertEqual(
            {
                entry['ros_topic_name']: entry['frame_id']
                for entry in bridge_config
                if entry['ros_topic_name'] == '/mid360/points'
            },
            {
                '/mid360/points': 'livox_frame',
            },
        )

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
