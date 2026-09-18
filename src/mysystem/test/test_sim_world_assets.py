import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'world.world'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'sim_worlds_launch.py'
PACKAGE_XML_PATH = PACKAGE_ROOT / 'package.xml'


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

    def test_package_exports_mesh_resource_path_for_gazebo(self):
        root = ET.parse(PACKAGE_XML_PATH).getroot()
        gazebo_export = root.find('export/gazebo_ros')

        self.assertIsNotNone(gazebo_export)
        self.assertEqual(gazebo_export.get('gazebo_model_path'), '${prefix}/..')


if __name__ == '__main__':
    unittest.main()
