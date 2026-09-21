import importlib.util
import pathlib
import unittest
from unittest.mock import patch

from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


LAUNCH_FILE = pathlib.Path(__file__).parents[1] / "launch" / "mycontrol_launch.py"


def load_launch_module():
    spec = importlib.util.spec_from_file_location("mycontrol_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MyControlLaunchTest(unittest.TestCase):
    def test_starts_livox_msg_driver_and_point_lio(self):
        module = load_launch_module()

        package_shares = {
            "livox_ros_driver2": "/share/livox_ros_driver2",
            "point_lio": "/share/point_lio",
        }
        with patch.object(
            module,
            "get_package_share_directory",
            side_effect=package_shares.__getitem__,
        ):
            launch_description = module.generate_launch_description()

        actions = launch_description.entities
        nodes = [action for action in actions if isinstance(action, Node)]
        includes = [
            action for action in actions if isinstance(action, IncludeLaunchDescription)
        ]

        self.assertEqual(
            [getattr(node, "_Node__package") for node in nodes], ["mycontrol"]
        )
        include_paths = {
            source._LaunchDescriptionSource__location[0].perform({})
            for source in (action.launch_description_source for action in includes)
        }
        self.assertEqual(
            include_paths,
            {
                "/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py",
                "/share/point_lio/launch/point_lio.launch.py",
            },
        )


if __name__ == "__main__":
    unittest.main()
