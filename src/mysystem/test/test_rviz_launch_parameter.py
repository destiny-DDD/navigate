import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_launch_module(filename):
    module_path = PACKAGE_ROOT / 'launch' / filename
    spec = importlib.util.spec_from_file_location(filename, module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def package_share(package_name):
    roots = {
        'mysystem': PACKAGE_ROOT,
        'livox_ros_driver2': PACKAGE_ROOT,
        'point_lio': PACKAGE_ROOT,
        'ros2_libxr': PACKAGE_ROOT,
    }
    return str(roots[package_name])


def test_mysystem_rviz_launch_can_disable_rviz():
    module = load_launch_module('mysystem_rviz_launch.py')
    module.get_package_share_directory = package_share

    launch_description = module.generate_launch_description()
    declarations = {
        action.name: action
        for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    rviz_nodes = [
        action
        for action in launch_description.entities
        if isinstance(action, Node) and action.node_executable == 'rviz2'
    ]

    assert declarations['rviz'].default_value[0].perform(LaunchContext()) == 'true'
    assert len(rviz_nodes) == 1
    assert isinstance(rviz_nodes[0].condition, IfCondition)

    context = LaunchContext()
    context.launch_configurations['rviz'] = 'false'
    assert rviz_nodes[0].condition.evaluate(context) is False


def test_mycontrol_forwards_rviz_parameter_to_mysystem():
    module_path = (
        PACKAGE_ROOT.parent / 'mycontrol' / 'launch' / 'mycontrol_launch.py'
    )
    spec = importlib.util.spec_from_file_location('mycontrol_launch', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = package_share

    launch_description = module.generate_launch_description()
    declarations = {
        action.name: action
        for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    system_include = next(
        action
        for action in launch_description.entities
        if isinstance(action, IncludeLaunchDescription)
        and dict(action.launch_arguments).keys() == {'rviz'}
        and isinstance(dict(action.launch_arguments)['rviz'], LaunchConfiguration)
    )

    assert declarations['rviz'].default_value[0].perform(LaunchContext()) == 'true'
    assert dict(system_include.launch_arguments).keys() == {'rviz'}
    context = LaunchContext()
    context.launch_configurations['rviz'] = 'false'
    forwarded_rviz = dict(system_include.launch_arguments)['rviz']
    assert forwarded_rviz.perform(context) == 'false'
