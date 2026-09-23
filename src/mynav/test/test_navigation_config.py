from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_params():
    with (ROOT / 'params' / 'nav2_params.yaml').open() as stream:
        return yaml.safe_load(stream)


def test_default_bt_recovery_actions_are_configured():
    behavior = load_params()['behavior_server']['ros__parameters']
    plugins = behavior['behavior_plugins']

    assert {'spin', 'backup', 'drive_on_heading', 'assisted_teleop', 'wait'} <= set(plugins)
    for name in plugins:
        assert name in behavior
        assert behavior[name]['plugin'].startswith('nav2_behaviors::')


def test_amcl_waits_for_real_initial_pose():
    amcl = load_params()['amcl']['ros__parameters']

    assert amcl['set_initial_pose'] is False
    assert amcl['robot_model_type'] == 'nav2_amcl::OmniMotionModel'


def test_laserscan_tf_window_covers_observed_sensor_delay():
    launch_text = (ROOT.parent / 'mycontrol' / 'launch' / 'mycontrol_launch.py').read_text()

    assert '"transform_tolerance": 0.2' in launch_text
