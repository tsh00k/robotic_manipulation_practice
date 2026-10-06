# Copyright 2026 anby
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch glue for the scene.* parameters, and the rule that only the bridge gets them."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
REPO_SRC = PACKAGE.parent


def load_launch_module():
    spec = importlib.util.spec_from_file_location(
        'demo_launch', PACKAGE / 'launch' / 'demo.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


demo = load_launch_module()


def context(scene_enabled='false', **given):
    """Build a launch context as if the user passed `given` and left the rest at 'auto'."""
    ctx = LaunchContext()
    ctx.launch_configurations['scene_enabled'] = scene_enabled
    for body in ('box', 'bin'):
        for field in demo.SCENE_POSE_FIELDS:
            ctx.launch_configurations[f'{body}_{field}'] = 'auto'
    ctx.launch_configurations.update(given)
    return ctx


def camera_context(camera, source):
    ctx = LaunchContext()
    ctx.launch_configurations['enable_rgbd_camera'] = camera
    ctx.launch_configurations['observation_source'] = source
    return ctx


def test_the_camera_follows_the_observation_source_unless_told():
    assert demo.camera_enabled(camera_context('auto', 'vision')) is True
    assert demo.camera_enabled(camera_context('auto', 'oracle')) is False
    assert demo.camera_enabled(camera_context('true', 'oracle')) is True
    assert demo.camera_enabled(camera_context('false', 'vision')) is False
    with pytest.raises(RuntimeError, match='enable_rgbd_camera'):
        demo.camera_enabled(camera_context('yes', 'vision'))


def test_the_defaults_give_vision_a_camera():
    defaults = {a.name: a.default_value for a in demo.generate_launch_description().entities
                if hasattr(a, 'default_value') and hasattr(a, 'name')}
    ctx = LaunchContext()
    for name in ('enable_rgbd_camera', 'observation_source'):
        ctx.launch_configurations[name] = ''.join(
            part.perform(ctx) for part in defaults[name])
    assert ctx.launch_configurations['observation_source'] == 'vision'
    assert demo.camera_enabled(ctx) is True


def test_scene_is_off_by_default_and_passes_nothing_else():
    assert demo.scene_parameters(context()) == {'scene.enabled': False}


def test_enabling_alone_passes_only_the_flag():
    # Every pose is then the node's own default, written once in the node.
    assert demo.scene_parameters(context('true')) == {'scene.enabled': True}


def test_only_the_values_the_user_gave_are_passed_as_floats():
    parameters = demo.scene_parameters(context('true', box_x='0.45', bin_yaw='-0.4', box_z='1'))
    assert parameters == {
        'scene.enabled': True, 'scene.box.x': 0.45, 'scene.bin.yaw': -0.4, 'scene.box.z': 1.0}
    # An int would be rejected by rclcpp for a parameter declared as double.
    assert all(isinstance(v, float) for k, v in parameters.items() if k != 'scene.enabled')


def test_a_pose_without_enabling_the_scene_is_an_error():
    with pytest.raises(RuntimeError, match='box_x:=0.4.*scene_enabled'):
        demo.scene_parameters(context(box_x='0.4'))


def test_a_value_that_is_not_a_number_is_an_error():
    with pytest.raises(RuntimeError, match="bin_pitch:='abc' is not a number"):
        demo.scene_parameters(context('true', bin_pitch='abc'))


def test_scene_enabled_must_be_a_boolean_word():
    with pytest.raises(RuntimeError, match='scene_enabled'):
        demo.scene_parameters(context('yes'))


def test_scene_parameters_are_built_only_for_the_bridge():
    source = (PACKAGE / 'launch' / 'demo.launch.py').read_text()
    # Defined once, called once (inside make_bridge_node). A second call site would be a
    # second node receiving the simulator's ground-truth poses.
    assert source.count('scene_parameters(') == 2
    bridge_start = source.index('def make_bridge_node')
    bridge_end = source.index('def generate_launch_description')
    assert 'scene_parameters(context)' in source[bridge_start:bridge_end]


@pytest.mark.parametrize(
    'package', ['task_executor', 'mujoco_perception', 'manipulation_interfaces'])
def test_other_packages_do_not_use_the_scene_configuration(package):
    # docs/adr/015: box/bin poses are simulator ground truth; perception and the executor
    # get them from the camera. Tripwire for anyone wiring the scene parameters, or the
    # bridge's private scene headers, into another package's product code.
    #
    # Files under test/ are skipped on purpose. Evaluation tools there start a bridge and
    # pass it `-p scene.box.yaw:=...` on its command line; that sets up the simulator and
    # gives nothing to the node under test. What must not happen is a node reading these
    # parameters, and no node in a test/ directory is one of the product nodes.
    suffixes = {'.cpp', '.hpp', '.py', '.txt', '.xml', '.yaml', '.msg', '.srv'}
    needles = ('scene_config', 'scene_ops', 'scene.enabled', 'scene.box', 'scene.bin')
    offenders = []
    for path in (REPO_SRC / package).rglob('*'):
        if path.suffix not in suffixes or not path.is_file():
            continue
        if 'test' in path.relative_to(REPO_SRC / package).parts[:-1]:
            continue
        text = path.read_text(errors='ignore')
        offenders += [f'{path.relative_to(REPO_SRC)}: {n}' for n in needles if n in text]
    assert not offenders, offenders


def test_the_scene_configuration_guard_still_catches_product_code(tmp_path):
    # The guard above skips test/ directories. Check it did not become blind: the same
    # scan, pointed at a product-code file that mentions a scene parameter, must report it.
    product = tmp_path / 'fake_package' / 'src'
    product.mkdir(parents=True)
    (product / 'node.cpp').write_text('declare_parameter("scene.bin.x", 0.0);\n')
    tools = tmp_path / 'fake_package' / 'test'
    tools.mkdir()
    (tools / 'probe.py').write_text("args += ['-p', 'scene.bin.x:=0.5']\n")
    root = tmp_path / 'fake_package'
    found = []
    for path in root.rglob('*'):
        if path.suffix in {'.cpp', '.py'} and path.is_file() and \
                'test' not in path.relative_to(root).parts[:-1] and \
                'scene.bin' in path.read_text():
            found.append(path.name)
    assert found == ['node.cpp']
