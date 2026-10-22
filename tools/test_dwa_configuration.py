#!/usr/bin/env python3
"""Validate resolved launch parameters without starting ROS nodes or hardware."""
from pathlib import Path
import subprocess
import math
import re
import unittest

import yaml
from dwa_local_planner.cfg import DWAPlannerConfig


ROOT = Path(__file__).resolve().parents[1]


def parameters(package, launch, *arguments):
    result = subprocess.run(
        ['roslaunch', '--dump-params', package, launch, *arguments],
        check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        timeout=30)
    return yaml.safe_load(result.stdout)


class DwaConfigurationTest(unittest.TestCase):
    def check_navigation(self, values, linear, angular):
        self.assertEqual(values['/move_base/base_local_planner'],
                         'dwa_local_planner/DWAPlannerROS')
        self.assertFalse(any('Teb' in key for key in values))
        prefix = '/move_base/DWAPlannerROS/'
        for key in ('max_vel_x', 'max_vel_trans'):
            self.assertAlmostEqual(values[prefix + key], linear)
        self.assertAlmostEqual(values[prefix + 'max_vel_theta'], angular)
        self.assertEqual(values[prefix + 'min_vel_x'], 0.0)
        self.assertEqual(values[prefix + 'min_vel_y'], 0.0)
        self.assertEqual(values[prefix + 'max_vel_y'], 0.0)
        self.assertTrue(values[prefix + 'use_dwa'])
        self.assertEqual(values[prefix + 'odom_topic'], '/odom')
        self.assertFalse(values['/move_base/GlobalPlanner/allow_unknown'])
        polygon = values['/robot_geometry/footprint']
        self.assertEqual(len(polygon), 24)
        for name in ('global_costmap', 'local_costmap'):
            self.assertEqual(values['/move_base/' + name + '/footprint'], polygon)

    def test_hardware_default_and_low_speed_with_both_global_planners(self):
        for smooth in ('true', 'false'):
            for linear, angular in ((0.50, 0.75), (0.10, 0.25)):
                with self.subTest(smooth=smooth, linear=linear):
                    values = parameters('maze_bridge', 'navigation_stack.launch',
                                        'mode:=navigation', 'smooth_global_plan:=' + smooth,
                                        'max_nav_v:=' + str(linear),
                                        'max_nav_w:=' + str(angular))
                    self.check_navigation(values, linear, angular)
                    self.assertTrue(values['/serial_bridge/safety_enable'])
                    self.assertEqual(values['/serial_bridge/footprint'],
                                     values['/robot_geometry/footprint'])
                    self.assertFalse(values['/move_base/clearing_rotation_allowed'])

    def test_simulation_default_and_low_speed(self):
        values = parameters('competition_sim', 'navigation.launch')
        self.check_navigation(values, 3.0, 1.0)
        for linear, angular in ((3.0, 1.0), (0.50, 0.75), (0.10, 0.25)):
            with self.subTest(linear=linear):
                values = parameters('competition_sim', 'navigation.launch',
                                    'max_nav_v:=' + str(linear),
                                    'max_nav_w:=' + str(angular))
                self.check_navigation(values, linear, angular)
                self.assertTrue(values['/competition_simulation'])
                self.assertFalse(values['/move_base/recovery_behavior_enabled'])
                self.assertEqual(values['/simulation_safety_gate/max_vel_x'], linear)
                self.assertEqual(values['/simulation_safety_gate/max_vel_theta'], angular)

    def test_stage_models_support_requested_simulation_speed(self):
        for path in (ROOT / 'src/competition_sim').rglob('*.world'):
            with self.subTest(world=str(path)):
                match = re.search(r'velocity_bounds\s*\[([^]]+)\]', path.read_text())
                self.assertIsNotNone(match)
                bounds = [float(value) for value in match.group(1).split()]
                self.assertEqual(bounds[0], 0.0)
                self.assertAlmostEqual(bounds[1], 3.0)
                self.assertAlmostEqual(math.radians(bounds[-1]), 1.0, places=6)
                self.assertAlmostEqual(math.radians(bounds[-2]), -1.0, places=6)

    def test_dwa_yaml_uses_supported_dynamic_parameters(self):
        values = yaml.safe_load((ROOT / 'src/maze_bridge/launch/'
                                 'dwa_local_planner_params.yaml').read_text())['DWAPlannerROS']
        # These parameters are read directly by the node, outside dynamic_reconfigure.
        direct = {'odom_topic', 'publish_traj_pc', 'publish_cost_grid_pc', 'global_frame_id'}
        self.assertFalse(set(values) - set(DWAPlannerConfig.defaults) - direct)


if __name__ == '__main__':
    unittest.main(verbosity=2)
