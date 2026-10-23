#!/usr/bin/env python3
"""Small geometry/scene regression tests; run after sourcing ROS 2 Humble."""
import unittest
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from dynamic_obstacles import SCENARIO, distance_to_path, in_box


class DynamicExperimentTests(unittest.TestCase):
    def test_segment_distance_not_vertex_distance(self):
        path = np.array([[0.,0.],[2.,0.],[2.,2.]])
        self.assertAlmostEqual(distance_to_path(np.array([1.,.2]),path), .2)
        self.assertAlmostEqual(distance_to_path(np.array([3.,1.]),path), 1.)
        self.assertAlmostEqual(distance_to_path(np.array([-1.,0.]),path), 1.)

    def test_trigger_boxes(self):
        self.assertTrue(in_box([12.2,7.1],SCENARIO['R3']['trigger_box']))
        self.assertFalse(in_box([10.8,4.],SCENARIO['R3']['trigger_box']))
        self.assertFalse(in_box([5.7,2.],SCENARIO['R2']['trigger_box']))
        self.assertTrue(in_box([5.7,5.6],SCENARIO['R2']['trigger_box']))

    def test_targets_clear_walls_and_visible_on_desktop(self):
        root = Path(__file__).resolve().parents[1]
        image = np.flipud(np.array(Image.open(root/'src/sp_nav_bringup/map/maze_map.pgm')))
        clearance = distance_transform_edt(image>128)*.05
        for robot in ('R2','R3'):
            for name in ('outbound','return','clearance_waypoint'):
                if name not in SCENARIO[robot]:
                    continue
                x,y = SCENARIO[robot][name]
                self.assertGreater(clearance[int(y/.05),int(x/.05)], np.sqrt(2)*.25,
                                   msg=f'{robot} {name}: insufficient rotation clearance')
                px,py = round(300+x/.05*1000/300),round((299-y/.05)*1000/300)
                self.assertLess(1751+px,4096)
                self.assertTrue(0<py<1000)
                recovered = np.array([(px-300)*.05/(1000/300),(299-py/(1000/300))*.05])
                self.assertLess(np.linalg.norm(recovered-[x,y]),.011)


if __name__ == '__main__':
    unittest.main()
