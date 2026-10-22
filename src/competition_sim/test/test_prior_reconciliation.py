#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import numpy as np
spec=importlib.util.spec_from_file_location('reconcile',Path(__file__).resolve().parents[1]/'scripts/reconcile_prior_map.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class ReconciliationTest(unittest.TestCase):
    def test_three_distinct_frames_and_occlusion_reset(self):
        evidence=m.FreeEvidence(np.array([[1,2]]))
        self.assertFalse(evidence.observe(1,[True,True]))
        self.assertFalse(evidence.observe(1,[True,True]))
        self.assertFalse(evidence.observe(2,[True,False]))
        self.assertTrue(evidence.observe(3,[True,True]))
        np.testing.assert_array_equal(evidence.cleared,[True,False])
        evidence.observe(4,[False,True]);evidence.observe(5,[False,True])
        np.testing.assert_array_equal(evidence.cleared,[True,True])
    def test_obstacle_blocks_free_ray(self):
        pts=np.array([[1.,0],[2.,0],[4.,0]])
        free=m.ray_free(pts,[0,0,0],[3.]*7,-.3,.1,.1,10.)
        np.testing.assert_array_equal(free,[True,True,False])
    def test_nan_and_neighbouring_hit_protect_cells(self):
        for ranges in ([5,5,float('nan'),5,5],[5,5,5,1,5]):
            free=m.ray_free(np.array([[2.,0]]),[0,0,0],ranges,-.2,.1,.1,10.)
            self.assertFalse(free[0])
    def test_infinity_range_limit_margin_and_fov(self):
        free=m.ray_free(np.array([[4.8,0],[4.95,0],[6.,0],[-1.,0]]),[0,0,0],
                        [float('inf')]*7,-.3,.1,.1,10.)
        np.testing.assert_array_equal(free,[True,False,False,False])
    def test_coordinate_transform(self):
        free=m.ray_free(np.array([[1,3.],[3.,1]]),[1,1,np.pi/2],[5.]*7,-.3,.1,.1,10.)
        np.testing.assert_array_equal(free,[True,False])
    def test_only_labelled_occupied_cells_mutable(self):
        from PIL import Image
        pkg=Path(__file__).resolve().parents[1]
        raw=np.asarray(Image.open(pkg/'maps/mapping_before_move.pgm'))
        fixed=np.asarray(Image.open(pkg/'maps/final_competition.pgm'))
        labels=np.asarray(Image.open(pkg/'maps/movable_prior_mask.png'))
        self.assertTrue(np.all(raw[labels>0]==0))
        np.testing.assert_array_equal(raw[labels==0],fixed[labels==0])
        evidence=m.FreeEvidence(labels)
        for t in range(3):evidence.observe(t,np.ones(len(evidence.rows),bool))
        updated=raw.copy();updated[evidence.rows[evidence.cleared],evidence.cols[evidence.cleared]]=254
        np.testing.assert_array_equal(updated,fixed)

if __name__=='__main__':unittest.main()
