"""Regression geometry for free thin-wall ends and the actual competition map."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np
from PIL import Image
import yaml
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/maze_bridge/src'))
from maze_bridge.corner_clearance import detect_wall_end_regions, generate


class WallEndpoints(unittest.TestCase):
    def ends(self, wall, **kwargs):
        return detect_wall_end_regions(wall, .025, [0, 0, 0], **kwargs)

    def test_horizontal_oblique_and_vertical_have_two_ends(self):
        wall = np.zeros((140, 140), np.uint8);wall[65:69, 20:120] = 1
        for angle in [0, 23, 45, 90]:
            with self.subTest(angle=angle):
                image = cv2.warpAffine(wall, cv2.getRotationMatrix2D((70,70), angle, 1),
                                       (140,140), flags=cv2.INTER_NEAREST)
                self.assertEqual(len(self.ends(image)), 2)

    def test_one_cell_wall_and_attached_T_only_expose_free_ends(self):
        wall = np.zeros((140,140), np.uint8);wall[20:120,20] = 1
        wall[70,20:100] = 1
        ends = self.ends(wall)
        self.assertEqual(len(ends), 3)
        self.assertFalse(any(np.linalg.norm(np.array(r['center'])-[.5125,1.7625])<.1 for r in ends))

    def test_closed_enclosure_wide_box_and_short_noise_have_no_ends(self):
        wall = np.zeros((140,140),np.uint8)
        wall[10:130,10:130] = 1;wall[14:126,14:126] = 0
        wall[30:60,30:60] = 1;wall[80:83,80:83] = 1
        self.assertEqual(self.ends(wall), [])

    def test_bounds_and_translated_origin(self):
        wall = np.zeros((140,140),np.uint8);wall[65:69,20:120] = 1
        all_ends = self.ends(wall)
        selected = self.ends(wall,bounds=[0,0,1,3])
        self.assertEqual(len(selected), 1)
        shifted = detect_wall_end_regions(wall,.025,[7,-2,0])
        np.testing.assert_allclose([r['center'] for r in shifted],np.array([r['center'] for r in all_ends])+[7,-2])

    def test_exact_disk_only_at_caps_and_original_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);a=np.full((120,120),254,np.uint8);a[40:42,20:100]=0
            Image.fromarray(np.flipud(a)).save(p/'source.pgm')
            meta=dict(image='source.pgm',resolution=.025,origin=[0.,0.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)
            (p/'source.yaml').write_text(yaml.safe_dump(meta))
            original=[(p/name).read_bytes() for name in ['source.yaml','source.pgm']]
            report=generate(p/'source.yaml',None,p/'mask',True)
            mask=np.flipud(np.array(Image.open(p/'mask.pgm')))==0
            seeds=np.zeros(a.shape,np.uint8)
            for end in report['selections']:
                for x,y in end['seed_cells']:seeds[y,x]=1
            distance=cv2.distanceTransform(1-seeds,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*.025
            self.assertTrue(np.all(distance[mask]<=.05+1e-7))
            self.assertTrue(mask[40,18]);self.assertFalse(mask[40,17])
            self.assertTrue(mask[40,101]);self.assertFalse(mask[40,102])
            self.assertFalse(mask[40,60]);self.assertFalse(mask[39,60])
            self.assertEqual(original,[(p/name).read_bytes() for name in ['source.yaml','source.pgm']])

    def test_actual_course_has_exactly_the_three_requested_free_ends(self):
        source=ROOT/'src/competition_sim/maps/final_competition.yaml'
        metadata=yaml.safe_load(source.read_text());pgm=source.parent/metadata['image']
        original=hashlib.sha256(pgm.read_bytes()).hexdigest()
        geometry=json.loads((source.parent/'geometry.json').read_text())
        # The other end of each listed wall is attached to the outer boundary.
        pixels=[geometry['wall_polylines_pixels'][0][0],
                geometry['wall_polylines_pixels'][1][-1],
                geometry['wall_polylines_pixels'][2][0]]
        sx,sy=geometry['calibration']['metres_per_pixel_xy']
        expected=np.array([[(x-9)*sx,(674-y)*sy] for x,y in pixels])
        with tempfile.TemporaryDirectory() as tmp:
            base=generate(source,None,Path(tmp)/'old',True,include_wall_endpoints=False)
            report=generate(source,None,Path(tmp)/'new',True)
            self.assertEqual(report['endpoint_count'],3)
            self.assertEqual(report['corner_count'],base['corner_count'])
            ends=[r for r in report['selections'] if r.get('kind')=='wall_endpoint']
            for point in expected:
                self.assertLess(min(np.linalg.norm(point-r['center']) for r in ends),.05)
            old_mask=np.array(Image.open(Path(tmp)/'old.pgm'))==0
            new_mask=np.array(Image.open(Path(tmp)/'new.pgm'))==0
            self.assertFalse(np.any(old_mask & ~new_mask))
            self.assertGreater(int(np.sum(new_mask & ~old_mask)),0)
        self.assertEqual(hashlib.sha256(pgm.read_bytes()).hexdigest(),original)


if __name__=='__main__':
    unittest.main()
