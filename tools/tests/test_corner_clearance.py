"""Geometry regressions for physical corner selection and additive masks."""
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
from PIL import Image
import yaml

spec = importlib.util.spec_from_file_location('corner_masks', Path(__file__).parents[1]/'generate_corner_clearance_map.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class CornerGeometry(unittest.TestCase):
    def regions(self, occupied):
        return module.detect_corner_regions(occupied, .025, [0, 0, 0])

    def test_straight_wall_and_grid_staircase_are_not_corners(self):
        straight = np.zeros((120, 120), np.uint8)
        straight[40:44, 10:110] = 1
        self.assertEqual(self.regions(straight), [])
        rotated = cv2.warpAffine(straight, cv2.getRotationMatrix2D((60, 60), 23, 1), (120, 120), flags=cv2.INTER_NEAREST)
        self.assertEqual(self.regions(rotated), [])

    def test_L_both_faces_are_one_corner_with_both_neighborhoods(self):
        wall = np.zeros((120, 120), np.uint8)
        wall[30:34, 30:100] = 1
        wall[30:100, 30:34] = 1
        regions = self.regions(wall)
        self.assertEqual(len(regions), 1)
        self.assertEqual(len(regions[0]['vertices']), 2)
        for evidence in regions[0]['evidence']:
            self.assertGreaterEqual(min(evidence['wall_lengths']), .25)
            self.assertGreaterEqual(evidence['turn_degrees'], 45)

    def test_closed_wall_has_four_physical_corners_not_eight(self):
        wall = np.zeros((120, 120), np.uint8)
        wall[20:100, 20:100] = 1
        wall[24:96, 24:96] = 0
        regions = self.regions(wall)
        self.assertEqual(len(regions), 4)
        self.assertTrue(all(len(r['vertices']) == 2 for r in regions))

    def test_short_square_noise_is_not_long_wall_corner(self):
        wall = np.zeros((120, 120), np.uint8)
        wall[40:44, 40:44] = 1
        self.assertEqual(self.regions(wall), [])

    def test_bounds_exclude_unrelated_contours(self):
        wall = np.zeros((160, 160), np.uint8)
        for x in [10, 90]:
            wall[10:14, x:x+60] = 1
            wall[10:70, x:x+4] = 1
        regions = module.detect_corner_regions(wall, .025, [0, 0, 0], bounds=[0, 0, 2, 2])
        self.assertEqual(len(regions), 1)
        self.assertLess(regions[0]['center'][0], 2)

    def test_additive_radius_original_and_unknown_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            occupied = np.zeros((120, 120), bool)
            occupied[30:34, 30:100] = True
            occupied[30:100, 30:34] = True
            pixels = np.full(occupied.shape, 254, np.uint8)
            pixels[occupied] = 0
            pixels[:5] = 205
            source = root/'map.pgm'
            Image.fromarray(np.flipud(pixels)).save(source)
            original = source.read_bytes()
            meta = dict(image=source.name, resolution=.025, origin=[0., 0., 0.], negate=0, occupied_thresh=.65, free_thresh=.196)
            (root/'map.yaml').write_text(yaml.safe_dump(meta))
            result = module.generate(root/'map.yaml', None, root/'mask', True, .05)
            self.assertEqual(source.read_bytes(), original)
            self.assertEqual(result['source_sha256'], hashlib.sha256(original).hexdigest())
            mask = np.flipud(np.array(Image.open(root/'mask.pgm')))
            # Mid-wall straight sections stay unchanged; no blanket inflation.
            self.assertEqual(mask[31, 75], 205)
            self.assertEqual(mask[3, 3], 205)
            # Outside a real corner: 2 cells=5 cm included, 3 cells excluded.
            self.assertEqual(mask[30, 28], 0)
            self.assertEqual(mask[30, 27], 205)
            self.assertGreater(result['added_free_cells'], 0)
            with self.assertRaises(ValueError):
                module.generate(root/'map.yaml', None, root/'map', True, .05)
            self.assertEqual(source.read_bytes(), original)

if __name__ == '__main__':
    unittest.main()
