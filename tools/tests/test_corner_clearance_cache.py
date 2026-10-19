"""Automatic-map regression tests: stale maps must not reuse old geometry."""
import json
import shutil
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
from PIL import Image
import yaml
sys.path.insert(0, str(Path(__file__).parents[2]/'src/maze_bridge/src'))
from maze_bridge.corner_clearance_cache import prepare, validate

class AutomaticCornerMaps(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root/'new_mapping_session.yaml'
        self.pixels = np.full((120, 120), 254, np.uint8)
        self.pixels[30:34, 30:100] = 0
        self.pixels[30:100, 30:34] = 0
        self.write()
        self.cache = self.root/'cache'

    def write(self, origin=None):
        Image.fromarray(np.flipud(self.pixels)).save(self.source.with_suffix('.pgm'))
        self.meta = dict(image=self.source.with_suffix('.pgm').name, resolution=.025,
                         origin=origin or [0., 0., 0.], negate=0, occupied_thresh=.65, free_thresh=.196)
        self.source.write_text(yaml.safe_dump(self.meta))

    def test_new_map_automatically_generated_then_reused_without_source_edits(self):
        original = [p.read_bytes() for p in [self.source, self.source.with_suffix('.pgm')]]
        mask, report, action = prepare(self.source, self.cache)
        self.assertEqual(action, 'generated')
        self.assertEqual(report['corner_count'], 1)
        self.assertEqual(report['endpoint_count'], 2)
        stamp = mask.stat().st_mtime_ns
        again, _, action = prepare(self.source, self.cache)
        self.assertEqual((again, action, again.stat().st_mtime_ns), (mask, 'cache', stamp))
        self.assertEqual(original, [p.read_bytes() for p in [self.source, self.source.with_suffix('.pgm')]])

    def test_origin_change_requires_new_cache_with_translated_corners(self):
        old, report, _ = prepare(self.source, self.cache)
        center = report['selections'][0]['center']
        self.write([10., -3., 0.])
        new, report, action = prepare(self.source, self.cache)
        self.assertNotEqual(old, new)
        self.assertEqual(action, 'generated')
        np.testing.assert_allclose(report['selections'][0]['center'], np.array(center)+[10., -3.])
        validate(self.source, new)
        with self.assertRaises(ValueError):
            validate(self.source, old)

    def test_image_change_and_parameter_change_require_new_masks(self):
        old, _, _ = prepare(self.source, self.cache)
        self.pixels[75:79, 34:100] = 0
        self.write()
        new, _, _ = prepare(self.source, self.cache)
        self.assertNotEqual(old, new)
        changed, report, _ = prepare(self.source, self.cache, extra=.075)
        self.assertNotEqual(new, changed)
        self.assertEqual(report['extra_clearance'], .075)

    def test_explicit_wrong_map_is_rejected(self):
        old, _, _ = prepare(self.source, self.cache)
        self.write([.5, .5, 0.])
        with self.assertRaises(ValueError):
            prepare(self.source, self.cache, explicit_mask=old)

    def test_straight_wall_protects_only_two_free_ends(self):
        self.pixels[:] = 254
        self.pixels[30:34, 30:100] = 0
        self.write()
        mask, report, _ = prepare(self.source, self.cache)
        self.assertEqual(report['corner_count'], 0)
        self.assertEqual(report['endpoint_count'], 2)
        pixels = np.flipud(np.array(Image.open(mask.with_suffix('.pgm'))))
        self.assertEqual(pixels[31, 65], 205)
        self.assertEqual(pixels[31, 28], 0)
        self.assertEqual(pixels[31, 27], 205)
        old, old_report, _ = prepare(self.source, self.cache, endpoints=False)
        self.assertEqual(old_report['selections'], [])
        self.assertEqual(old_report['mask_cells'], 0)
        self.assertNotEqual(old, mask)

    def test_corrupted_cache_is_regenerated(self):
        mask, _, _ = prepare(self.source, self.cache)
        mask.with_suffix('.pgm').write_bytes(b'broken')
        repaired, _, action = prepare(self.source, self.cache)
        self.assertEqual(mask, repaired)
        self.assertEqual(action, 'generated')
        validate(self.source, repaired)

    def test_automatic_main_bounds_include_internal_wall_but_exclude_far_clutter(self):
        self.pixels[:] = 254
        self.pixels[20:100, 20:100] = 0
        self.pixels[24:96, 24:96] = 254
        self.pixels[45:49, 45:80] = 0
        self.pixels[45:80, 45:49] = 0
        self.pixels[1:5, 1:18] = 0
        self.pixels[1:18, 1:5] = 0
        _, report, _ = self._prepare_current_pixels()
        self.assertEqual((report['corner_count'], report['endpoint_count']), (5, 2))
        _, whole_map, _ = prepare(self.source, self.cache, scope='all')
        self.assertEqual((whole_map['corner_count'], whole_map['endpoint_count']), (6, 4))

    def test_matching_sidecar_reused_stale_sidecar_preserved_but_not_loaded(self):
        mask, report, _ = prepare(self.source, self.cache)
        preferred = self.source.with_name(self.source.stem+'_corner_clearance.yaml')
        shutil.copy2(mask.with_suffix('.pgm'), preferred.with_suffix('.pgm'))
        meta = yaml.safe_load(mask.read_text())
        meta['image'] = preferred.with_suffix('.pgm').name
        preferred.write_text(yaml.safe_dump(meta))
        preferred.with_suffix('.json').write_text(json.dumps(report))
        chosen, _, action = prepare(self.source, self.cache)
        self.assertEqual((chosen, action), (preferred, 'reviewed_sidecar'))
        original = preferred.read_bytes()
        self.write([2., 3., 0.])
        chosen, _, action = prepare(self.source, self.cache)
        self.assertNotEqual(chosen, preferred)
        self.assertEqual(action, 'generated')
        self.assertEqual(preferred.read_bytes(), original)

    def test_whole_map_override_does_not_reuse_bounded_sidecar(self):
        from maze_bridge.corner_clearance_cache import compatible
        report = dict(method='all_corners', extra_clearance=.05,
                      detection=dict(minimum_wall_length=.25, minimum_turn_degrees=45., bounds=[0, 0, 2, 2],
                                     include_wall_endpoints=True, wall_endpoint_radius=.05))
        self.assertTrue(compatible(report, .05, .25, 45., None, 'main_walls'))
        self.assertFalse(compatible(report, .05, .25, 45., None, 'all'))

    def test_old_corner_only_sidecar_is_preserved_and_regenerated(self):
        old, report, _ = prepare(self.source, self.cache, endpoints=False)
        preferred = self.source.with_name(self.source.stem+'_corner_clearance.yaml')
        shutil.copy2(old.with_suffix('.pgm'), preferred.with_suffix('.pgm'))
        meta = yaml.safe_load(old.read_text());meta['image'] = preferred.with_suffix('.pgm').name
        preferred.write_text(yaml.safe_dump(meta))
        # A previous release did not record either endpoint field.
        report['detection'].pop('include_wall_endpoints')
        report['detection'].pop('wall_endpoint_radius')
        preferred.with_suffix('.json').write_text(json.dumps(report))
        saved = [p.read_bytes() for p in [preferred, preferred.with_suffix('.pgm')]]
        chosen, current, action = prepare(self.source, self.cache)
        self.assertNotEqual(chosen, preferred)
        self.assertEqual((action, current['endpoint_count']), ('generated', 2))
        self.assertEqual(saved, [p.read_bytes() for p in [preferred, preferred.with_suffix('.pgm')]])
        with self.assertRaisesRegex(ValueError, 'lacks requested wall-end'):
            prepare(self.source, self.cache, explicit_mask=preferred)

    def test_endpoint_radius_change_invalidates_cache(self):
        first, _, _ = prepare(self.source, self.cache)
        second, report, _ = prepare(self.source, self.cache, endpoint_radius=.075)
        self.assertNotEqual(first, second)
        self.assertEqual(report['wall_endpoint_radius'], .075)

    def _prepare_current_pixels(self):
        self.write()
        return prepare(self.source, self.cache)

if __name__ == '__main__':
    unittest.main()
