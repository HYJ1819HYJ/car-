#!/usr/bin/env python3
"""Generate an additive planning mask around selected real occupied cells."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from scipy.ndimage import distance_transform_edt


def generate(map_path, config_path, output):
    meta = yaml.safe_load(map_path.read_text())
    config = yaml.safe_load(config_path.read_text())
    if map_path.name != config['map']:
        raise ValueError('Selections belong to a different map')
    if meta.get('negate', 0) or abs(meta['origin'][2]) > 1e-9:
        raise ValueError('Only unrotated standard occupancy maps are supported')
    image_path = map_path.parent / Path(meta['image']).name
    original_bytes = image_path.read_bytes()
    pixels = np.flipud(np.array(Image.open(image_path).convert('L')))
    occupied = (255 - pixels.astype(float)) / 255 > meta['occupied_thresh']
    yy, xx = np.indices(pixels.shape)
    wx = meta['origin'][0] + (xx + .5) * meta['resolution']
    wy = meta['origin'][1] + (yy + .5) * meta['resolution']
    seeds = np.zeros(pixels.shape, dtype=bool)
    selections = []
    for region in config['regions']:
        x, y = region['center']; radius = region['selection_radius']
        selected = occupied & ((wx-x)**2 + (wy-y)**2 <= radius**2)
        if not selected.any():
            raise ValueError('No occupied cells selected: ' + region['name'])
        seeds |= selected
        selections.append(dict(name=region['name'], cells=int(selected.sum())))
    extra = float(config['extra_clearance'])
    if not math.isfinite(extra) or not 0 < extra <= .10:
        raise ValueError('Extra clearance must be in (0, .10] meters')
    distance = distance_transform_edt(~seeds) * meta['resolution']
    mask = distance <= extra + 1e-9
    # Unknown background means "no additional constraint" in the additive
    # global layer. The local rolling layer interprets it as zero cost.
    overlay = np.full(pixels.shape, 205, dtype=np.uint8)
    overlay[mask] = 0
    pgm = output.with_suffix('.pgm')
    Image.fromarray(np.flipud(overlay)).save(pgm)
    saved = dict(meta, image=pgm.name)
    output.with_suffix('.yaml').write_text(yaml.safe_dump(saved, sort_keys=False))
    report = dict(map=map_path.name, source_sha256=hashlib.sha256(original_bytes).hexdigest(),
                  mask_sha256=hashlib.sha256(pgm.read_bytes()).hexdigest(),
                  selections=selections, extra_clearance=extra,
                  mask_cells=int(mask.sum()), added_free_cells=int((mask & ~occupied & (pixels>230)).sum()))
    output.with_suffix('.json').write_text(json.dumps(report, indent=2))
    assert image_path.read_bytes() == original_bytes
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(generate(args.map, args.config, args.output), indent=2))
