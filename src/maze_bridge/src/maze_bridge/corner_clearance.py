#!/usr/bin/env python3
"""Generate additive map protection at wall corners and thin-wall free ends."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import cv2
from PIL import Image
import yaml


def image_path(map_path, meta):
    image = Path(meta['image'])
    if image.is_absolute():
        return image if image.exists() else map_path.parent / image.name
    return map_path.parent / image


def main_wall_bounds(occupied, resolution, origin):
    """Use the main connected wall's bounds, including detached walls inside.

    This selects a complete maze in the competition maps without reusing any
    coordinates from an earlier mapping session. --bounds/whole-map override it.
    """
    count, _, stats, _ = cv2.connectedComponentsWithStats(occupied.astype(np.uint8), connectivity=8)
    candidates = [i for i in range(1, count) if max(stats[i, 2:4])*resolution >= .25]
    if not candidates:
        return None
    x, y, width, height, _ = stats[max(candidates, key=lambda i: stats[i, 4])]
    return [float(origin[0]+x*resolution-.15), float(origin[1]+y*resolution-.15),
            float(origin[0]+(x+width)*resolution+.15), float(origin[1]+(y+height)*resolution+.15)]


def _wall_contours(occupied, resolution, minimum_wall_length):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        occupied.astype(np.uint8), connectivity=8)
    walls = np.zeros(occupied.shape, dtype=np.uint8)
    for component in range(1, count):
        if max(stats[component, 2:4]) * resolution >= minimum_wall_length and stats[component, 4] >= 4:
            walls[labels == component] = 255
    contours, _ = cv2.findContours(walls, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    return contours, labels


def detect_wall_end_regions(occupied, resolution, origin, minimum_wall_length=.25,
                            bounds=None, maximum_wall_thickness=.12):
    """Find a contour's thin cap/reversal, not a junction or a box corner.

    The same long-wall contours as corner detection are used. A thin wall's
    two faces reverse at its free cap, either at one simplified vertex or at
    the ends of a short cap edge. Keep only terminal occupied cells as seeds:
    the requested disk must not thicken the whole last 15 cm of the wall.
    """
    contours, _ = _wall_contours(occupied, resolution, minimum_wall_length)
    candidates = []
    minimum = minimum_wall_length / resolution
    thickness = maximum_wall_thickness / resolution
    for contour in contours:
        polygon = cv2.approxPolyDP(contour, .05 / resolution, True).reshape(-1, 2).astype(float)
        if len(polygon) < 2:
            continue
        for i, p in enumerate(polygon):
            previous, following = polygon[i-1], polygon[(i+1) % len(polygon)]
            incoming, outgoing = p-previous, following-p
            a, b = np.linalg.norm(incoming), np.linalg.norm(outgoing)
            if min(a, b) >= minimum and np.dot(incoming, outgoing)/(a*b) <= -math.cos(math.radians(10)):
                candidates.append((p, previous, following))
            # A retained cap edge has short width and long, opposing faces.
            after = polygon[(i+2) % len(polygon)]
            other = after-following
            c = np.linalg.norm(other)
            # Simplification may move a cap vertex along a slanted face.
            # Test transverse thickness, with only bounded longitudinal slack.
            transverse = abs(np.cross(incoming, outgoing))/a if a else math.inf
            if (0 < b <= thickness+2*.05/resolution and transverse <= thickness and
                    min(a, c) >= minimum and
                    np.dot(incoming, other)/(a*c) <= -math.cos(math.radians(20))):
                candidates.append(((p+following)/2, previous, after))
    ends = []
    height, width = occupied.shape
    for point, before, after in candidates:
        inward = (before-point)/np.linalg.norm(before-point) + (after-point)/np.linalg.norm(after-point)
        norm = np.linalg.norm(inward)
        if norm < 1e-9:
            continue
        outward = -inward/norm
        reach = int(math.ceil(thickness+2))
        x0, y0 = np.maximum(0, np.floor(point-reach).astype(int))
        x1, y1 = np.minimum([width, height], np.ceil(point+reach+1).astype(int))
        ys, xs = np.nonzero(occupied[y0:y1, x0:x1])
        cells = np.column_stack([xs+x0, ys+y0])
        delta = cells-point
        along = delta @ outward
        across = delta @ np.array([-outward[1], outward[0]])
        near = (np.linalg.norm(delta, axis=1) <= thickness+1) & (np.abs(across) <= thickness/2+1)
        if not near.any():
            continue
        # Select the terminal row on an axial wall and its raster equivalents
        # on an oblique wall. No neighbouring long wall-face seeds are added.
        cap = cells[near & (along >= along[near].max()-.75)]
        center = np.asarray(origin[:2]) + (cap.mean(axis=0)+.5)*resolution
        if bounds is not None and not (bounds[0] <= center[0] <= bounds[2] and bounds[1] <= center[1] <= bounds[3]):
            continue
        if any(np.linalg.norm(center-r['center']) < maximum_wall_thickness and
               np.dot(outward, r['outward_axis']) > .8 for r in ends):
            continue
        ends.append(dict(kind='wall_endpoint', center=center.tolist(),
                         seed_cells=cap.tolist(),
                         vertices=(np.asarray(origin[:2])+(cap+.5)*resolution).tolist(),
                         outward_axis=outward.tolist()))
    ends.sort(key=lambda r: tuple(r['center']))
    for i, region in enumerate(ends):
        region['name'] = 'wall_endpoint_%03d' % (i+1)
    return ends


def detect_corner_regions(occupied, resolution, origin, radius=.15,
                          minimum_wall_length=.25, minimum_turn_degrees=45., bounds=None):
    """Find physical wall turns supported by two sufficiently long segments.

    Contours use occupied cells only: unknown/free boundaries are not walls.
    Simplification suppresses single-cell staircase turns on oblique walls.
    The output remains map-specific and is recorded for visual inspection.
    """
    tolerance = .05
    if (not math.isfinite(minimum_wall_length) or minimum_wall_length < .15 or
            not math.isfinite(minimum_turn_degrees) or not 0 < minimum_turn_degrees < 170):
        raise ValueError('Invalid wall length or corner angle threshold')
    if bounds is not None and (len(bounds) != 4 or not all(math.isfinite(v) for v in bounds)
                               or bounds[0] >= bounds[2] or bounds[1] >= bounds[3]):
        raise ValueError('Bounds must be xmin ymin xmax ymax')
    contours, labels = _wall_contours(occupied, resolution, minimum_wall_length)
    candidates = []
    for contour in contours:
        polygon = cv2.approxPolyDP(contour, tolerance / resolution, True).reshape(-1, 2)
        if len(polygon) < 2:
            continue
        for index, point in enumerate(polygon):
            before, after = point - polygon[index-1], polygon[(index+1) % len(polygon)] - point
            length_a, length_b = np.linalg.norm(before), np.linalg.norm(after)
            if min(length_a, length_b) < 1e-9:
                continue
            turn = math.degrees(math.acos(np.clip(np.dot(before, after) / (length_a*length_b), -1, 1)))
            # A short cap or a near-180-degree fold on a single scan trace
            # is not a corner between two long walls. Both turn signs count.
            if not minimum_turn_degrees <= turn <= 170 or min(length_a, length_b)*resolution < minimum_wall_length:
                continue
            center = [float(origin[0] + (point[0]+.5)*resolution),
                      float(origin[1] + (point[1]+.5)*resolution)]
            if bounds is not None and not (bounds[0] <= center[0] <= bounds[2] and
                                           bounds[1] <= center[1] <= bounds[3]):
                continue
            if any(np.linalg.norm(np.array(center)-r['center']) < resolution*.5 for r in candidates):
                continue
            candidates.append(dict(center=center, selection_radius=radius,
                                   turn_degrees=float(turn),
                                   wall_lengths=[float(length_a*resolution), float(length_b*resolution)],
                                   axes=[float(math.atan2(before[1], before[0]) % math.pi),
                                         float(math.atan2(after[1], after[0]) % math.pi)],
                                   component=int(labels[point[1], point[0]])))
    candidates.sort(key=lambda r: tuple(r['center']))
    groups = []
    def axis_error(a, b):
        delta = abs(a-b) % math.pi
        return min(delta, math.pi-delta)
    for candidate in candidates:
        matched = None
        for group in groups:
            for member in group:
                if candidate['component'] != member['component']:
                    continue
                if np.linalg.norm(np.array(candidate['center'])-member['center']) > .20:
                    continue
                axes, other = candidate['axes'], member['axes']
                error = min(max(axis_error(axes[0], other[0]), axis_error(axes[1], other[1])),
                            max(axis_error(axes[0], other[1]), axis_error(axes[1], other[0])))
                if error < math.radians(25):
                    matched = group
                    break
            if matched is not None:
                break
        if matched is None:
            groups.append([candidate])
        else:
            matched.append(candidate)
    regions = []
    for group in groups:
        regions.append(dict(center=np.mean([v['center'] for v in group], axis=0).tolist(),
                            selection_radius=radius, vertices=[v['center'] for v in group],
                            evidence=[dict(turn_degrees=v['turn_degrees'], wall_lengths=v['wall_lengths'])
                                      for v in group]))
    for index, region in enumerate(regions):
        region['name'] = 'corner_%03d' % (index+1)
    return regions


def generate(map_path, config_path, output, all_corners=False, extra_clearance=.05,
             minimum_wall_length=.25, minimum_turn_degrees=45., bounds=None, scope='all',
             include_wall_endpoints=True, wall_endpoint_radius=.05):
    meta = yaml.safe_load(map_path.read_text())
    config = (dict(map=map_path.name, extra_clearance=extra_clearance)
              if all_corners else yaml.safe_load(config_path.read_text()))
    if map_path.name != config['map']:
        raise ValueError('Selections belong to a different map')
    if meta.get('negate', 0) or abs(meta['origin'][2]) > 1e-9:
        raise ValueError('Only unrotated standard occupancy maps are supported')
    source_image = image_path(map_path, meta)
    if output.with_suffix('.pgm').resolve() == source_image.resolve() or output.with_suffix('.yaml').resolve() == map_path.resolve():
        raise ValueError('The overlay must not overwrite the source map')
    original_bytes = source_image.read_bytes()
    pixels = np.flipud(np.array(Image.open(source_image).convert('L')))
    occupied = (255 - pixels.astype(float)) / 255 > meta['occupied_thresh']
    if not math.isfinite(meta['resolution']) or meta['resolution'] <= 0:
        raise ValueError('Map resolution must be positive and finite')
    if scope not in ['all', 'main_walls']:
        raise ValueError('Unsupported corner scope: ' + scope)
    effective_bounds = bounds
    if all_corners and bounds is None and scope == 'main_walls':
        effective_bounds = main_wall_bounds(occupied, meta['resolution'], meta['origin'])
    if all_corners:
        config['regions'] = detect_corner_regions(occupied, meta['resolution'], meta['origin'],
                                                  minimum_wall_length=minimum_wall_length,
                                                  minimum_turn_degrees=minimum_turn_degrees, bounds=effective_bounds)
        if include_wall_endpoints:
            config['regions'] += detect_wall_end_regions(occupied, meta['resolution'], meta['origin'],
                                                         minimum_wall_length, effective_bounds)
    if not math.isfinite(wall_endpoint_radius) or not 0 < wall_endpoint_radius <= .10:
        raise ValueError('Wall endpoint radius must be in (0, .10] meters')
    yy, xx = np.indices(pixels.shape)
    wx = meta['origin'][0] + (xx + .5) * meta['resolution']
    wy = meta['origin'][1] + (yy + .5) * meta['resolution']
    seeds = np.zeros(pixels.shape, dtype=bool)
    endpoint_seeds = np.zeros(pixels.shape, dtype=bool)
    selections = []
    for region in config['regions']:
        selected = np.zeros(pixels.shape, dtype=bool)
        # Preserve every member's neighborhood when two wall faces are
        # grouped as one physical corner for reporting/inspection.
        if region.get('kind') == 'wall_endpoint':
            cap = np.asarray(region['seed_cells'], dtype=int)
            selected[cap[:, 1], cap[:, 0]] = occupied[cap[:, 1], cap[:, 0]]
            endpoint_seeds |= selected
        else:
            radius = region['selection_radius']
            for x, y in region.get('vertices', [region['center']]):
                selected |= occupied & ((wx-x)**2 + (wy-y)**2 <= radius**2)
            seeds |= selected
        if not selected.any():
            raise ValueError('No occupied cells selected: ' + region['name'])
        selections.append(dict(region, cells=int(selected.sum())))
    extra = float(config['extra_clearance'])
    if not math.isfinite(extra) or not 0 < extra <= .10:
        raise ValueError('Extra clearance must be in (0, .10] meters')
    # Exact disk on the map lattice, including the no-corner/empty-seed case.
    # OpenCV is already a runtime dependency; avoid requiring SciPy on the car.
    def dilate_disk(seeds, radius):
        cells = int(math.ceil(radius / meta['resolution']))
        ky, kx = np.mgrid[-cells:cells+1, -cells:cells+1]
        disk = ((kx*kx+ky*ky)*meta['resolution']**2 <= (radius+1e-9)**2).astype(np.uint8)
        return cv2.dilate(seeds.astype(np.uint8), disk, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
    mask = dilate_disk(seeds, extra) | dilate_disk(endpoint_seeds, wall_endpoint_radius)
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
                  method='all_corners' if all_corners else 'selected_regions',
                  detection=dict(minimum_wall_length=minimum_wall_length,
                                 minimum_turn_degrees=minimum_turn_degrees,
                                 maximum_turn_degrees=170, bounds=effective_bounds, scope=scope,
                                 include_wall_endpoints=include_wall_endpoints,
                                 wall_endpoint_radius=wall_endpoint_radius,
                                 maximum_endpoint_wall_thickness=.12) if all_corners else None,
                  corner_count=sum(r.get('kind') != 'wall_endpoint' for r in selections),
                  endpoint_count=sum(r.get('kind') == 'wall_endpoint' for r in selections),
                  wall_endpoint_radius=wall_endpoint_radius,
                  selections=selections, extra_clearance=extra,
                  mask_cells=int(mask.sum()), added_free_cells=int((mask & ~occupied & (pixels>230)).sum()))
    output.with_suffix('.json').write_text(json.dumps(report, indent=2))
    assert source_image.read_bytes() == original_bytes
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--config', type=Path)
    mode.add_argument('--all-corners', action='store_true', help='Detect long-wall turns and thin-wall free ends')
    parser.add_argument('--no-wall-endpoints', action='store_true', help='Retain corner-only automatic detection')
    parser.add_argument('--wall-endpoint-radius', type=float, default=.05)
    parser.add_argument('--extra-clearance', type=float, default=.05, help='Meters, for --all-corners')
    parser.add_argument('--minimum-wall-length', type=float, default=.25)
    parser.add_argument('--minimum-turn-degrees', type=float, default=45.)
    parser.add_argument('--bounds', type=float, nargs=4, metavar=('XMIN', 'YMIN', 'XMAX', 'YMAX'))
    parser.add_argument('--scope', choices=['all', 'main_walls'], default='all')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(generate(args.map, args.config, args.output,
                              args.all_corners, args.extra_clearance,
                              args.minimum_wall_length, args.minimum_turn_degrees, args.bounds, args.scope,
                              not args.no_wall_endpoints, args.wall_endpoint_radius), indent=2))


if __name__ == '__main__':
    main()
