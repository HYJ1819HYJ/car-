"""Prepare and validate per-map corner overlays before publishing them to ROS."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
import yaml
from .corner_clearance import generate, image_path

GENERATOR_VERSION = 2


def validate(source_path, mask_path):
    source, mask = [yaml.safe_load(p.read_text()) for p in [source_path, mask_path]]
    if source.get('negate', 0) or abs(source['origin'][2]) > 1e-9:
        raise ValueError('Only unrotated standard occupancy maps are supported')
    report = json.loads(mask_path.with_suffix('.json').read_text())
    if report['map'] != source_path.name:
        raise ValueError('Corner selections belong to another map')
    for key in ['resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh']:
        if mask[key] != source[key]:
            raise ValueError('Corner mask metadata mismatch: ' + key)
    for meta, path, key in [(source, source_path, 'source_sha256'), (mask, mask_path, 'mask_sha256')]:
        if hashlib.sha256(image_path(path, meta).read_bytes()).hexdigest() != report[key]:
            raise ValueError('Map or mask changed; regenerate corner selections')
    return report


def compatible(report, extra, length, angle, bounds, scope, endpoints=True, endpoint_radius=.05):
    if report.get('method') != 'all_corners' or report['extra_clearance'] != extra:
        return False
    detection = report.get('detection') or {}
    if detection.get('include_wall_endpoints', False) != endpoints:
        return False
    if endpoints and detection.get('wall_endpoint_radius') != endpoint_radius:
        return False
    if detection.get('minimum_wall_length') != length or detection.get('minimum_turn_degrees') != angle:
        return False
    if bounds is not None:
        return detection.get('bounds') == list(bounds)
    if scope == 'all':
        return detection.get('scope', 'all') == 'all' and detection.get('bounds') is None
    # A source-matched, explicitly bounded sidecar is a reviewed map-specific
    # selection. Preserve it instead of expanding protection to outside clutter.
    return detection.get('scope', 'all') == scope or detection.get('bounds') is not None


def prepare(source_path, cache_root, explicit_mask=None, extra=.05, length=.25,
            angle=45., bounds=None, scope='main_walls', endpoints=True, endpoint_radius=.05):
    source_path = Path(source_path).resolve()
    if explicit_mask:
        mask = Path(explicit_mask).resolve()
        report = validate(source_path, mask)
        if report['extra_clearance'] != extra:
            raise ValueError('Explicit mask clearance does not match requested clearance')
        detection = report.get('detection') or {}
        if endpoints and (not detection.get('include_wall_endpoints') or
                          detection.get('wall_endpoint_radius') != endpoint_radius):
            raise ValueError('Explicit mask lacks requested wall-end protection; regenerate it or disable wall endpoints')
        return mask, report, 'explicit'
    preferred = source_path.with_name(source_path.stem+'_corner_clearance.yaml')
    try:
        report = validate(source_path, preferred)
        if compatible(report, extra, length, angle, bounds, scope, endpoints, endpoint_radius):
            return preferred, report, 'reviewed_sidecar'
    except (OSError, ValueError, KeyError, TypeError):
        pass
    meta = yaml.safe_load(source_path.read_text())
    fingerprint = dict(map=source_path.name, metadata=meta,
                       image_sha256=hashlib.sha256(image_path(source_path, meta).read_bytes()).hexdigest(),
                       version=GENERATOR_VERSION, extra=extra, length=length, angle=angle,
                       bounds=bounds, scope=scope, endpoints=endpoints, endpoint_radius=endpoint_radius)
    key = hashlib.sha256(json.dumps(fingerprint, sort_keys=True, allow_nan=False).encode()).hexdigest()
    cache_root = Path(cache_root).expanduser()
    cache_root.mkdir(parents=True, exist_ok=True)
    folder = cache_root/key
    mask = folder/'overlay.yaml'
    with (cache_root/(key+'.lock')).open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            report = validate(source_path, mask)
            if report.get('generator_version') == GENERATOR_VERSION:
                return mask, report, 'cache'
        except (OSError, ValueError, KeyError, TypeError):
            pass
        # Publish only a completely written generation. Leave source/sidecars
        # intact, including a stale sidecar that a user may need to inspect.
        with tempfile.TemporaryDirectory(prefix='generating-', dir=cache_root) as temporary:
            output = Path(temporary)/'overlay'
            report = generate(source_path, None, output, True, extra, length, angle, bounds, scope,
                              endpoints, endpoint_radius)
            report['generator_version'] = GENERATOR_VERSION
            output.with_suffix('.json').write_text(json.dumps(report, indent=2))
            validate(source_path, output.with_suffix('.yaml'))
            if folder.exists():
                # Rebuild an invalid cache without replacing a loaded map's
                # source. The lock excludes another generator for this key.
                import shutil
                shutil.rmtree(folder)
            os.replace(temporary, folder)
        return mask, report, 'generated'
