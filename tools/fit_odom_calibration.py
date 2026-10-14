#!/usr/bin/env python3
"""Fit manually measured calibration trials; does not access ROS or write settings."""
import argparse
import json
import math
from pathlib import Path


def fit(records, mode):
    if len(records) < 3:
        raise ValueError('至少需要三组有效实测记录')
    keys = ('wheel_diameter', 'wheel_base', 'ticks_per_rev', 'left_encoder_sign', 'right_encoder_sign')
    reference = records[0]['params']
    for r in records:
        if r['mode'] != mode or r['issues']:
            raise ValueError('记录模式不符或存在采样问题；先剔除无效组')
        if any(r['params'][k] != reference[k] for k in keys):
            raise ValueError('同批记录的里程计参数不同；请分开拟合')
    if mode == 'straight':
        observed = [r['encoder_distance_m'] for r in records]
        truth = [r['truth'] for r in records]
        if any(x <= 0 for x in observed+truth):
            raise ValueError('直线记录必须为正向有效距离')
        ratio = sum(x*y for x,y in zip(observed,truth))/sum(x*x for x in observed)
        result = reference['wheel_diameter']*ratio
        key = 'wheel_diameter'
    else:
        observed = [math.radians(r['encoder_turn_deg']) for r in records]
        truth = [math.radians(r['truth']) for r in records]
        if any(abs(t) < .1 or o*t <= 0 for o,t in zip(observed,truth)):
            raise ValueError('转向记录的角度过小或方向不一致')
        ratio = sum(o*t for o,t in zip(observed,truth))/sum(t*t for t in truth)
        result = reference['wheel_base']*ratio
        key = 'wheel_base'
    if not math.isfinite(result) or result <= 0:
        raise ValueError('拟合结果无效')
    return {**reference, key:result}, key


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=('straight','rotate'), required=True)
    p.add_argument('--value-only', action='store_true')
    p.add_argument('files', nargs='+')
    args = p.parse_args()
    try:
        records = [json.loads(Path(name).read_text()) for name in args.files]
        params,key = fit(records,args.mode)
    except (ValueError, KeyError, OSError) as exc:
        p.error(str(exc))
    if args.value_only:
        print(f'{params[key]:.9f}')
    else:
        print(json.dumps({'trials':len(records), 'candidate_params':params},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
