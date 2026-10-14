#!/usr/bin/env python3
"""Read-only calibration recorder. Never publishes commands or updates parameters."""
import argparse
import json
import math
from pathlib import Path
import threading
import time


def wrapped_delta(current, previous):
    return ((int(current) - int(previous) + 32768) % 65536) - 32768


def angle_delta(current, previous):
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('straight', 'rotate', 'wheel'), required=True)
    parser.add_argument('--truth', type=float, required=True,
                        help='实际距离(m)、实际有符号转角(deg)，或车轮实际有符号圈数')
    parser.add_argument('--wheel', choices=('left', 'right'), help='wheel 模式必填')
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, help='固定采样秒数；默认使用两次回车开始/结束')
    args = parser.parse_args()
    if not math.isfinite(args.truth) or args.truth == 0:
        parser.error('truth 必须为非零有限数；左转为正，右转为负')
    if args.mode == 'wheel' and not args.wheel:
        parser.error('wheel 模式需要 --wheel left 或 right')
    if args.mode == 'straight' and args.truth <= 0:
        parser.error('straight 模式使用向前行驶，truth 必须为正')
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('duration 必须为正有限数')
    out = Path(args.output)
    if out.exists():
        parser.error('输出文件已存在，请换一个名称，避免覆盖实测记录')

    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int16MultiArray
    rospy.init_node('record_odom_calibration', anonymous=True)
    params = {k: rospy.get_param('/odom_publisher/' + k, default)
              for k, default in [('wheel_diameter', .166224818), ('wheel_base', .408879504),
                                 ('ticks_per_rev', 4096), ('left_encoder_sign', 1),
                                 ('right_encoder_sign', 1), ('max_delta_ticks', 10000)]}
    lock = threading.Lock()
    last = {}; active = False
    totals = {'left': 0, 'right': 0, 'yaw': 0.0}
    events = []; issues = []; gaps = {'odom': [], 'encoder': []}

    def odom_cb(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        p = msg.pose.pose.position
        item = [time.monotonic(), msg.header.stamp.to_sec(), p.x, p.y, yaw,
                msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        with lock:
            previous = last.get('odom')
            if active and previous:
                totals['yaw'] += angle_delta(yaw, previous[4])
                gaps['odom'].append(item[0]-previous[0])
                if item[1] <= previous[1]: issues.append('odom 时间戳不递增')
                events.append(['odom', *item])
            last['odom'] = item

    def encoder_cb(msg):
        if len(msg.data) < 2:
            return
        item = [time.monotonic(), int(msg.data[1]), int(msg.data[0])]
        with lock:
            previous = last.get('encoder')
            if active and previous:
                gaps['encoder'].append(item[0]-previous[0])
                for side, index in [('left', 1), ('right', 2)]:
                    delta = wrapped_delta(item[index], previous[index])
                    if abs(delta) > params['max_delta_ticks']:
                        issues.append(side+' 编码器跳变；本组不能用于标定')
                    else:
                        sign = -1 if params[side+'_encoder_sign'] < 0 else 1
                        totals[side] += sign*delta
                events.append(['encoder', *item])
            last['encoder'] = item

    subscribers = [rospy.Subscriber('/odom', Odometry, odom_cb, queue_size=200),
                   rospy.Subscriber('/wheel_encoder', Int16MultiArray, encoder_cb, queue_size=200)]
    deadline = time.monotonic()+10
    while time.monotonic() < deadline and not rospy.is_shutdown():
        with lock:
            ready = all(t in last and time.monotonic()-last[t][0] < .3 for t in ('odom','encoder'))
        if ready: break
        time.sleep(.05)
    else:
        raise SystemExit('没有收到新鲜 /odom 与 /wheel_encoder；请先启动标定模式')
    print('当前参数：'+json.dumps(params), flush=True)
    print('本工具只记录，不驱动车辆。实际距离/角度必须由尺量或地面标线给出。', flush=True)
    if args.duration is None:
        input('确认小车停稳并对齐起点，按回车开始记录：')
    with lock:
        start = {k:list(v) for k,v in last.items()}
        if any(time.monotonic()-v[0] > .3 for v in start.values()):
            raise SystemExit('开始时反馈已过期；本组未记录')
        if abs(start['odom'][5]) > .01 or abs(start['odom'][6]) > .03:
            raise SystemExit('开始时小车未停稳；本组未记录')
        active = True
    print('记录中：用手柄完成指定动作，完全停稳后再结束记录。', flush=True)
    if args.duration is None:
        input('小车已停稳后，按回车结束记录：')
    else:
        time.sleep(args.duration)
    with lock:
        active = False
        end = {k:list(v) for k,v in last.items()}
        for topic in ('odom','encoder'):
            if time.monotonic()-end[topic][0] > .3:
                issues.append(topic+' 结束时反馈过期')
            if gaps[topic] and max(gaps[topic]) > .5:
                issues.append(topic+' 采样间隔超过0.5秒，建议重测')
        if abs(end['odom'][5]) > .01 or abs(end['odom'][6]) > .03:
            issues.append('结束时小车未停稳，建议重测')
    truth = args.truth
    if args.duration is None:
        unit = {'straight':'米', 'rotate':'度（左正右负）', 'wheel':'车轮圈数'}[args.mode]
        while True:
            entered = input(f'请输入实际测量值，单位 {unit}；回车确认 {truth}：').strip()
            try:
                candidate = float(entered) if entered else truth
                if not math.isfinite(candidate) or candidate == 0 or (args.mode == 'straight' and candidate < 0):
                    raise ValueError
                truth = candidate
                break
            except ValueError:
                print('请输入非零有限数；直线距离必须为正。', flush=True)
    scale = math.pi*params['wheel_diameter']/params['ticks_per_rev']
    encoder_distance = scale*(totals['left']+totals['right'])/2
    encoder_angle = scale*(totals['right']-totals['left'])/params['wheel_base']
    dx=end['odom'][2]-start['odom'][2]; dy=end['odom'][3]-start['odom'][3]
    heading=start['odom'][4]
    report = {'mode':args.mode, 'truth':truth, 'wheel':args.wheel, 'params':params,
              'start':start, 'end':end, 'delta_ticks':dict(left=totals['left'],right=totals['right']),
              'encoder_distance_m':encoder_distance, 'odom_forward_m':dx*math.cos(heading)+dy*math.sin(heading),
              'odom_lateral_m':-dx*math.sin(heading)+dy*math.cos(heading),
              'odom_turn_deg':math.degrees(totals['yaw']), 'encoder_turn_deg':math.degrees(encoder_angle),
              'max_gap_s':{k:max(v) if v else None for k,v in gaps.items()}, 'issues':issues}
    if args.mode == 'straight':
        if encoder_distance <= .05 or totals['left'] <= 0 or totals['right'] <= 0:
            issues.append('未检测到足够的正向直线运动，请检查方向或重新测量')
        if not issues:
            report['candidate_wheel_diameter_m']=params['wheel_diameter']*truth/encoder_distance
    elif args.mode == 'rotate':
        true_angle=math.radians(truth)
        if abs(encoder_angle) < .1 or encoder_angle*true_angle <= 0:
            issues.append('转角过小或方向与真值不一致，请检查方向或重新测量')
        if not issues:
            report['candidate_wheel_base_m']=params['wheel_base']*encoder_angle/true_angle
    else:
        count=totals[args.wheel]
        if abs(count) < 100 or count*truth <= 0:
            issues.append('轮圈计数过小或方向与真值不一致')
        if not issues:
            report['measured_ticks_per_rev']=count/truth
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x',encoding='utf-8') as f:
        json.dump({**report,'events':events},f,ensure_ascii=False,indent=2)
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    print('已保存：'+str(out)+'；候选值未自动写入参数。',flush=True)


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        raise SystemExit('记录中断；未生成校准结果。请先用手柄停止小车。')
