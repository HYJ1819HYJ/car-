#!/usr/bin/env python3
"""Run complete closed-loop scenarios, one isolated ROS master at a time."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import time

from PIL import Image,ImageDraw

ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/'src/competition_sim'
WRAPPER=ROOT.parent/'xiaoche-ros1.sh'
OUT=ROOT/'outputs/added_obstacles_20261019'


def master_running():
    with socket.socket() as connection:
        connection.settimeout(.2)
        return connection.connect_ex(('127.0.0.1',11329))==0


def stop_simulation():
    if master_running():
        subprocess.run([str(WRAPPER),'env','ROS_MASTER_URI=http://127.0.0.1:11329',
                        'rosnode','kill','/competition_stage'],stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL,timeout=15)
    end=time.monotonic()+20
    while master_running() and time.monotonic()<end:
        time.sleep(.2)
    if master_running():
        raise RuntimeError('The previous simulation master did not shut down')


def plot_trace(case,result,directory):
    picture=Image.open(PKG/'scenarios'/case/'preview.png').convert('RGB')
    draw=ImageDraw.Draw(picture)
    h=picture.height//3
    points=[((p[1]+.3)/.025*3,(h-1-(p[2]+.3)/.025)*3) for p in result['trace']]
    if len(points)>1:draw.line(points,fill='#145ecb',width=4)
    draw.text((15,28),result['state']+' | '+str(round(result['distance_m'],2))+' m',fill='#145ecb')
    picture.save(directory/'trajectory.png')


def save_summary(summary):
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    lines=['# 建图后新增两固定箱体：闭环测试','',
           '导航共用原四箱底图；每组的六箱实景地图不输入导航。','',
           '| 场景 | 验收 | 导航状态 | 距离 m | 新箱最近间隔 cm | 碰撞采样 |',
           '| --- | --- | --- | --- | --- | --- |']
    for row in summary:
        distance=f'{row["distance_m"]:.2f}' if 'distance_m' in row else '—'
        clearance=f'{row["min_added_clearance_m"]*100:.1f}' if row.get('min_added_clearance_m') is not None else '—'
        collisions=str(row.get('collision_samples',0)+row.get('added_collision_samples',0)) if 'state' in row else '—'
        lines.append(f'| {row["scenario"]} | {"通过" if row["accepted"] else "未通过"} | '
                     f'{row.get("state",row.get("error",""))} | {distance} | {clearance} | {collisions} |')
    lines+=['','封堵组的预期为安全停车并报告无法规划，因此 FAILED 可以是该组的正确行为。',
             '逐组 result.json 保存原图一致性、激光/代价地图观测和最终实际速度；trajectory.png 保存实际轨迹。','']
    (OUT/'summary.md').write_text('\n'.join(lines))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenarios',nargs='*',help='Default: all six cases')
    args=parser.parse_args()
    catalogue=json.loads((PKG/'scenarios/catalogue.json').read_text())
    selected=args.scenarios or [c['id'] for c in catalogue]
    definitions={c['id']:c for c in catalogue}
    if master_running():
        raise SystemExit('Stop the currently running simulation before starting the suite.')
    summary=[]
    OUT.mkdir(parents=True,exist_ok=True)
    for case in selected:
        definition=definitions[case]
        directory=OUT/case
        directory.mkdir(parents=True,exist_ok=True)
        result_path=directory/'result.json'
        if result_path.exists():
            result_path.rename(directory/('previous_result_'+str(int(time.time()))+'.json'))
        print('START '+case,flush=True)
        with (directory/'launch.log').open('w') as log:
            process=subprocess.Popen([str(ROOT/'start-competition-sim.sh'),
                                      'scenario:='+case,'show_rviz:=false'],
                                     stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+175
            try:
                while not result_path.exists() and time.monotonic()<deadline:
                    if process.poll() is not None:
                        raise RuntimeError(f'{case}: roslaunch exited with {process.returncode}')
                    time.sleep(.25)
                if not result_path.exists():
                    raise RuntimeError(f'{case}: report timeout')
                result=json.loads(result_path.read_text())
                collision_free=result['collision_samples']==0 and result['added_collision_samples']==0
                seen=all(n>0 for n in result['laser_frames_seen'].values())
                mapped=all(n>0 for n in result['costmap_frames_seen'].values())
                expected=(result['state']=='SUCCEEDED' if definition['expected']=='reach_goal'
                          else result['state']!='SUCCEEDED' and result['stationary_final'] and
                               max(abs(n) for n in result['final_actual_velocity'])<.005)
                accepted=bool(expected and collision_free and result['base_navigation_map_matches']
                              and seen and mapped and result['reverse_commands']==0)
                record={'scenario':case,'title':definition['title'],'expected':definition['expected'],
                        'accepted':accepted,**{k:v for k,v in result.items()
                                             if k not in ['trace','added_boxes','scenario','expected']}}
                plot_trace(case,result,directory)
            except Exception as error:
                record={'scenario':case,'accepted':False,'error':str(error)}
            finally:
                stop_simulation()
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:process.terminate()
        summary.append(record)
        save_summary(summary)
        print('RESULT '+json.dumps({k:v for k,v in record.items() if k in
              ['scenario','accepted','state','error','distance_m','added_collision_samples',
               'collision_samples','laser_frames_seen','costmap_frames_seen','stationary_final']},
              ensure_ascii=False),flush=True)
    print(f'SUMMARY {sum(r["accepted"] for r in summary)}/{len(summary)} accepted',flush=True)
    raise SystemExit(0 if all(r['accepted'] for r in summary) else 1)


if __name__=='__main__':
    main()
