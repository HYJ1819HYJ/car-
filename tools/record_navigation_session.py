#!/usr/bin/env python3
"""Bounded background rosbag recording. Subscribes only; never commands hardware."""
import argparse,datetime,json,os,signal,subprocess,sys,time,shutil
from pathlib import Path
P=argparse.ArgumentParser();P.add_argument('--worker',type=Path);P.add_argument('--minutes',type=float,default=30);a=P.parse_args()
root=Path(__file__).resolve().parent.parent
if not a.worker:
    import rosgraph
    rosgraph.Master('/recording_preflight').getPid()
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    folder=root/'outputs/navigation_records'/('navigation_'+stamp);folder.mkdir(parents=True)
    with (folder/'supervisor.log').open('w') as out:
        child=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--worker',str(folder),'--minutes',str(a.minutes)],stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT,start_new_session=True,close_fds=True)
    print(json.dumps({'directory':str(folder),'supervisor_pid':child.pid,'maximum_minutes':a.minutes}),flush=True);sys.exit(0)
folder=a.worker;folder.mkdir(exist_ok=True,parents=True)
(folder/'supervisor.pid').write_text(str(os.getpid())+'\n')
import rosgraph,rospy
rospy.init_node('navigation_recording_monitor',anonymous=True,disable_signals=True)
stop=False

def finish(signum,frame):
    global stop
    stop=True
signal.signal(signal.SIGINT,finish);signal.signal(signal.SIGTERM,finish)
params={ns:rospy.get_param(ns,{}) for ns in ['/move_base','/serial_bridge','/odom_publisher','/amcl','/richbeam_lidar','/scan_self_filter','/corner_clearance/map_provider']}
processes=[]
for p in Path('/proc').iterdir():
    if p.name.isdigit():
        try:
            argv=(p/'cmdline').read_bytes().decode(errors='replace').split('\0')
            if any('/roslaunch' in s or '/serial_bridge_node' in s for s in argv):processes.append({'pid':int(p.name),'argv':[s for s in argv if s]})
        except OSError:pass
metadata={'started_at':datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),'master':os.environ.get('ROS_MASTER_URI','http://localhost:11311'),'ROS_DISTRO':os.environ.get('ROS_DISTRO'),'maximum_minutes':a.minutes,'topics_at_start':rospy.get_published_topics(),'parameters':params,'processes':processes}
assert metadata['ROS_DISTRO']=='noetic'
(folder/'start_snapshot.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
# Preserve the exact static map used by this launch, even if it is overwritten later.
for entry in processes:
    argv=entry['argv']
    if 'navigation_stack.launch' in argv:
        for item in argv:
            if item.startswith('map_file:='):
                path=Path(item.split(':=',1)[1])
                if path.is_file():
                    shutil.copy2(path,folder/path.name)
                    import yaml
                    image=Path(yaml.safe_load(path.read_text())['image'])
                    if not image.is_absolute():image=path.parent/image
                    if image.is_file():shutil.copy2(image,folder/image.name)
topics=['/map','/corner_clearance_map','/map_metadata','/scan','/scan_obstacles','/scan_raw','/scan_self_filtered_count','/tf','/tf_static','/odom','/wheel_encoder','/cmd_vel','/stm32_command_sent','/stm32_feedback_text','/control_source','/safety_stop','/safety_collision_prediction','/front_obstacle_distance','/rear_obstacle_distance','/nearest_obstacle_radius','/joystick_lb_pressed','/move_base/status','/move_base/goal','/move_base/current_goal','/move_base_simple/goal','/move_base/result','/move_base/feedback','/move_base/recovery_status','/move_base/GlobalPlanner/plan','/move_base/NavfnROS/plan','/move_base/DWAPlannerROS/global_plan','/move_base/DWAPlannerROS/local_plan','/move_base/global_costmap/costmap','/move_base/global_costmap/costmap_updates','/move_base/global_costmap/footprint','/move_base/local_costmap/costmap','/move_base/local_costmap/costmap_updates','/move_base/local_costmap/footprint','/amcl_pose','/initialpose','/rosout_agg','/goal_reached','/goal_detected','/blue_target/observation']
topics.extend(['/move_base/SmoothGlobalPlanner/plan',
               '/move_base/SmoothGlobalPlanner/raw_plan',
               '/move_base/SmoothGlobalPlanner/diagnostics'])
metadata['recorded_topics']=topics
(folder/'start_snapshot.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
log=(folder/'rosbag_record.log').open('w')
record=subprocess.Popen(['rosbag','record','--lz4','--split','--duration=300','--buffsize=128','-O',str(folder/'navigation')]+topics+['__name:=navigation_fault_record'],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
(folder/'recorder.pid').write_text(str(record.pid)+'\n')
start=time.monotonic();ready=False
try:
    while not stop and time.monotonic()-start<a.minutes*60:
        if record.poll() is not None:raise RuntimeError('rosbag exited with status '+str(record.returncode))
        active=list(folder.glob('*.bag.active'))
        ready=ready or bool(active)
        status={'state':'recording' if ready else 'starting','seconds_elapsed':round(time.monotonic()-start,1),'recorder_pid':record.pid,'active_files':[p.name for p in active],'bags':[p.name for p in folder.glob('*.bag')],'updated_at':datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat()}
        (folder/'status.json').write_text(json.dumps(status,ensure_ascii=False,indent=2)+'\n')
        time.sleep(1)
finally:
    if record.poll() is None:
        record.send_signal(signal.SIGINT)
        try:record.wait(timeout=20)
        except subprocess.TimeoutExpired:
            record.terminate();record.wait(timeout=5)
    log.close()
    final={'state':'stopped','duration_s':round(time.monotonic()-start,1),'recorder_exit_code':record.returncode,'bags':[p.name for p in folder.glob('*.bag')],'unfinished_files':[p.name for p in folder.glob('*.bag.active')],'ended_at':datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat()}
    (folder/'status.json').write_text(json.dumps(final,ensure_ascii=False,indent=2)+'\n')
    info=[]
    for bag in sorted(folder.glob('*.bag')):
        output=subprocess.run(['rosbag','info','--yaml',str(bag)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        (folder/(bag.name+'.info.yaml')).write_text(output.stdout);info.append({'file':bag.name,'bytes':bag.stat().st_size,'info_exit_code':output.returncode})
    (folder/'completed_files.json').write_text(json.dumps(info,indent=2)+'\n')
