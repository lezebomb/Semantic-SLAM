#!/usr/bin/env python3
"""Run inspected bag with scoped processes, fresh output, and retained evidence."""
import argparse
import datetime
import json
import os
import platform
import signal
import shutil
import sqlite3
import subprocess
import sys
import time
import psutil
from pathlib import Path
from inspect_bag import inspect_bag

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser()
    workspace = Path(os.environ.get('RTABMAP_WS', str(Path.home() / 'ros2')))
    jetson_bag = ROOT/'tests/demo_mapping.db3'
    default_bag = jetson_bag if jetson_bag.is_file() else workspace/'data/demo_mapping/demo_mapping_bag/demo_mapping.db3'
    p.add_argument('--bag',default=str(default_bag))
    p.add_argument('--model',type=Path,default=ROOT/'models/yolov8n-seg.pt')
    p.add_argument('--rate',type=float,choices=[0.25,0.5],default=0.5)
    p.add_argument('--geometry-only',action='store_true',help='Run geometry first, without semantic nodes')
    p.add_argument('--no-rviz',action='store_true')
    p.add_argument('--domain-id',type=int,default=77)
    p.add_argument('--duration',type=float,help='Wall seconds of playback for a smoke run; default is full bag')
    p.add_argument('--hold',action='store_true',help='Keep map and RViz visible after playback until Ctrl+C')
    p.add_argument('--allow-empty', action='store_true',
                   help='Validate geometry/segmentation interface even when all masks are empty')
    p.add_argument('--confidence',type=float,default=0.35)
    p.add_argument('--image-size',type=int,default=640)
    p.add_argument('--device',default=os.environ.get('SEMANTIC_DEVICE', '0' if platform.machine() == 'aarch64' else 'cpu'),
                   help='YOLO device: default GPU 0 on Jetson, CPU on this local x86 environment')
    args=p.parse_args()
    if args.duration is not None and args.duration <= 0:
        p.error('--duration must be positive')
    if not 0 <= args.domain_id <= 232:
        p.error('--domain-id must be between 0 and 232')
    if not 0 < args.confidence <= 1 or args.image_size <= 0:
        p.error('--confidence must be in (0, 1] and --image-size must be positive')
    bag=inspect_bag(args.bag)
    args.model=args.model.expanduser().resolve()
    if not args.model.is_file():
        raise RuntimeError('Missing model checkpoint: ' + str(args.model))
    suffix=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d-%H%M%S')
    out=ROOT/'runs'/f'{suffix}-{os.getpid()}'
    out.mkdir(parents=True,exist_ok=False)
    (ROOT/'artifacts').mkdir(exist_ok=True)
    (ROOT/'artifacts/latest_run.txt').write_text(str(out)+'\n')
    (out/'bag_inspection.json').write_text(json.dumps(bag,indent=2))
    env=os.environ.copy()
    # VS Code Snap exports GTK/GIO paths whose core20 libc conflicts with Ubuntu 22.04 Qt.
    for key in ('GTK_PATH','GTK_EXE_PREFIX','GIO_MODULE_DIR','GIO_EXTRA_MODULES',
                'QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH','QT_QPA_FONTDIR',
                'GDK_PIXBUF_MODULE_FILE','GDK_PIXBUF_MODULEDIR'):
        if '/snap/' in env.get(key,''):
            env.pop(key,None)
    for key in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_QPA_FONTDIR'):
        env.pop(key, None)
    env.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1',ROS_HOME=str(out/'ros_home'),
               ROS_LOG_DIR=str(out/'ros_logs'),XDG_CONFIG_HOME=str(out/'gui_config'),
               XDG_CACHE_HOME=str(out/'gui_cache'),YOLO_CONFIG_DIR=str(out/'yolo_config'),
               MPLCONFIGDIR=str(out/'matplotlib'),RCUTILS_COLORIZED_OUTPUT='0',
               OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='2',
               PYTHONUNBUFFERED='1')
    for key in ('ROS_HOME','ROS_LOG_DIR','XDG_CONFIG_HOME','XDG_CACHE_HOME','MPLCONFIGDIR'):
        Path(env[key]).mkdir(parents=True,exist_ok=True)
    (Path(env['YOLO_CONFIG_DIR'])/'Ultralytics').mkdir(parents=True,exist_ok=True)
    logs=[];player=None;launch=None;observer=None;telemetry=None
    resource_samples=[]
    processes={}
    last_resource_sample=0

    def sample_resources():
        nonlocal last_resource_sample
        if time.monotonic()-last_resource_sample<5:return
        last_resource_sample=time.monotonic()
        items=[]
        for proc in [psutil.Process()]+psutil.Process().children(recursive=True):
            try:
                key=(proc.pid,proc.create_time())
                if key not in processes: processes[key]=proc;proc.cpu_percent()
                p=processes[key]
                items.append(dict(pid=p.pid,name=p.name(),cpu_percent=p.cpu_percent(),rss_MiB=p.memory_info().rss/1024**2))
            except psutil.Error:pass
        resource_samples.append(dict(wall_time=time.time(),processes=items,
            cpu_percent_sum=sum(i["cpu_percent"] for i in items),
            rss_sum_MiB=sum(i["rss_MiB"] for i in items),system_memory_used_MiB=psutil.virtual_memory().used/1024**2))
        (out/"resources.json").write_text(json.dumps(resource_samples,indent=2))
    events=[]

    def event(kind,**data):
        e=dict(event=kind,wall_time=time.time(),**data);events.append(e)
        (out/'events.json').write_text(json.dumps(events,indent=2))
        print(json.dumps(e,ensure_ascii=False),flush=True)

    def spawn(cmd,name):
        log=(out/(name+'.log')).open('w',buffering=1);logs.append(log)
        proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        event('started',name=name,pid=proc.pid,cmd=cmd)
        return proc

    def check():
        if launch.poll() is not None:
            raise RuntimeError('Launch terminated; inspect launch.log')
        text=(out/'launch.log').read_text(errors='replace')
        if 'process has died' in text or 'Traceback (most recent call last)' in text:
            raise RuntimeError('Pipeline node failed; inspect launch.log')
        if observer and observer.poll() is not None:
            raise RuntimeError('Verifier terminated; inspect observer.log')
        return text

    def stop(proc,name):
        if proc is None or proc.poll() is not None:return
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5)
        event('stopped',name=name,returncode=proc.returncode)

    success=False
    try:
        event('configuration',geometry_only=args.geometry_only,output=str(out),domain_id=args.domain_id,rate=args.rate,
              bag_duration_s=bag['duration_s'],selected_topics=bag['selected'])
        cmd=['ros2','launch',str(ROOT/'launch/indoor_semantic.launch.py'),
             'output_dir:='+str(out),'rviz:='+str(not args.no_rviz).lower(),
             'confidence:='+str(args.confidence),'image_size:='+str(args.image_size),'device:='+args.device,
             'model_path:='+str(args.model), 'semantics:='+str(not args.geometry_only).lower()]
        cmd += [k+':='+bag['selected'][k] for k in ('rgb_base','depth_base','camera_info')]
        if shutil.which('tegrastats'):
            telemetry=spawn(['tegrastats','--interval','1000'],'tegrastats')
        else:
            event('tegrastats_unavailable', note='No Jetson resource telemetry on this host')
        (ROOT/'logs').mkdir(exist_ok=True)
        (ROOT/'logs/active_run.json').write_text(json.dumps(dict(pid=os.getpid(),
            start_ticks=Path('/proc/self/stat').read_text().split()[21], script=str(Path(__file__).resolve()),output=str(out))))
        launch=spawn(cmd,'launch')
        deadline=time.monotonic()+90
        while True:
            text=check()
            if args.geometry_only and text.count('process started with pid')>=2:
                time.sleep(3);check();break
            if not args.geometry_only and all(marker in text for marker in ('YOLO_READY','PROJECTOR_READY','GRID_READY')):break
            if time.monotonic()>deadline:raise RuntimeError('Pipeline startup timed out')
            time.sleep(0.5)
        observer_cmd = [sys.executable,str(ROOT/'scripts/observe_ros.py'),str(out)]
        if args.geometry_only:
            observer_cmd.append('--geometry-only')
        if args.allow_empty:
            observer_cmd.append('--allow-empty')
        observer=spawn(observer_cmd,'observer')
        time.sleep(3)
        check()
        player=spawn(['ros2','bag','play',bag['bag_db'],'--clock','50','--rate',str(args.rate),
                      '--disable-keyboard-controls','--delay','2'],'bag')
        started=time.monotonic()
        last_report=started
        while player.poll() is None:
            check();sample_resources();time.sleep(1)
            elapsed=time.monotonic()-started
            if time.monotonic()-last_report>=30:
                path=out/'verification.json'
                v=json.loads(path.read_text()) if path.exists() else {}
                event('progress',wall_elapsed_s=round(elapsed,1),success=v.get('success'),
                      map_points=v.get('topics',{}).get('/semantic/map',{}).get('points'),
                      detected_frames=v.get('detector_frames_with_instances'))
                last_report=time.monotonic()
            if args.duration and elapsed>=args.duration:
                stop(player,'bag');break
        if player.returncode not in (0,-signal.SIGINT,130):
            raise RuntimeError(f'Bag player exited with {player.returncode}')
        event('playback_finished',wall_elapsed_s=round(time.monotonic()-started,2),
              full_bag=args.duration is None,returncode=player.returncode)
        for _ in range(6):check();time.sleep(1)
        v=json.loads((out/'verification.json').read_text())
        success=v['success']
        event('verification',success=success,tf_checks=v['tf_checks'],
              max_tf_point_error_m=v['max_tf_point_error_m'])
        if not success:raise RuntimeError('End-to-end evidence incomplete; inspect verification.json')
        if not args.no_rviz:
            try:
                from capture_rviz import capture
                capture(out/'rviz_live.png')
                event('rviz_screenshot',path=str(out/'rviz_live.png'))
            except Exception as error:
                event('screenshot_unavailable',message=str(error))
        if args.hold:
            event('holding',message='RViz and map remain alive; Ctrl+C saves and stops this run')
            while True:check();time.sleep(1)
    except KeyboardInterrupt:
        event('interrupted')
    except Exception as e:
        success=False
        event('error',message=str(e))
    finally:
        stop(player,'bag');stop(launch,'launch');stop(observer,'observer');stop(telemetry,'tegrastats')
        for log in logs:log.close()
        db=out/'rtabmap.db'
        database={}
        if db.exists():
            with sqlite3.connect(db.as_uri()+'?mode=ro',uri=True) as con:
                result=con.execute('PRAGMA quick_check').fetchone()[0]
                count=con.execute('SELECT count(*) FROM Node').fetchone()[0]
            database=dict(bytes=db.stat().st_size,quick_check=result,nodes=count)
            event('database',path=str(db),**database)
        node_stats={}
        for name,field in ([] if args.geometry_only else [('detector','failures'),('projector','errors')]):
            path=out/(name+'_stats.json')
            stats=json.loads(path.read_text()) if path.exists() else {}
            node_stats[name]=stats.get(field)
        shutdown_warnings=[]
        node_shutdown_errors=[]
        path=out/'launch.log'
        if path.exists():
            for line in path.read_text(errors='replace').splitlines():
                if '[ERROR]' in line and 'process has died' in line:
                    if '[rviz2-' in line:
                        shutdown_warnings.append(line)
                    else:
                        node_shutdown_errors.append(line)
        success=bool(success and database.get('quick_check')=='ok' and
                     all(value==0 for value in node_stats.values()) and not node_shutdown_errors)
        (out/'result.json').write_text(json.dumps(dict(success=success,output=str(out),
               rate=args.rate,geometry_only=args.geometry_only,full_bag=args.duration is None,database=database,
               node_errors=node_stats,node_shutdown_errors=node_shutdown_errors,
               gui_shutdown_forced=any('exit code -9' in line for line in shutdown_warnings),
               shutdown_warnings=shutdown_warnings),indent=2))
        print('Results: '+str(out),flush=True)
        active=ROOT/'logs/active_run.json'
        if active.exists() and json.loads(active.read_text()).get('pid')==os.getpid(): active.unlink()
    return 0 if success else 1


if __name__=='__main__':
    sys.exit(main())
