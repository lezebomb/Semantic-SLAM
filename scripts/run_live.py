#!/usr/bin/env python3
"""RGB-D hand-pushed RTAB-Map + YOLO. No wheel, LiDAR or chassis control."""
import argparse,datetime,fcntl,hashlib,json,math,os,platform,shutil,signal,sqlite3,subprocess,sys,time
from pathlib import Path
import psutil,yaml
from mapping_config import ROOT,load_config,input_topics,record_topics,tf_topics
MODEL_SHA='a7cd8f929e1903d78a12a48efecab430209f18dc46cb96c3599a5980c63c423c'

def arguments():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'config/robot_mapping.yaml')
    p.add_argument('--output',type=Path)
    p.add_argument('--replay',type=Path,help='Replay only raw RGB-D and camera static calibration on an isolated Domain')
    p.add_argument('--rate',type=float,default=.5)
    p.add_argument('--duration',type=float,help='Seconds after READY; live default 600, replay default until end; 0 until stopped')
    p.add_argument('--device',default=os.environ.get('SEMANTIC_DEVICE','0' if platform.machine()=='aarch64' else 'cpu'))
    p.add_argument('--rviz',action='store_true')
    p.add_argument('--data-only',action='store_true',help='Compatibility flag: this baseline always runs data/mapping only')
    p.add_argument('--no-video',action='store_true')
    p.add_argument('--video',dest='no_video',action='store_false')
    p.set_defaults(no_video=False)
    p.add_argument('--no-record',action='store_true')
    p.add_argument('--check',action='store_true',help='Software/model/NMS check only; no ROS nodes or hardware')
    p.add_argument('--dry-run','--show-args',dest='dry_run',action='store_true')
    p.add_argument('--status',action='store_true')
    a=p.parse_args()
    if not math.isfinite(a.rate) or not 0<a.rate<=1:p.error('--rate must be in (0,1]')
    if a.duration is not None and (not math.isfinite(a.duration) or a.duration<0):p.error('Invalid duration')
    if a.replay and not a.replay.exists():p.error('Replay input does not exist')
    return a

def check_software(a,c):
    from ament_index_python.packages import get_package_prefix
    for pkg in ('rtabmap_slam','rtabmap_odom','rtabmap_msgs','tf2_ros','rosbag2_transport','cv_bridge','message_filters'):
        get_package_prefix(pkg)
    model=ROOT/'models/yolov8n-seg.pt'
    if hashlib.sha256(model.read_bytes()).hexdigest()!=MODEL_SHA:raise RuntimeError('Original YOLO model SHA256 differs')
    python=os.environ.get('SEMANTIC_PYTHON',str(ROOT/'.venv/bin/python'))
    code="""import torch,ultralytics,rclpy,cv2,json,sys
from torchvision.ops import nms
device=sys.argv[1]
if device!='cpu' and not torch.cuda.is_available():raise RuntimeError('Requested CUDA device is unavailable')
target='cpu' if device=='cpu' else ('cuda:'+device if device.isdigit() else device)
boxes=torch.tensor([[0.,0.,10.,10.],[1.,1.,9.,9.]],device=target)
scores=torch.tensor([.9,.8],device=target)
assert nms(boxes,scores,.5).numel()==1
print(json.dumps({'cuda':torch.cuda.is_available(),'torch':torch.__version__,'ultralytics':ultralytics.__version__,'torchvision_nms_device':target,'torchvision_nms_passed':True}))
"""
    result=subprocess.run([python,'-c',code,a.device],capture_output=True,text=True,timeout=45)
    if result.returncode:
        raise RuntimeError('Python/GPU/NMS check failed: '+result.stderr.strip()[-2000:])
    info=json.loads(result.stdout.strip().splitlines()[-1])
    if a.device!='cpu' and not info['cuda']:raise RuntimeError('GPU requested but CUDA is unavailable; no silent CPU fallback')
    return dict(model_sha256=MODEL_SHA,python=python,device=a.device,**info)

def main():
    a=arguments();c=load_config(a.config);replay=bool(a.replay);logs=ROOT/'logs';logs.mkdir(exist_ok=True)
    active=logs/'active_live.json'
    if a.status:
        if not active.exists():print('没有本组正在运行的托管任务。');return 0
        record=json.loads(active.read_text());proc=Path('/proc')/str(record['pid'])
        try:
            assert (proc/'stat').read_text().split()[21]==record['start_ticks']
            assert str(ROOT/'scripts/run_live.py').encode() in (proc/'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError,AssertionError):print('记录对应的任务已退出。');return 0
        status=Path(record['output'])/'session.json';print(status.read_text() if status.exists() else json.dumps(record));return 0
    domain=int(os.environ.get('ROS_DOMAIN_ID','77')) if replay else 0
    if replay and domain==0:domain=77
    if not 0<=domain<=232:raise ValueError('Invalid ROS Domain')
    duration=a.duration if a.duration is not None else (0 if replay else c['runtime']['duration_seconds'])
    plan=dict(backend='RTAB-Map + original YOLOv8n-seg',mode='replay' if replay else 'rgbd_handpush',
        odometry_source='rtabmap_odom/rgbd_odometry',geometry_executable='rtabmap_slam/rtabmap',
        domain=domain,device=a.device,namespace=c['namespace'],tf_topics=tf_topics(c),inputs=c['topics'],
        replay_topics=input_topics(c),record_topics=record_topics(c),duration_after_ready=duration,
        controls_requested=False,controls_published=False,wheel_odometry_used=False,lidar_used=False,
        old_global_dynamic_tf_consumed=False)
    if a.dry_run:print(json.dumps(plan,ensure_ascii=False,indent=2));return 0
    lock=(logs/'live.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise RuntimeError('本组任务已运行，不能重复启动；用 stop.sh 保存退出。')
    software=check_software(a,c)
    if a.check:print(json.dumps({'software_check':'passed','hardware_check':'not_run',**software,**plan},ensure_ascii=False,indent=2));return 0
    import rclpy
    from mapping_runtime import Runtime,atomic_json
    from ament_index_python.packages import get_package_prefix,PackageNotFoundError
    stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    out=(a.output or ROOT/'runs'/(('replay-' if replay else 'handpush-')+stamp)).expanduser().resolve()
    out.mkdir(parents=True,exist_ok=False)
    if shutil.disk_usage(out).free<c['recording']['minimum_free_gib']*1024**3:raise RuntimeError('Insufficient free disk space')
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1' if replay else '0',
        ROS_HOME=str(out/'ros_home'),ROS_LOG_DIR=str(out/'ros_logs'),PYTHONUNBUFFERED='1',
        RCUTILS_COLORIZED_OUTPUT='0',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1')
    for key in ('ROS_HOME','ROS_LOG_DIR'):Path(env[key]).mkdir(exist_ok=True)
    os.environ.update({k:env[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_HOME','ROS_LOG_DIR')})
    rclpy.init(args=[]);node=None;children={};handles=[];events=[];resources=[];resource_handles={}
    state=dict(**plan,output=str(out),software=software,phase='starting',ready=False,ever_ready=False,recording=False)
    stop_requested=False;reason='';ready_wall=None;bad_since=None;replay_finished=None
    atomic_json(active,dict(pid=os.getpid(),start_ticks=Path('/proc/self/stat').read_text().split()[21],script=str(Path(__file__).resolve()),output=str(out)))
    atomic_json(out/'configuration.json',dict(plan=plan,config=c,software=software,
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ['scripts','semantic','launch','config'] for p in (ROOT/folder).iterdir()
            if p.is_file() and p.suffix in ('.py','.sh','.yaml','.rviz')}))
    def event(kind,**values):
        entry=dict(event=kind,wall_time=time.time(),**values);events.append(entry)
        atomic_json(out/'events.json',events);print(json.dumps(entry,ensure_ascii=False),flush=True)
    def halt(signum,frame):
        nonlocal stop_requested,reason
        stop_requested=True;reason='user_stop'
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,halt)
    def spawn(name,cmd,required=True):
        f=(out/(name+'.log')).open('w');handles.append(f)
        p=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        children[name]=(p,required);event('started',name=name,pid=p.pid,argv=cmd)
    def stop_child(name):
        if name not in children:return
        p=children[name][0]
        if p.poll() is not None:return
        try:p.send_signal(signal.SIGINT)
        except ProcessLookupError:p.wait();return
        until=time.monotonic()+(40 if name=='pipeline' else 20)
        while p.poll() is None and time.monotonic()<until:
            if node:rclpy.spin_once(node,timeout_sec=.02)
            else:time.sleep(.02)
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
        event('stopped',name=name,returncode=p.returncode)
    def record_has_data():
        # Never SELECT from an actively written SQLite bag.
        for p in (out/'bag').glob('*.db3'):
            try:
                if p.stat().st_size>131072:
                    with p.open('rb') as f:
                        if f.read(16)==b'SQLite format 3\x00':return True
            except FileNotFoundError:pass
        for p in (out/'bag').glob('*.zstd'):
            try:
                if p.stat().st_size>0:return True
            except FileNotFoundError:pass
        return False
    def sample_resources():
        rows=[]
        for process in [psutil.Process()]+psutil.Process().children(recursive=True):
            try:
                key=(process.pid,process.create_time())
                if key not in resource_handles:resource_handles[key]=process;process.cpu_percent()
                p=resource_handles[key];rows.append(dict(pid=p.pid,name=p.name(),cpu_percent=p.cpu_percent(),rss_mib=p.memory_info().rss/1048576))
            except psutil.Error:pass
        resources.append(dict(wall_time=time.time(),system_cpu_percent=psutil.cpu_percent(),processes=rows,
            process_cpu_sum_percent=sum(x['cpu_percent'] for x in rows),process_rss_sum_mib=sum(x['rss_mib'] for x in rows),free_disk_bytes=shutil.disk_usage(out).free))
        if len(resources)%5==0:atomic_json(out/'resources.json',resources)
    try:
        node=Runtime(c,out,sim=replay,video=c['video']['enabled'] and not a.no_video)
        until=time.monotonic()+2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
        existing=[ns.rstrip('/')+'/'+name for name,ns in node.get_node_names_and_namespaces()
            if name in ('rtabmap','rgbd_odometry','icp_odometry','yolo_detector','semantic_projector','semantic_grid_node')]
        if existing:raise RuntimeError('Do not mix existing mapping/odometry nodes: '+', '.join(existing))
        if node.count_publishers(c['topics']['odom']):raise RuntimeError('The private visual odometry topic already has a publisher')
        if not replay:
            pubs=[node.count_publishers(c['topics'][k]) for k in ('rgb','depth','camera_info')]
            if any(pubs):
                if not all(pubs):raise RuntimeError('Camera is partially running; inspect its existing producer')
                event('reuse_data_source',source='camera')
            elif c['sources']['camera']=='auto':spawn('camera',['bash',str(ROOT/'scripts/run_camera.sh')])
            else:raise RuntimeError('No RGB-D camera publisher')
        if c['recording']['enabled'] and not a.no_record:
            qos={c['topics'][k]:dict(reliability='best_effort',durability='volatile',history='keep_last',depth=10)
                 for k in ('rgb','depth','camera_info','odom')}
            for topic in ('/tf_static',tf_topics(c)[1]):qos[topic]=dict(reliability='reliable',durability='transient_local',history='keep_last',depth=1)
            q=out/'recording_qos.yaml';q.write_text(yaml.safe_dump(qos))
            cmd=['ros2','bag','record','-s','sqlite3','-o',str(out/'bag'),'--max-bag-size',str(c['recording']['split_bytes']),
                 '--max-bag-duration',str(c['recording']['split_seconds']),'--qos-profile-overrides-path',str(q)]
            try:get_package_prefix('rosbag2_compression_zstd');available=True
            except PackageNotFoundError:available=False
            compression=c['recording']['compression']
            if compression=='zstd' and not available:raise RuntimeError('Missing rosbag2_compression_zstd')
            if available and compression in ('auto','zstd'):
                cmd+=['--compression-mode','file','--compression-format','zstd','--compression-threads','1'];state['bag_compression']='zstd'
            else:state['bag_compression']='none';event('raw_recording_fallback')
            if replay:cmd+=['--use-sim-time']
            spawn('recording',cmd+record_topics(c))
        else:state['bag_compression']='not_recorded'
        if shutil.which('tegrastats'):spawn('tegrastats',['tegrastats','--interval','1000'],False)
        else:event('tegrastats_unavailable',gpu_load='not_measured')
        spawn('pipeline',['ros2','launch',str(ROOT/'launch/live_semantic.launch.py'),
            'config:='+str(a.config.resolve()),'output_dir:='+str(out),'use_sim_time:='+str(replay).lower(),'device:='+a.device,'rviz:='+str(a.rviz).lower()])
        if replay:spawn('player',['ros2','bag','play',str(a.replay.resolve()),'--clock','--rate',str(a.rate),'--delay','3','--topics',*input_topics(c)])
        begin=time.monotonic();last_status=last_notice=last_resources=0
        while not stop_requested:
            for _ in range(8):rclpy.spin_once(node,timeout_sec=.01)
            node.tick();now=time.monotonic()
            for name,(p,required) in children.items():
                code=p.poll()
                if code is None:continue
                if name=='player' and code==0:
                    if replay_finished is None:replay_finished=now
                elif required:raise RuntimeError(name+' exited: '+str(code)+'; inspect '+name+'.log')
            if node.fatal:raise RuntimeError(node.fatal)
            if shutil.disk_usage(out).free<c['recording']['stop_free_gib']*1024**3:raise RuntimeError('Free disk limit reached; saving')
            if now-last_status>=1:
                observed=node.snapshot();missing=node.readiness(initial=not state['ever_ready'])
                if 'recording' in children:
                    state['recording']=record_has_data()
                    if not state['recording']:missing.append('recording has no confirmed messages yet')
                if not missing:
                    bad_since=None
                    if not state['ready']:
                        state.update(ready=True,ever_ready=True,phase='ready',ready_wall_time=time.time())
                        if ready_wall is None:ready_wall=now
                        node.initial_verified=True
                        event('READY_REPLAY_VALIDATED' if replay else 'READY_FOR_HANDPUSH',recording=state['recording'])
                        print(('本机回放链路已就绪；不代表实车通过。' if replay else 'RGB-D RTAB-Map＋YOLO已就绪，可以缓慢手推；未启动底盘控制。')+' 输出：'+str(out),flush=True)
                elif state['ever_ready']:
                    if bad_since is None:bad_since=now;event('PAUSE_HANDPUSH',missing=missing)
                    state.update(ready=False,phase='paused')
                    if now-bad_since>c['runtime']['failure_timeout']:raise RuntimeError('Required RGB-D/visual-odometry/TF output stopped: '+', '.join(missing))
                elif now-begin>c['runtime']['readiness_timeout']:raise TimeoutError('Not ready: '+', '.join(missing))
                if now-last_notice>=5:event('status',phase=state['phase'],missing=missing,graph_poses=observed['graph_poses'],geometry_points=observed['geometry_points'],semantic_voxels=node.grid.get('voxels',0));last_notice=now
                state['latest']=observed;atomic_json(out/'session.json',state);last_status=now
            if now-last_resources>=1:sample_resources();last_resources=now
            if ready_wall and duration and now-ready_wall>=duration:reason='duration_finished';break
            if replay_finished and now-replay_finished>3:
                if not state['ready']:raise RuntimeError('Replay ended before readiness')
                reason='replay_finished';break
        state['phase']=reason or 'user_stop'
    except Exception as e:
        if stop_requested:state.update(phase='user_stop');event('user_stop')
        else:state.update(phase='failed',error=repr(e));event('failure',error=repr(e))
    finally:
        state.update(ready=False,controls_published=False)
        if node and 'pipeline' in children:
            try:
                if node.save_client.wait_for_service(timeout_sec=2):
                    from std_srvs.srv import Trigger
                    future=node.save_client.call_async(Trigger.Request());until=time.monotonic()+5
                    while not future.done() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
                    state['semantic_save_service']=dict(success=future.result().success,message=future.result().message) if future.done() else dict(success=False,error='timeout')
            except Exception as e:state['semantic_save_service']=dict(success=False,error=repr(e))
        for name in ('player','recording','pipeline','camera','tegrastats'):stop_child(name)
        if node:
            try:state['final_observation']=node.snapshot();node.close_artifacts()
            except Exception as e:state['artifact_save_error']=repr(e)
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for f in handles:f.close()
        atomic_json(out/'resources.json',resources);state['recording']=False
        state['exit_codes']={name:p.returncode for name,(p,required) in children.items()}
        db_path=out/'rtabmap.db'
        if db_path.exists():
            try:
                with sqlite3.connect('file:'+str(db_path)+'?mode=ro',uri=True) as db:state['database']=dict(quick_check=db.execute('pragma quick_check').fetchone()[0],nodes=db.execute('select count(*) from Node').fetchone()[0])
            except sqlite3.Error as e:state['database']=dict(error=str(e))
        state['finished_wall_time']=time.time();atomic_json(out/'session.json',state);atomic_json(out/'result.json',state)
        if active.exists() and json.loads(active.read_text()).get('pid')==os.getpid():active.unlink()
        event('saved',output=str(out),phase=state['phase']);lock.close()
    return 1 if state['phase']=='failed' else 0

if __name__=='__main__':
    try:sys.exit(main())
    except Exception as e:print('STARTUP FAILED: '+str(e),file=sys.stderr);sys.exit(2)
