"""Configuration for the RGB-D hand-pushed RTAB-Map + YOLO baseline."""
from pathlib import Path
import math
import yaml
ROOT = Path(__file__).resolve().parents[1]
CAMERA_TOPICS = ('rgb', 'depth', 'camera_info')

def load_config(path=None):
    c=yaml.safe_load(Path(path or ROOT/'config/robot_mapping.yaml').read_text())
    if c.get('backend')!='rtabmap' or c.get('odometry',{}).get('source')!='rgbd':
        raise ValueError('This baseline requires RTAB-Map RGB-D visual odometry')
    ns=c['namespace']
    if not ns.startswith('/') or ns=='/' or ns.endswith('/'):raise ValueError('Use a dedicated absolute namespace')
    for key in CAMERA_TOPICS:
        if not c['topics'][key].startswith('/'):raise ValueError('Camera topics must be absolute')
    if c['topics']['odom']!=ns+'/odom':raise ValueError('Use this project visual odometry topic only')
    if any(key in c['topics'] for key in ('encoder',)):raise ValueError('Wheel input is not enabled in the RGB-D baseline')
    f=c['frames']
    if f['base']!=f['camera'] or f['camera'] in (f['map'],f['odom']) or f['map']==f['odom']:
        raise ValueError('Use the camera body as reference, with separate map and odometry frames')
    if c.get('mounts'):raise ValueError('Use camera-internal calibration; no unverified robot/laser mounts')
    if c.get('gamepad',{}).get('enabled'):raise ValueError('The hand-pushed baseline never starts chassis control')
    if c['sources'].get('camera') not in ('auto','existing'):raise ValueError('Invalid camera source mode')
    if not 0<float(c['sync_seconds'])<=.04:raise ValueError('RGB-D matching window must be positive and at most 40 ms')
    if not 0<float(c['video']['fps'])<=30:raise ValueError('Invalid video FPS')
    for k in ('readiness_timeout','stream_timeout','failure_timeout'):
        if not math.isfinite(c['runtime'][k]) or c['runtime'][k]<=0:raise ValueError('Invalid runtime timeout')
    return c

def input_topics(c):
    # Replay raw camera observations only; NEVER old odometry, TF motion or controls.
    return [c['topics'][k] for k in CAMERA_TOPICS]+([c['topics']['scan']] if 'scan' in c['topics'] else [])+['/tf_static']

def tf_topics(c):return c['namespace']+'/tf',c['namespace']+'/tf_static'

def record_topics(c):
    ns=c['namespace']
    return list(dict.fromkeys(input_topics(c)+[c['topics']['odom']]+list(tf_topics(c))+
        [ns+'/info',ns+'/mapGraph',ns+'/odom_info']+
        ['/semantic/'+k for k in ('label_image','instance_image','detector_status','points_camera','pointcloud','map_status')]))

def visual_odometry_parameters(c,use_sim_time=False):
    p=dict(c.get('visual_odometry',{}))
    p.update(frame_id=c['frames']['base'],odom_frame_id=c['frames']['odom'],publish_tf=True,
        use_sim_time=use_sim_time,subscribe_rgbd=False,approx_sync=True,
        approx_sync_max_interval=c['sync_seconds'],qos=2,qos_camera_info=2,
        topic_queue_size=30,sync_queue_size=40,wait_for_transform=.2)
    return p

def rtabmap_parameters(c,output,use_sim_time=False):
    p=dict(c['rtabmap'])
    p.update(frame_id=c['frames']['base'],map_frame_id=c['frames']['map'],odom_frame_id='',
        subscribe_odom=True,subscribe_depth=True,subscribe_scan=False,subscribe_scan_cloud=False,
        subscribe_rgbd=False,subscribe_odom_info=False,publish_tf=True,use_sim_time=use_sim_time,
        database_path=str(Path(output)/'rtabmap.db'),approx_sync=True,
        approx_sync_max_interval=c['sync_seconds'],qos_image=2,qos_camera_info=2,
        qos_odom=2,topic_queue_size=30,sync_queue_size=40)
    p['Reg/Strategy']='0'
    return p
