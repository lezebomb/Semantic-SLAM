#!/usr/bin/env python3
"""Bounded camera/LiDAR/GPU check. No chassis, arm, motion publisher or invented TF."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request

import psutil

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)
env = os.environ.copy()
env.update(ROS_DOMAIN_ID='77', ROS_LOCALHOST_ONLY='1', OMP_NUM_THREADS='2',
           OPENBLAS_NUM_THREADS='1', ROS_HOME=str(out/'ros_home'),
           ROS_LOG_DIR=str(out/'ros_logs'))
for name in ['ros_home', 'ros_logs']:
    (out/name).mkdir(exist_ok=True)
processes, handles, samples, errors = {}, [], [], []
result = dict(output=str(out), motion_commands_sent=0, domain_id=77)


def start(name, command, seconds=60):
    log = (out/(name+'.log')).open('w')
    handles.append(log)
    p = subprocess.Popen(['timeout', '--signal=INT', '--kill-after=5s', str(seconds)] + command,
                         stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    processes[name] = p
    return p


def stop(p):
    if p.poll() is not None:
        return
    os.killpg(p.pid, signal.SIGINT)
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGTERM)
        p.wait(timeout=4)


def interrupted(signum, frame):
    raise KeyboardInterrupt


for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]:
    signal.signal(sig, interrupted)
monitors = {}
try:
    device = json.loads((out/'lidar_device_before.json').read_text())
    assert device['/api/v1/system/firmware']['model'] == 'LakiBeam1'
    assert device['/api/v1/sensor/overview']['host'] == {'ip': '192.168.10.102', 'port': 2368}
    start('camera_driver', ['bash', str(ROOT/'scripts/run_camera.sh')], 65)
    start('lidar_driver', ['bash', str(ROOT/'scripts/run_lidar.sh')], 65)
    start('tegrastats', ['tegrastats', '--interval', '1000'], 65)
    time.sleep(5)
    for name in ['camera_driver', 'lidar_driver']:
        assert processes[name].poll() is None, name+' exited; inspect its log'
    start('interfaces', [sys.executable, str(ROOT/'scripts/inspect_live_interfaces.py'),
                        '--seconds', '10', '--output', str(out/'interfaces.json')], 20)
    start('scan', [sys.executable, str(ROOT/'scripts/validate_scan.py'),
                  '--seconds', '10', '--output', str(out/'scan')], 20)
    start('gpu_capture', [sys.executable, str(ROOT/'scripts/capture_d435_frame.py'),
                         '--output', str(out/'camera')], 45)
    deadline = time.monotonic()+47
    while any(processes[k].poll() is None for k in ['interfaces', 'scan', 'gpu_capture']):
        if time.monotonic() > deadline:
            raise TimeoutError('Static check deadline exceeded')
        items = []
        for p in psutil.Process().children(recursive=True):
            try:
                key = (p.pid, p.create_time())
                if key not in monitors:
                    monitors[key] = p
                    p.cpu_percent()
                m = monitors[key]
                items.append(dict(pid=p.pid, name=p.name(), cpu_percent=m.cpu_percent(),
                                  rss_mib=m.memory_info().rss/1048576))
            except psutil.Error:
                pass
        samples.append(dict(wall_time=time.time(), processes=items))
        time.sleep(.5)
    for k in ['interfaces', 'scan', 'gpu_capture']:
        if processes[k].returncode != 0:
            errors.append(k+' exited '+str(processes[k].returncode))
except (Exception, KeyboardInterrupt) as e:
    errors.append(type(e).__name__+': '+str(e))
finally:
    for p in reversed(list(processes.values())):
        stop(p)
    for f in handles:
        f.close()
    result['exit_codes'] = {k:p.returncode for k,p in processes.items()}
    result['errors'] = errors
    for key, name in [('camera', 'camera/result.json'), ('scan', 'scan/result.json'),
                      ('interfaces', 'interfaces.json')]:
        if (out/name).is_file():
            result[key] = json.loads((out/name).read_text())
    op = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with op.open('http://192.168.10.160/api/v1/sensor/overview', timeout=3) as f:
            after = json.load(f)
        before = device['/api/v1/sensor/overview']
        result['lidar_settings_unchanged'] = all(before[k]==after[k] for k in
                ['scanfreq', 'laser_enable', 'resolution', 'scan_range', 'filter', 'host'])
        (out/'lidar_device_after.json').write_text(json.dumps(after, indent=2))
    except Exception as e:
        result['lidar_settings_error'] = str(e)
    result['driver_error_lines'] = {name: [l for l in (out/(name+'.log')).read_text().splitlines()
        if any(x in l for x in ['[ERROR]', '[FATAL]', 'Traceback', 'terminate called'])]
        for name in ['camera_driver', 'lidar_driver'] if (out/(name+'.log')).exists()}
    result['success'] = bool(not errors and result.get('camera', {}).get('device')=='cuda:0'
        and result.get('scan', {}).get('success') and result.get('lidar_settings_unchanged')
        and not any(result['driver_error_lines'].values()))
    (out/'resources.json').write_text(json.dumps(samples, indent=2))
    (out/'static_result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='interfaces'}, indent=2))
raise SystemExit(0 if result['success'] else 1)
