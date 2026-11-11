"""Managed joystick input and the existing safe velocity chain; no direct MCU output."""
import copy
import math
import re
import subprocess
import time
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

from mapping_config import ROOT


class GamepadGuard:
    def __init__(self, node, config, args, output, spawn, stop_child, pump, event):
        self.node=node;self.c=config['gamepad'];self.ns=config['namespace'];self.output=output
        self.spawn=spawn;self.stop_child=stop_child;self.pump=pump;self.event=event
        path=Path(self.c['config']);path=path if path.is_absolute() else ROOT/path
        source=yaml.safe_load(path.read_text())
        self.joy_parameters=copy.deepcopy(source['joy_node']['ros__parameters'])
        self.teleop_parameters=copy.deepcopy(source['teleop_twist_joy_node']['ros__parameters'])
        if args.linear is not None:self.teleop_parameters['scale_linear']['x']=args.linear
        if args.angular is not None:self.teleop_parameters['scale_angular']['yaw']=args.angular
        self.joy=None;self.joy_at=0.;self.raw_at=0.;self.enabled=False;self.arm_wait=None
        self.chain={};self.last_chain_check=0.
        self.not_before=0.
        self.subs=[node.create_subscription(Joy,'/joy',self.receive_joy,qos_profile_sensor_data),
                   node.create_subscription(Twist,self.c['command_topic'],self.receive_raw,qos_profile_sensor_data)]
        self.config_path=output/'gamepad.yaml'
        self.config_path.write_text(yaml.safe_dump({
            self.ns+'/joy_node':{'ros__parameters':self.joy_parameters},
            self.ns+'/teleop_twist_joy_node':{'ros__parameters':self.teleop_parameters}},sort_keys=False))

    def receive_joy(self, msg):
        self.joy=msg;self.joy_at=time.monotonic()

    def receive_raw(self, msg):
        self.raw_at=time.monotonic()

    def neutral(self):
        j=self.joy
        return (j is not None and time.monotonic()-self.joy_at<self.c['input_timeout']
                and len(j.buttons)>0 and len(j.axes)>1 and j.buttons[0]==0
                and all(math.isfinite(v) and abs(v)<=.12 for v in j.axes[:2]))

    def executable(self, package, name):
        path=Path(get_package_prefix(package))/'lib'/package/name
        if not path.is_file():raise RuntimeError('Missing control executable: '+str(path))
        return str(path)

    def call(self, service_type, name, request):
        client=self.node.create_client(service_type,name)
        try:
            if not client.wait_for_service(timeout_sec=2):raise RuntimeError('Control-chain service unavailable: '+name)
            future=client.call_async(request);until=time.monotonic()+2
            while not future.done() and time.monotonic()<until:self.pump(.03)
            if not future.done():raise RuntimeError('Control-chain service timeout: '+name)
            return future.result()
        finally:self.node.destroy_client(client)

    def check_chain(self):
        # This vehicle already has smoothing, collision monitoring and an adapter.
        # Publishing a second /cmd_vel would make the adapter refuse to drive.
        if self.c['command_topic']!='/cmd_vel_raw':raise RuntimeError('Use the verified upstream /cmd_vel_raw control input')
        endpoints=self.node.get_publishers_info_by_topic('/cmd_vel')
        if len(endpoints)!=1 or endpoints[0].node_name!='field_velocity_adapter':
            raise RuntimeError('Expected exactly one field_velocity_adapter publisher on /cmd_vel; shared processes were not changed')
        safe=self.node.get_publishers_info_by_topic('/cmd_vel_safe')
        if len(safe)!=1 or safe[0].node_name!='collision_monitor':raise RuntimeError('Collision-monitor output is absent or has competing publishers')
        raw=self.node.get_publishers_info_by_topic('/cmd_vel_raw')
        allowed={'controller_server'} | ({'teleop_twist_joy_node'} if self.enabled else set())
        if any(p.node_name not in allowed or (p.node_name=='teleop_twist_joy_node' and p.node_namespace!=self.ns) for p in raw):
            raise RuntimeError('Unknown or independent control publisher on /cmd_vel_raw')
        subscribers=self.node.get_subscriptions_info_by_topic('/cmd_vel_raw')
        if not any(p.node_name=='velocity_smoother' for p in subscribers):raise RuntimeError('Velocity smoother is not subscribed to /cmd_vel_raw')
        for name in ('velocity_smoother','collision_monitor'):
            response=self.call(GetState,'/'+name+'/get_state',GetState.Request())
            if response.current_state.id!=3:raise RuntimeError(name+' is not active; this entry does not change shared lifecycle state')
        endpoint=endpoints[0];adapter=endpoint.node_namespace.rstrip('/')+'/'+endpoint.node_name
        names=['max_linear','max_angular','cmd_vel_topic','hardware_output_topic','zero_only']
        response=self.call(GetParameters,adapter+'/get_parameters',GetParameters.Request(names=names))
        values=dict(zip(names,map(parameter_value_to_python,response.values)))
        if values['cmd_vel_topic']!='/cmd_vel_safe' or values['hardware_output_topic']!='/cmd_vel' or values['zero_only'] is not False:
            raise RuntimeError('Shared adapter configuration is unsuitable for driving: '+str(values))
        if not all(isinstance(values[k],(float,int)) and math.isfinite(values[k]) and values[k]>0 for k in ('max_linear','max_angular')):
            raise RuntimeError('Shared adapter speed limits could not be confirmed')
        self.chain=values;self.last_chain_check=time.monotonic()
        return values

    def prepare(self):
        known=[name for name,ns in self.node.get_node_names_and_namespaces()]
        if self.node.count_publishers('/joy') or any(n in known for n in ('joy_node','teleop_twist_joy_node')):
            raise RuntimeError('An independent gamepad session exists; no duplicate was started and no other process was stopped')
        if subprocess.run(['systemctl','is-active','--quiet','jetson-ros.service']).returncode:
            raise RuntimeError('Shared chassis communication service is not active; it was not restarted')
        self.check_chain()
        result=subprocess.run([self.executable('joy','joy_enumerate_devices')],capture_output=True,text=True,timeout=8)
        (self.output/'gamepad_devices.log').write_text(result.stdout+result.stderr)
        device_ids={int(m.group(1)) for line in result.stdout.splitlines() if (m:=re.match(r'^\s*(\d+)\s*:',line))}
        if result.returncode or self.joy_parameters.get('device_id',0) not in device_ids:
            raise RuntimeError('No configured SDL gamepad input device. Pair/connect the controller and confirm HID input first; Bluetooth connected alone is insufficient. See gamepad_devices.log')
        self.spawn('joy',[self.executable('joy','joy_node'),'--ros-args','--params-file',str(self.config_path),
                         '-r','__ns:='+self.ns,'-r','__node:=joy_node','-r','joy:=/joy','-r','/joy:=/joy'])
        self.event('waiting_for_gamepad_input',message='Release A and centre the left stick; waiting for actual /joy messages')
        until=time.monotonic()+self.c['neutral_timeout'];began=time.monotonic()
        while time.monotonic()<until:
            self.pump(.03)
            if self.neutral() and time.monotonic()-began>=2:
                if self.raw_at and time.monotonic()-self.raw_at<2:
                    raise RuntimeError('Another controller is actively publishing /cmd_vel_raw; no teleop was started')
                self.event('gamepad_input_verified',shared_limits=self.chain,
                           requested_linear=self.teleop_parameters['scale_linear']['x'],
                           requested_angular=self.teleop_parameters['scale_angular']['yaw'])
                return
        raise RuntimeError('No fresh neutral /joy input: release A, centre the stick and check the OS input driver')

    def health(self):
        missing=[]
        if not self.joy_at or time.monotonic()-self.joy_at>=self.c['input_timeout']:missing.append('fresh /joy input')
        if not self.enabled and not self.neutral():missing.append('release A and centre the left stick')
        if time.monotonic()-self.last_chain_check>5:self.check_chain()
        if self.enabled:
            pubs=self.node.get_publishers_info_by_topic(self.c['command_topic'])
            if not any(p.node_name=='teleop_twist_joy_node' and p.node_namespace==self.ns for p in pubs):missing.append('managed teleop publisher discovery')
            subs=self.node.get_subscriptions_info_by_topic('/joy')
            if not any(p.node_name=='teleop_twist_joy_node' and p.node_namespace==self.ns for p in subs):missing.append('managed teleop /joy subscription')
        return missing

    def activate(self):
        if self.enabled:return
        if time.monotonic()<self.not_before:return
        if self.arm_wait is None:self.arm_wait=time.monotonic()
        if not self.neutral():
            if time.monotonic()-self.arm_wait>self.c['neutral_timeout']:raise RuntimeError('Release A and centre the stick before enabling the verified mapping session')
            return
        if self.raw_at and time.monotonic()-self.raw_at<2:raise RuntimeError('Another command stream is active on /cmd_vel_raw; teleop was not enabled')
        self.check_chain()
        self.spawn('teleop',[self.executable('teleop_twist_joy','teleop_node'),'--ros-args','--params-file',str(self.config_path),
                            '-r','__ns:='+self.ns,'-r','__node:=teleop_twist_joy_node',
                            '-r','joy:=/joy','-r','/joy:=/joy',
                            '-r','cmd_vel:='+self.c['command_topic'],'-r','/cmd_vel:='+self.c['command_topic']])
        self.enabled=True;self.arm_wait=None
        self.event('managed_teleop_started',command_topic=self.c['command_topic'],wheel_execution_verified=False)

    def deactivate(self):
        if self.enabled:
            self.enabled=False;self.stop_child('teleop');self.arm_wait=None
            self.not_before=time.monotonic()+2
            self.event('managed_teleop_disabled',message='Shared adapter timeout remains responsible for zero output; shared processes are untouched')

    def snapshot(self):
        return dict(teleop_enabled=self.enabled,joy_received=self.joy is not None,
                    joy_age_seconds=time.monotonic()-self.joy_at if self.joy_at else None,
                    neutral=self.neutral(),shared_limits=self.chain,
                    requested_linear=self.teleop_parameters['scale_linear']['x'],
                    requested_angular=self.teleop_parameters['scale_angular']['yaw'],
                    wheel_execution_verified=False)
