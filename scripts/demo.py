#!/usr/bin/env python3
"""Run a repeatable control demonstration against the running simulation."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool, UInt8
from ros_gz_interfaces.msg import Contacts
from model_geometry import Model
import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Demo(Node):
    def __init__(self, namespace):
        super().__init__('sentry_demo', parameter_overrides=[Parameter('use_sim_time', value=True)])
        prefix = '/' + namespace.strip('/') + '/'
        self.velocity = self.create_publisher(Twist, prefix + 'cmd_vel', 1)
        self.gimbal = self.create_publisher(Vector3, prefix + 'gimbal_cmd', 1)
        self.height_command = self.create_publisher(Float64, prefix + 'height_cmd', 1)
        self.lift_command = self.create_publisher(Float64, prefix + 'lift_cmd', 1)
        self.mode = self.create_publisher(Bool, prefix + 'gimbal_world_mode', 1)
        self.joints = {}; self.odom = None; self.imu_count = 0; self.gimbal_imu_count = 0
        self.cloud_count=0; self.cloud=None
        self.contacts = 0; self.limits = None; self.trace = []; self.results = []
        self.model=Model();self.self_hits=set();self.sampled_max_height=0.;self.pass_max_height=0.
        self.create_subscription(PointCloud2, prefix + 'odin1/cloud_raw', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(JointState, prefix + 'joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Odometry, prefix + 'ground_truth/odom', lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)
        self.create_subscription(Imu, prefix + 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Imu, prefix + 'gimbal_imu', self.on_gimbal_imu, qos_profile_sensor_data)
        self.create_subscription(Contacts, prefix + 'tunnel_contacts', self.on_contacts, qos_profile_sensor_data)
        self.create_subscription(UInt8, prefix + 'height_limits', lambda m: setattr(self, 'limits', m.data), 1)

    def on_cloud(self, msg):
        self.cloud_count+=1; self.cloud=msg

    def cloud_points(self):
        points=point_cloud2.read_points_numpy(self.cloud,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
        return points[np.isfinite(points).all(axis=1)]

    def on_joints(self, msg):
        self.joints = {n: (msg.position[i], msg.velocity[i]) for i, n in enumerate(msg.name)}

    def on_imu(self, _): self.imu_count += 1
    def on_gimbal_imu(self, _): self.gimbal_imu_count += 1
    def on_contacts(self, msg):
        self.contacts += len(msg.contacts)

    def sim_time(self): return self.get_clock().now().nanoseconds * 1e-9
    def q(self, name): return self.joints[name + '_joint'][0]
    def height(self): return self.geometry_height()
    def barrel_pitch(self): return self.q('gimbal_pitch')
    def base_transform(self):
        pose=self.odom.pose.pose;q=pose.orientation;x,y,z,w=q.x,q.y,q.z,q.w
        base=np.eye(4)
        base[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
        base[:3,3]=[pose.position.x,pose.position.y,pose.position.z]
        return base
    def geometry_height(self):
        return self.model.bounds({n:v[0] for n,v in self.joints.items()},self.base_transform())[1][2]
    def pose(self):
        p = self.odom.pose.pose
        q = p.orientation
        return p.position.x, p.position.y, math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def send(self, vx=0., vy=0., wz=0., height=None, lift=None, yaw=0., pitch=0., world=False):
        msg = Twist(); msg.linear.x = float(vx); msg.linear.y = float(vy); msg.angular.z = float(wz)
        self.velocity.publish(msg)
        self.gimbal.publish(Vector3(x=float(yaw), y=float(pitch)))
        if height is not None:self.height_command.publish(Float64(data=float(height)))
        if lift is not None:self.lift_command.publish(Float64(data=float(lift)))
        self.mode.publish(Bool(data=world))

    def wait_ready(self):
        end = time.monotonic() + 70
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)
            if len(self.joints) == 12 and self.odom and self.imu_count and self.limits is not None and self.cloud_count>0:
                return
        raise RuntimeError('Simulation feedback startup exceeded 70 seconds.')

    def run_for(self, duration, command=None):
        start = self.sim_time(); end = time.monotonic() + max(20, duration * 8)
        last_send = -1.; last_trace = -1.
        while self.sim_time() - start < duration:
            if time.monotonic() > end: raise RuntimeError('Simulation time advance exceeded the allowed duration.')
            rclpy.spin_once(self, timeout_sec=.01)
            now = self.sim_time()
            if command and now - last_send >= .02:
                command(); last_send = now
            if self.odom and now - last_trace >= .05:
                x, y, a = self.pose()
                overall=self.geometry_height()
                self.sampled_max_height=max(self.sampled_max_height,overall)
                self.self_hits.update(self.model.intersections({n:v[0] for n,v in self.joints.items()}))
                if 1.2<x<2.8:self.pass_max_height=max(self.pass_max_height,overall)
                self.trace.append([now,x,y,a,self.q('gimbal_yaw'),self.barrel_pitch(),self.q('lift'),self.q('lift_stage'),overall,self.contacts])
                last_trace = now

    def check(self, name, condition, **measurements):
        item = {'name': name, 'passed': bool(condition), **measurements}
        self.results.append(item); print(json.dumps(item, ensure_ascii=False), flush=True)
        if not condition: raise RuntimeError(f'Test failed: {name}')

    def go(self, x, y, yaw=0., duration=15.):
        def command():
            px, py, a = self.pose(); dx=x-px; dy=y-py
            vx = .9 * (math.cos(a)*dx + math.sin(a)*dy)
            vy = .9 * (-math.sin(a)*dx + math.cos(a)*dy)
            scale = min(1., .35 / max(.001, math.hypot(vx, vy)))
            self.send(vx*scale, vy*scale, max(-.6, min(.6, 1.2*wrap(yaw-a))))
        self.run_for(duration, command)
        px, py, a = self.pose()
        self.check('waypoint', math.hypot(x-px,y-py)<.10 and abs(wrap(yaw-a))<.12,
                   target=[x,y,yaw], actual=[px,py,a])

    def exercise(self):
        self.run_for(1., lambda: self.send(lift=0.))
        self.run_for(4., lambda: self.send(yaw=.5,pitch=.2))
        self.check('original_yaw_pitch',abs(self.q('gimbal_yaw')-.5)<.04 and abs(self.barrel_pitch()-.2)<.04,
                   yaw=self.q('gimbal_yaw'),pitch=self.barrel_pitch(),joint_count=len(self.joints))
        # Publish only slide commands to verify that both gimbal targets persist.
        self.run_for(4.,lambda: self.height_command.publish(Float64(data=.30)))
        self.check('lift_preserves_yaw_pitch',abs(self.q('gimbal_yaw')-.5)<.01 and abs(self.barrel_pitch()-.2)<.01
                   and abs(self.q('lift')-(.30-self.model.params['folded_height']))<.002,
                   yaw=self.q('gimbal_yaw'),pitch=self.barrel_pitch(),lift=self.q('lift'))
        self.run_for(3.,lambda: self.send(yaw=-.3,pitch=.1))
        self.check('gimbal_preserves_lift',abs(self.q('lift')-(.30-self.model.params['folded_height']))<.002
                   and abs(self.q('gimbal_yaw')+.3)<.02 and abs(self.barrel_pitch()-.1)<.01,
                   yaw=self.q('gimbal_yaw'),pitch=self.barrel_pitch(),lift=self.q('lift'))
        self.run_for(4.,lambda: self.send(height=.40))
        self.check('deployed_height',.397<self.geometry_height()<.402 and bool(self.limits&2) and abs(self.barrel_pitch())<.003,
                   overall_height=self.geometry_height(),pitch=self.barrel_pitch(),lift=self.q('lift'),switches=self.limits)
        self.check('telescopic_stage',abs(self.q('lift_stage')-.5*self.q('lift'))<.001,
                   carriage=self.q('lift'),stage=self.q('lift_stage'))
        self.run_for(1.,lambda: self.send(lift=1.))
        self.check('upper_travel_limit',abs(self.q('lift')-self.model.params['lift_stroke'])<.002,lift=self.q('lift'))
        self.run_for(5.,lambda: self.send(lift=-.1))
        self.check('folded_height',self.geometry_height()<.25 and abs(self.barrel_pitch())<.003
                   and abs(self.q('lift'))<.001 and bool(self.limits&1),
                   overall_height=self.geometry_height(),pitch=self.barrel_pitch(),lift=self.q('lift'))
        x0,y0,a0=self.pose()
        self.run_for(2., lambda: self.send(vx=.25))
        self.run_for(2.)
        x,y,a=self.pose(); speed=math.hypot(self.odom.twist.twist.linear.x,self.odom.twist.twist.linear.y)
        self.check('forward_and_command_timeout', x-x0>.25 and abs(y-y0)<.12 and speed<.06, distance=x-x0, lateral=y-y0, stopped_speed=speed)
        y0=y
        self.run_for(2.,lambda: self.send(vy=.25))
        self.run_for(1.,lambda: self.send())
        self.check('sideways', self.pose()[1]-y0>.25, distance=self.pose()[1]-y0)
        self.run_for(3.,lambda: self.send(wz=.35,world=True))
        a=self.pose()[2]; world_error=wrap(a+self.q('gimbal_yaw'))
        self.check('spin_and_world_yaw', abs(a)>.5 and abs(world_error)<.15, base_yaw=a, world_yaw_error=world_error)
        self.run_for(1.,lambda: self.send())
        self.go(.6,0.,duration=12.)
        before=self.contacts
        self.go(3.1,0.,duration=16.)
        self.check('folded_250mm_tunnel_passage',self.contacts==before and self.pass_max_height<.25,
                   new_contacts=self.contacts-before,end_x=self.pose()[0],maximum_height=self.pass_max_height)
        self.run_for(5.,lambda: self.send(height=.40))
        self.check('raise_after_exit',.397<self.geometry_height()<.402,overall_height=self.geometry_height())
        self.check('motion_geometry',not self.self_hits and self.sampled_max_height<.405,
                   overlapping_pairs=sorted(self.self_hits),maximum_height=self.sampled_max_height)
        points=self.cloud_points()
        self.check('odin_point_cloud',self.cloud_count>20 and self.cloud.header.frame_id=='odin_lidar_frame'
                   and self.cloud.width*self.cloud.height==43200 and len(points)>100
                   and self.sim_time()-(self.cloud.header.stamp.sec+self.cloud.header.stamp.nanosec*1e-9)<.5,
                   frames=self.cloud_count,frame_id=self.cloud.header.frame_id,points_per_frame=self.cloud.width*self.cloud.height,valid_points=len(points))
        self.check('sensor_feedback', self.imu_count>100 and self.gimbal_imu_count>100, imu_samples=self.imu_count, gimbal_imu_samples=self.gimbal_imu_count)

    def lidar_exercise(self):
        # The separate fixture has its front wall face at world x=1.95 m.
        observations=[]
        for yaw,pitch,lift in [(0.,0.,0.),(0.,.35,0.),(.4,.35,0.),(.4,.35,.12)]:
            self.run_for(4.,lambda:self.send(yaw=yaw,pitch=pitch,lift=lift))
            points=self.cloud_points()
            points=points[np.isfinite(points).all(axis=1) & (points[:,0]>.2)]
            central=points[(np.abs(points[:,1]/points[:,0])<.02) & (np.abs(points[:,2]/points[:,0])<.02)]
            measured=float(np.median(central[:,0])) if len(central) else float('nan')
            transform=self.model.frames({n:v[0] for n,v in self.joints.items()},self.base_transform())['odin_lidar_frame']
            expected=(1.95-transform[0,3])/transform[0,0]
            world_points=points@transform[:3,:3].T+transform[:3,3]
            ground=world_points[(world_points[:,0]<1.8) & (points[:,2]<-.05)]
            ground_error=float(np.median(ground[:,2])) if len(ground) else float('nan')
            self.check('odin_wall_range',abs(measured-expected)<.04 and abs(ground_error)<.02 and self.cloud.header.frame_id=='odin_lidar_frame',
                       yaw=yaw,pitch=pitch,lift=lift,measured_m=measured,expected_m=expected,ground_error_m=ground_error,valid_points=len(points))
            observations.append((measured,transform.copy()))
        self.check('odin_pitch_independence',abs(observations[1][0]-observations[0][0])<.02,
                   range_change_m=observations[1][0]-observations[0][0])
        self.check('odin_yaw_response',abs(observations[2][0]-observations[1][0])>.05,
                   range_change_m=observations[2][0]-observations[1][0])
        self.check('odin_lift_response',abs(observations[3][1][2,3]-observations[2][1][2,3]-.12)<.003,
                   rise_m=observations[3][1][2,3]-observations[2][1][2,3])

    def collision_exercise(self):
        self.run_for(5.,lambda: self.send(height=.40))
        before=self.contacts
        self.run_for(9.,lambda: self.send(vx=.25,height=.40))
        self.send(height=.40)
        self.check('raised_tunnel_contact', self.contacts>before and self.pose()[0]<2.6, new_contacts=self.contacts-before, end_x=self.pose()[0])


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--namespace',default='red_standard_robot1')
    parser.add_argument('--case',choices=['full','collision','lidar'],default='full')
    parser.add_argument('--output',default='reports/latest')
    args=parser.parse_args()
    output=Path(args.output); output.mkdir(parents=True,exist_ok=True)
    rclpy.init(); node=Demo(args.namespace); error=None
    try:
        node.wait_ready()
        {'full':node.exercise,'collision':node.collision_exercise,'lidar':node.lidar_exercise}[args.case]()
    except Exception as exc:
        error=str(exc); print(error,flush=True)
    finally:
        node.send(pitch=node.barrel_pitch() if 'gimbal_pitch_joint' in node.joints else 0.)
        result={'model':'vertical_slide_yaw_odin_pitch','case':args.case,'passed':error is None,'error':error,'checks':node.results}
        (output/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
        import csv
        with (output/'trajectory.csv').open('w') as f:
            writer=csv.writer(f); writer.writerow(['sim_time','x','y','yaw','gimbal_yaw','barrel_pitch','lift','lift_stage','overall_height','contacts']); writer.writerows(node.trace)
        node.destroy_node(); rclpy.shutdown()
    if error: raise SystemExit(1)


if __name__=='__main__': main()
