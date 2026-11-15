#!/usr/bin/env python3
"""Check the expanded model and its control interfaces."""
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import xacro
from generate_lift import build

from model_geometry import Model
import json
model=Model()
root=Path(__file__).resolve().parents[1]
robot=model.robot
links={link.get('name'):link for link in robot.findall('link')}
joints={joint.get('name'):joint for joint in robot.findall('joint')}
assert len(links)==17 and len(joints)==16
for joint in joints.values():
    assert joint.find('parent').get('link') in links
    assert joint.find('child').get('link') in links
mass=0.
for link in links.values():
    inertial=link.find('inertial')
    if inertial is not None:
        m=float(inertial.find('mass').get('value'));assert m>0;mass+=m
        a={k:float(v) for k,v in inertial.find('inertia').attrib.items()}
        matrix=np.array([[a['ixx'],a['ixy'],a['ixz']],[a['ixy'],a['iyy'],a['iyz']],[a['ixz'],a['iyz'],a['izz']]])
        eig=np.linalg.eigvalsh(matrix)
        assert eig[0]>0 and eig[2]<=eig[0]+eig[1]+1e-9
    for mesh in link.findall('visual/geometry/mesh')+link.findall('collision/geometry/mesh'):
        assert (root/'src/tunnel_sentinel_description'/mesh.get('filename')).is_file()
assert abs(mass-(53.643409367134+1.3+model.params['odin']['mass']+model.params['odin']['mount_mass']))<1e-8
control=robot.find('ros2_control');assert control is not None
assert all(child.tag in ('hardware','joint','sensor','gpio') for child in control)
command_types=[]
for joint in control.findall('joint'):
    assert joint.get('name') in joints
    assert all(child.tag in ('command_interface','state_interface','param') for child in joint)
    command_types.extend(child.get('name') for child in joint.findall('command_interface'))
assert command_types.count('effort')==12
original=E.parse(root/'src/tunnel_sentinel_description/reference/tunnel_sentinel.urdf').getroot()
def value(node):
    return node.tag,dict(node.attrib),(node.text or '').strip(),[value(c) for c in node]
for tag in ['link','joint']:
    actual={e.get('name'):value(e) for e in robot.findall(tag)}
    expected={e.get('name'):value(e) for e in original.findall(tag)}
    if tag=='joint':
        original.find("joint[@name='gimbal_yaw_joint']/parent").set('link','lift_carriage_link')
        expected={e.get('name'):value(e) for e in original.findall(tag)}
    assert all(actual[name]==item for name,item in expected.items()),(tag,'original mechanical parameter changed')
assert {e.get('name'):value(e) for e in build()}=={e.get('name'):value(e) for e in robot if e.tag in ('link','joint')}
assert (root/'src/tunnel_sentinel_description/reference/tunnel_sentinel.urdf').read_bytes()==Path('/home/mul/Downloads/tunnel_sentinel.urdf').read_bytes()
stroke=model.params['lift_stroke']
folded=model.bounds(model.pose())[1][2]
expanded=model.bounds(model.pose(lift=stroke))[1][2]
assert folded<.25 and abs(expanded-.40)<1e-6
assert abs(model.params['folded_height']-folded)<1e-9
assert abs(model.frames(model.pose(lift=stroke))['lift_stage_link'][2,3]-model.params['root_height']-stroke/2)<1e-9
assert joints['lift_joint'].get('type')=='prismatic'
assert joints['lift_stage_joint'].get('type')=='prismatic'
assert abs(float(joints['lift_stage_joint'].find('limit').get('upper'))-stroke/2)<1e-9
assert joints['odin_mount_joint'].find('parent').get('link')=='gimbal_yaw_link'
assert joints['odin_joint'].get('type')=='fixed'
assert joints['odin_lidar_joint'].get('type')=='fixed'
for yaw in [0, .7, -1.2]:
    for lift in [0, stroke]:
        low=model.frames(model.pose(pitch=0,yaw=yaw,lift=lift))['odin_lidar_frame']
        tilted=model.frames(model.pose(pitch=model.params['pitch_max'],yaw=yaw,lift=lift))['odin_lidar_frame']
        assert np.allclose(low,tilted,atol=1e-12)
        assert np.allclose(low[:3,0],[np.cos(yaw),np.sin(yaw),0],atol=1e-12)
origin=model.frames(model.pose())['odin_lidar_frame']
raised=model.frames(model.pose(lift=stroke))['odin_lidar_frame']
assert abs(raised[2,3]-origin[2,3]-stroke)<1e-12
sensor=robot.find("gazebo[@reference='odin_link']/sensor")
assert sensor.get('type')=='gpu_lidar'
assert sensor.find('gz_frame_id').text=='odin_lidar_frame'
poses=0
for lift in np.linspace(0,stroke,5):
    for pitch in np.linspace(model.params['pitch_min'],model.params['pitch_max'],5):
        for yaw in np.linspace(-np.pi,np.pi,9):
            for steering in [0,.6,1.2]:
                q=model.pose(pitch,yaw,lift)
                for name in joints:
                    if name.endswith('steering_joint'):q[name]=steering
                hits=model.intersections(q)
                assert not hits,(lift,pitch,yaw,steering,hits)
                poses+=1
result={'passed':True,'reference_unchanged':True,'original_mechanical_parameters_preserved':True,
        'attachment_change':'gimbal_yaw_joint parent: base_link -> lift_carriage_link',
        'links':len(links),'joints':len(joints),'effort_joints':12,'mass_kg':mass,
        'folded_height_m':folded,'deployed_height_m':expanded,'deployed_pitch_rad':0,'lift_stroke_m':stroke,
        'odin_follows_yaw_and_lift':True,'odin_independent_of_pitch':True,
        'odin_sensor_position_m':origin[:3,3].tolist(),'tested_poses':poses,'non_adjacent_overlaps':0,'folded_bounds_m':[v.tolist() for v in model.bounds(model.pose())]}
(root/'reports').mkdir(exist_ok=True)
(root/'reports/model_check.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
