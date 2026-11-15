#!/usr/bin/env python3
"""Generate slide and yaw-mounted Odin1 extensions from the supplied reference file."""
from pathlib import Path
import xml.etree.ElementTree as E
import yaml

ROOT = Path(__file__).resolve().parents[1]
DESC = ROOT / 'src/tunnel_sentinel_description'


def build():
    robot = E.parse(DESC / 'reference/tunnel_sentinel.urdf').getroot()
    params = yaml.safe_load((DESC / 'config/model.yaml').read_text())
    # The original Yaw transform is now relative to the moving carriage.
    robot.find("joint[@name='gimbal_yaw_joint']/parent").set('link', 'lift_carriage_link')
    for name, mass, width in [('lift_base', .5, .08), ('lift_stage', .35, .064), ('lift_carriage', .45, .048)]:
        link = E.SubElement(robot, 'link', name=name + '_link')
        inertial = E.SubElement(link, 'inertial')
        E.SubElement(inertial, 'origin', xyz='0 0 -0.02')
        E.SubElement(inertial, 'mass', value=str(mass))
        transverse = mass * (width * width + .1 * .1) / 12
        E.SubElement(inertial, 'inertia', ixx=str(transverse), iyy=str(transverse),
                     izz=str(mass * width * width / 6), ixy='0', ixz='0', iyz='0')
        for tag in ('visual', 'collision'):
            obj = E.SubElement(link, tag, name=name + '_' + tag)
            E.SubElement(E.SubElement(obj, 'geometry'), 'mesh', filename='meshes/' + name + '.stl')
    for name, child, kind in [('lift_base_joint', 'lift_base_link', 'fixed'),
                               ('lift_joint', 'lift_carriage_link', 'prismatic'),
                               ('lift_stage_joint', 'lift_stage_link', 'prismatic')]:
        joint = E.SubElement(robot, 'joint', name=name, type=kind)
        E.SubElement(joint, 'parent', link='base_link')
        E.SubElement(joint, 'child', link=child)
        E.SubElement(joint, 'origin', xyz='0 0 0')
        if kind == 'prismatic':
            ratio = .5 if name == 'lift_stage_joint' else 1.
            E.SubElement(joint, 'axis', xyz='0 0 1')
            E.SubElement(joint, 'limit', lower='0', upper=str(params['lift_stroke'] * ratio),
                         effort='300', velocity=str(params['lift_speed'] * ratio))
            E.SubElement(joint, 'dynamics', damping='2', friction='0')
    odin = params['odin']
    for name, mass, size, com in [
            ('odin_mount', odin['mount_mass'], [.24, .20, .003], [.09, .10, -.0075]),
            ('odin', odin['mass'], odin['body_size'], [0, 0, 0])]:
        link = E.SubElement(robot, 'link', name=name + '_link')
        inertial = E.SubElement(link, 'inertial')
        E.SubElement(inertial, 'origin', xyz=' '.join(map(str, com)))
        E.SubElement(inertial, 'mass', value=str(mass))
        x, y, z = size
        E.SubElement(inertial, 'inertia', ixx=str(mass*(y*y+z*z)/12),
                     iyy=str(mass*(x*x+z*z)/12), izz=str(mass*(x*x+y*y)/12), ixy='0', ixz='0', iyz='0')
        for tag in ('visual', 'collision'):
            obj = E.SubElement(link, tag, name=name + '_' + tag)
            mesh = 'odin1' if name == 'odin' else name
            E.SubElement(E.SubElement(obj, 'geometry'), 'mesh', filename='meshes/' + mesh + '.stl')
    E.SubElement(robot, 'link', name='odin_lidar_frame')
    for name, parent, child, xyz, rpy in [
            ('odin_mount_joint', 'gimbal_yaw_link', 'odin_mount_link', [0,0,0], [0,0,0]),
            ('odin_joint', 'odin_mount_link', 'odin_link', odin['body_xyz'], odin['body_rpy']),
            ('odin_lidar_joint', 'odin_link', 'odin_lidar_frame', odin['sensor_xyz'], [0,0,0])]:
        joint = E.SubElement(robot, 'joint', name=name, type='fixed')
        E.SubElement(joint, 'parent', link=parent)
        E.SubElement(joint, 'child', link=child)
        E.SubElement(joint, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy)))
    return robot


if __name__ == '__main__':
    robot = build()
    E.indent(robot, space='  ')
    output = DESC / 'urdf/tunnel_sentinel_lift.urdf'
    E.ElementTree(robot).write(output, encoding='utf-8', xml_declaration=True)
    print(output)
