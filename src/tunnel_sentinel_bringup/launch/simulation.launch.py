from pathlib import Path
import fcntl
import os
import tempfile
import shutil
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def start(context):
    # Keep the file open until the launcher and all its child processes exit.
    global _instance_lock
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    lock_dir = Path(os.environ.get('ROS_HOME', tempfile.gettempdir())) / 'locks'
    lock_dir.mkdir(parents=True, exist_ok=True)
    lock = (lock_dir / f'tunnel_sentinel_domain_{domain}.lock').open('a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        return [LogInfo(msg=f'ROS domain {domain} 的哨兵仿真已在运行。可在另一终端执行 make demo；重新开始场景时，先结束原启动终端。')]
    _instance_lock = lock
    desc = Path(get_package_share_directory('tunnel_sentinel_description'))
    bringup = Path(get_package_share_directory('tunnel_sentinel_bringup'))
    ctrl = Path(get_package_share_directory('universal_controller'))
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    software = LaunchConfiguration('software_rendering').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    model = yaml.safe_load((desc / 'config/model.yaml').read_text())
    controllers = yaml.safe_load((ctrl / 'config/controllers.yaml').read_text())
    parameters = controllers['/**/sentry_controller']['ros__parameters']
    for key in ('wheel_radius', 'half_spacing', 'pitch_min', 'pitch_max', 'folded_height', 'lift_stroke', 'lift_speed', 'lift_mass', 'pitch_mass', 'pitch_com_x'):
        parameters[key] = model[key]
    temp = tempfile.TemporaryDirectory(prefix='sentry_sim_')
    controller_path = Path(temp.name) / 'controllers.yaml'
    controller_path.write_text(yaml.safe_dump(controllers))
    robot_xml = xacro.process_file(str(desc / 'urdf/tunnel_sentinel.urdf.xacro'), mappings={
        'model_file': str(desc / 'config/model.yaml'), 'source_file': str(desc / 'urdf/tunnel_sentinel_lift.urdf'),
        'controllers_file': str(controller_path), 'namespace': ns,
    }).toxml()
    import xml.etree.ElementTree as ET
    robot_tree=ET.fromstring(robot_xml)
    # Primitive collisions preserve the central opening and hollow sleeve walls.
    primitives = yaml.safe_load((desc / 'meshes/primitives.yaml').read_text())
    for link in robot_tree.findall('link'):
        for collision in list(link.findall('collision')):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                continue
            for index, part in enumerate(primitives[Path(mesh.get('filename')).name]):
                obj = ET.SubElement(link, 'collision', name=f"{collision.get('name')}_{index}")
                ET.SubElement(obj, 'origin', xyz=' '.join(map(str, part.get('xyz', [0, 0, 0]))),
                              rpy=' '.join(map(str, part.get('rpy', [0, 0, 0]))))
                geometry = ET.SubElement(obj, 'geometry')
                attributes = {key: ' '.join(map(str, part[key])) if isinstance(part[key], list) else str(part[key])
                              for key in ('size', 'radius', 'length') if key in part}
                ET.SubElement(geometry, part['type'], **attributes)
            link.remove(collision)
    mesh_folder=Path(temp.name)/'meshes'
    shutil.copytree(desc/'meshes',mesh_folder)
    for mesh in robot_tree.iter('mesh'):
        mesh.set('filename',(mesh_folder/Path(mesh.get('filename')).name).as_uri())
    robot_xml=ET.tostring(robot_tree,encoding='unicode')
    robot_file = Path(temp.name) / 'robot.urdf'
    robot_file.write_text(robot_xml)
    bridge = [
        {'ros_topic_name': '/clock', 'gz_topic_name': '/clock', 'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock', 'direction': 'GZ_TO_ROS'},
    ]
    for topic, ros_type, gz_type in [
        ('imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU'),
        ('gimbal_imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU'),
        ('ground_truth/odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry'),
        ('tunnel_contacts', 'ros_gz_interfaces/msg/Contacts', 'gz.msgs.Contacts'),
    ]:
        bridge.append({'ros_topic_name': f'/{ns}/{topic}', 'gz_topic_name': f'/{ns}/{topic}' if topic != 'tunnel_contacts' else '/tunnel_contacts',
                       'ros_type_name': ros_type, 'gz_type_name': gz_type, 'direction': 'GZ_TO_ROS'})
    bridge.append({'ros_topic_name': f'/{ns}/odin1/cloud_raw', 'gz_topic_name': f'/{ns}/odin1/scan/points',
                   'ros_type_name': 'sensor_msgs/msg/PointCloud2', 'gz_type_name': 'gz.msgs.PointCloudPacked', 'direction': 'GZ_TO_ROS'})
    bridge.append({'ros_topic_name': '/tf', 'gz_topic_name': f'/{ns}/ground_truth/tf', 'ros_type_name': 'tf2_msgs/msg/TFMessage', 'gz_type_name': 'gz.msgs.Pose_V', 'direction': 'GZ_TO_ROS'})
    bridge_path = Path(temp.name) / 'bridge.yaml'
    bridge_path.write_text(yaml.safe_dump(bridge))
    world = LaunchConfiguration('world').perform(context)
    world = world or str(bringup / 'worlds/tunnel_test.sdf')
    render_env = {'QT_QPA_PLATFORM': 'xcb', 'GSETTINGS_BACKEND': 'memory'}
    render_engine = 'ogre2'
    gui_config = bringup / 'config/gui.config'
    if software:
        render_engine = 'ogre'
        render_env.update({'LIBGL_ALWAYS_SOFTWARE': '1', '__GLX_VENDOR_LIBRARY_NAME': 'mesa',
                           'GALLIUM_DRIVER': 'llvmpipe'})
        gui_config = Path(temp.name) / 'gui.config'
        gui_config.write_text((bringup / 'config/gui.config').read_text().replace('<engine>ogre2</engine>', '<engine>ogre</engine>'))
    simulator = ExecuteProcess(cmd=['gz', 'sim', '-r', '-v', '2', '--render-engine-server', render_engine, *(['--gui-config', str(gui_config), '--render-engine-gui', render_engine] if gui else ['-s', *([] if software else ['--headless-rendering'])]), world], additional_env=render_env, output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', arguments=['-name', 'tunnel_sentinel', '-file', str(robot_file), '-z', str(model['root_height']+.004)], output='screen')
    state_pub = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=ns,
                     parameters=[{'robot_description': robot_xml, 'use_sim_time': True}], output='screen')
    spawner = Node(package='controller_manager', executable='spawner',
                   arguments=['joint_state_broadcaster', 'sentry_controller', '-c', f'/{ns}/controller_manager', '--controller-manager-timeout', '60', '--switch-timeout', '30'], output='screen')
    actions = [simulator, state_pub,
               Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{'config_file': str(bridge_path), 'use_sim_time': True}], output='screen'),
               spawn, RegisterEventHandler(OnProcessExit(target_action=spawn,
                   on_exit=lambda event, _: [spawner] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Robot creation failed.'))])),
               RegisterEventHandler(OnProcessExit(target_action=spawner,
                   on_exit=lambda event, _: [] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Controller startup failed.'))])),
               RegisterEventHandler(OnProcessExit(target_action=simulator,
                   on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited.'))])),
               RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=lambda _: temp.cleanup() or [])]))]
    if use_rviz:
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(desc / 'rviz/sentry.rviz')], parameters=[{'use_sim_time': True}], additional_env=render_env))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('software_rendering', default_value='false'),
        DeclareLaunchArgument('namespace', default_value='red_standard_robot1'),
        DeclareLaunchArgument('world', default_value=''),
        OpaqueFunction(function=start),
    ])
