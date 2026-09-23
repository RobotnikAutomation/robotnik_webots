import os
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from robotnik_common.launch import AddArgumentParser, ExtendedArgument
from webots_ros2_driver.utils import controller_ip_address, controller_protocol
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable(name='SNAP', value=''))
    add_to_launcher = AddArgumentParser(ld)

    arg = ExtendedArgument(
        name='robot_id',
        description='Robot personal name',
        default_value='robot',
        use_env=True,
        environment='ROBOT_ID',
    )
    add_to_launcher.add_arg(arg)
    robot_id = LaunchConfiguration('robot_id')

    arg = ExtendedArgument(
        name='robot',
        description='Robot model (rbkairos, rbrobout, rbwatcher)',
        default_value='rbwatcher',
        use_env=True,
        environment='ROBOT',
    )
    add_to_launcher.add_arg(arg)
    robot = LaunchConfiguration('robot')

    package_dir = get_package_share_directory('robotnik_webots')
    arg = ExtendedArgument(
        name='robot_model',
        description='Robot resource variant (currently the same as robot)',
        default_value=robot,
        use_env=True,
        environment='ROBOT_MODEL',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='x',
        description='x position in world',
        default_value='0.0',
    )
    add_to_launcher.add_arg(arg)
    x_pos = LaunchConfiguration('x')

    arg = ExtendedArgument(
        name='y',
        description='y position in world',
        default_value='0.0',
    )
    add_to_launcher.add_arg(arg)
    y_pos = LaunchConfiguration('y')

    arg = ExtendedArgument(
        name='z',
        description='z position in world',
        default_value='0.0',
    )
    add_to_launcher.add_arg(arg)
    z_pos = LaunchConfiguration('z')

    arg = ExtendedArgument(
        name='yaw',
        description='yaw rotation in radians',
        default_value='0.0',
    )
    add_to_launcher.add_arg(arg)
    yaw = LaunchConfiguration('yaw')
    
    arg = ExtendedArgument(
        name='run_rviz',
        description='Launch RViz2',
        default_value='false'
    )
    add_to_launcher.add_arg(arg)
    run_rviz = LaunchConfiguration('run_rviz')

    params = add_to_launcher.process_arg()
        
    def create_robot_state_publisher(context):
        model = params['robot'].perform(context)
        controller_urdf = os.path.join(
            package_dir, 'resource', model, 'controller.urdf'
        )
        try:
            with open(controller_urdf, 'r', encoding='utf-8') as urdf_file:
                initial_description = urdf_file.read()
        except OSError as error:
            raise RuntimeError(
                f"Cannot read controller URDF for model '{model}': {controller_urdf}"
            ) from error

        # webots_ros2_driver later replaces this with the full Webots-derived
        # description.  Until then, publish a valid description that already
        # contains ros2_control and matching ordinary URDF joints.  The former
        # empty dummy could reach controller manager first and make it abort
        # nondeterministically under load.
        try:
            robot_root = ET.fromstring(initial_description)
        except ET.ParseError as error:
            raise RuntimeError(f"Invalid controller URDF: {controller_urdf}") from error
        ET.SubElement(robot_root, 'link', name='bootstrap_base_link')
        for control_joint in robot_root.findall('./ros2_control/joint'):
            joint_name = control_joint.get('name')
            if not joint_name:
                continue
            child_link = f'bootstrap_{joint_name}_link'
            ET.SubElement(robot_root, 'link', name=child_link)
            urdf_joint = ET.SubElement(
                robot_root, 'joint', name=joint_name, type='continuous'
            )
            ET.SubElement(urdf_joint, 'parent', link='bootstrap_base_link')
            ET.SubElement(urdf_joint, 'child', link=child_link)
            ET.SubElement(urdf_joint, 'axis', xyz='0 0 1')
        initial_description = ET.tostring(robot_root, encoding='unicode')
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': initial_description}],
            namespace=params['robot_id'],
        )]

    ld.add_action(OpaqueFunction(function=create_robot_state_publisher))
    
    spawn_robot_service_call = ExecuteProcess(
    cmd=[
        'ros2', 'service', 'call', 
        '/Ros2Supervisor/spawn_node_from_string', 
        'webots_ros2_msgs/srv/SpawnNodeFromString', 
        [
            TextSubstitution(text='{data: "'), 
            robot, 
            TextSubstitution(text=' { name \\"'), robot_id, TextSubstitution(text='\\" '), 
            TextSubstitution(text='robot_base_link \\"'), robot_id, TextSubstitution(text='_base_link\\" '), 
            TextSubstitution(text='robot_front_rgbd_camera_color_frame \\"'), robot_id, TextSubstitution(text='_front_rgbd_camera_color_frame\\" '),
            TextSubstitution(text='robot_front_rgbd_camera_infra2_frame \\"'), robot_id, TextSubstitution(text='_front_rgbd_camera_infra2_frame\\" '),
            TextSubstitution(text='robot_front_rgbd_camera_depth_frame \\"'), robot_id, TextSubstitution(text='_front_rgbd_camera_depth_frame\\" '),
            TextSubstitution(text='robot_front_rgbd_camera_link \\"'), robot_id, TextSubstitution(text='_front_rgbd_camera_link\\" '),
            TextSubstitution(text='robot_imu_base_link \\"'), robot_id, TextSubstitution(text='_imu_base_link\\" '),
            TextSubstitution(text='robot_back_right_wheel_link \\"'), robot_id, TextSubstitution(text='_back_right_wheel_link\\" '),
            TextSubstitution(text='robot_front_right_wheel_link \\"'), robot_id, TextSubstitution(text='_front_right_wheel_link\\" '),
            TextSubstitution(text='robot_back_left_wheel_link \\"'), robot_id, TextSubstitution(text='_back_left_wheel_link\\" '),
            TextSubstitution(text='robot_front_left_wheel_link \\"'), robot_id, TextSubstitution(text='_front_left_wheel_link\\" '),
            TextSubstitution(text='robot_imu_link \\"'), robot_id, TextSubstitution(text='_imu_link\\" '),
            TextSubstitution(text='robot_top_structure_link \\"'), robot_id, TextSubstitution(text='_top_structure_link\\" '),
            TextSubstitution(text='robot_gps_base_link \\"'), robot_id, TextSubstitution(text='_gps_base_link\\" '),
            TextSubstitution(text='robot_gps_link \\"'), robot_id, TextSubstitution(text='_gps_link\\" '),
            TextSubstitution(text='robot_top_3d_laser_link \\"'), robot_id, TextSubstitution(text='_top_3d_laser_link\\" '),
            TextSubstitution(text='robot_top_3d_laser_base_link \\"'), robot_id, TextSubstitution(text='_top_3d_laser_base_link\\" '),
            # TextSubstitution(text='robot_top_ptz_camera_zoom_color_link \\"'), robot_id, TextSubstitution(text='_top_ptz_camera_zoom_color_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_optical_frame_link \\"'), robot_id, TextSubstitution(text='_top_ptz_camera_optical_frame_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_frame_link \\"'), robot_id, TextSubstitution(text='_top_ptz_camera_frame_link\\" '),
            # TextSubstitution(text='robot_top_ptz_camera_zoom_thermal_link \\"'), robot_id, TextSubstitution(text='_top_ptz_camera_zoom_thermal_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_optical_thermal_frame_link \\"'), robot_id, TextSubstitution(text='_top_ptz_camera_optical_thermal_frame_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_thermal_frame_link \\"'), robot_id, TextSubstitution(text='_top_ptz_camera_thermal_frame_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_tilt_link \\"'), robot_id, TextSubstitution(text='_top_ptz_camera_tilt_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_pan_link \\"'), robot_id, TextSubstitution(text='_top_ptz_camera_pan_link\\" '),
            TextSubstitution(text='robot_top_ptz_camera_base_link \\"'), robot_id, TextSubstitution(text='_top_ptz_camera_base_link\\" '),
            TextSubstitution(text='translation '), 
            x_pos, TextSubstitution(text=' '), 
            y_pos, TextSubstitution(text=' '), 
            z_pos,
            TextSubstitution(text=' rotation 0 0 1 '),
            yaw,
            TextSubstitution(text=' }"}')
        ]
    ],
    output='screen'
    )
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '-0.117500',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', [robot_id, '_base_link'],
            '--child-frame-id', [robot_id, '_base_footprint'],
        ],
        namespace=params['robot_id'],
    )
    ld.add_action(footprint_publisher)
    
    
    worldtf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--x', x_pos, '--y', y_pos, '--z', z_pos,
            '--yaw', yaw, '--pitch', '0', '--roll', '0',
            '--frame-id', 'world', '--child-frame-id', [robot_id, '_odom'],
        ],
        namespace=params['robot_id'],
    )
    ld.add_action(worldtf_publisher)
    
    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        namespace=params['robot_id'],
    )
    
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
        namespace=params['robot_id'],
    )
    
    ros2_control_params = [package_dir, '/resource/',robot,'/ros2control.yml']
    node_name = ['webots_controller_',robot_id]
    webots_controller = (os.path.join(get_package_share_directory('webots_ros2_driver'), 'scripts', 'webots-controller'))
    protocol = controller_protocol()
    ip_address = controller_ip_address() if (protocol == 'tcp') else ''
    ip_address_option = [] if not ip_address else ['--ip-address=' + ip_address]
    port = '1234'
    ros_arguments = [
    f"-r", ["__ns:=/",robot_id],
    f"-p", ["robot_description:=",package_dir,'/resource/',robot,'/','controller.urdf'],
    f"-p", "use_sim_time:=True",
    f"-p", "set_robot_state_publisher:=True"]
    
    
    robot_driver = ExecuteProcess(
    cmd=[
    	webots_controller,
    	["--robot-name=",robot_id],
    	['--protocol=', protocol],
    	*ip_address_option,
    	['--port=', port],
        'ros2',
        '--ros-args',
        *ros_arguments,
        '--params-file',
        ros2_control_params
    ],
    output='screen',
    name=node_name,
    # A reconnect would retrigger WaitForControllerConnection with the same
    # controller-spawner action, which ROS launch cannot execute twice.
    respawn=False,
    # Set WEBOTS_HOME to package directory to load correct controller library
    additional_env={
        'WEBOTS_HOME': get_package_prefix('webots_ros2_driver'),
        'SNAP': '',
    },
    )
    
    waiting_node = WaitForControllerConnection(
        target_driver=robot_driver,
        nodes_to_start=joint_state_broadcaster
    )
    ld.add_action(waiting_node)
    
    def start_driver_after_spawn(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg='ERROR: robot spawn service call failed'),
                EmitEvent(event=Shutdown(reason='Robot spawn failed')),
            ]
        return [LogInfo(msg='Robot spawned; starting Webots driver'), robot_driver]

    def start_diffdrive_after_joint_states(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg='ERROR: joint state broadcaster failed'),
                EmitEvent(event=Shutdown(reason='Controller startup failed')),
            ]
        return [LogInfo(msg='Joint state broadcaster active'), diffdrive_controller_spawner]

    def shutdown_after_driver_exit(event, _context):
        level = 'ERROR' if event.returncode != 0 else 'INFO'
        return [
            LogInfo(msg=f'{level}: Webots driver exited with status {event.returncode}'),
            EmitEvent(event=Shutdown(reason='Webots driver exited')),
        ]

    def shutdown_after_rviz_exit(event, _context):
        level = 'ERROR' if event.returncode != 0 else 'INFO'
        return [
            LogInfo(msg=f'{level}: RViz exited with status {event.returncode}'),
            EmitEvent(event=Shutdown(reason='RViz exited')),
        ]

    # Register every handler before starting the service call so quick process
    # exits cannot be missed.  In particular, the driver must not start until
    # the supervisor has finished inserting this robot into the world.
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot_service_call,
        on_exit=start_driver_after_spawn,
    )))
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=joint_state_broadcaster,
        on_exit=start_diffdrive_after_joint_states,
    )))
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=robot_driver,
        on_exit=shutdown_after_driver_exit,
    )))
    ld.add_action(spawn_robot_service_call)
    
    rviz2_config = [get_package_share_directory('robotnik_webots'),'/resource/', robot,'/rviz_config.rviz']

    rviz_environment = {'SNAP': ''}
    if os.environ.get('SNAP_NAME') and os.environ.get('SNAP_NAME') != 'webots':
        # Graphical processes started from a snap-packaged IDE otherwise load
        # that snap's GTK/GIO modules and can fail against the host glibc.
        for variable in (
            'GDK_PIXBUF_MODULEDIR',
            'GDK_PIXBUF_MODULE_FILE',
            'GIO_MODULE_DIR',
            'GSETTINGS_SCHEMA_DIR',
            'GTK_EXE_PREFIX',
            'GTK_IM_MODULE_FILE',
            'GTK_PATH',
            'LOCPATH',
        ):
            rviz_environment[variable] = ''
        original_xdg_data_dirs = os.environ.get('XDG_DATA_DIRS_VSCODE_SNAP_ORIG')
        if original_xdg_data_dirs:
            rviz_environment['XDG_DATA_DIRS'] = original_xdg_data_dirs

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        namespace=params['robot_id'],
        arguments=['-d', rviz2_config],
        condition=IfCondition(run_rviz),
        additional_env=rviz_environment,
    )

    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=rviz2,
        on_exit=shutdown_after_rviz_exit,
    )))
    ld.add_action(rviz2)

    return ld
