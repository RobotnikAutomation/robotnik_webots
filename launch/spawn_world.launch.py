import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)

def load_urdf(file_path):
    with open(file_path, 'r') as f:
        return f.read()

def generate_launch_description():
    package_dir = get_package_share_directory('robotnik_webots')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'default_world.wbt'),
        ros2_supervisor=True,
#        additional_env={
#        '__NV_PRIME_RENDER_OFFLOAD': '1',
#        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'
#}
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
