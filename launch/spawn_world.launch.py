# Copyright (c) 2025, Robotnik Automation S.L.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from robotnik_common.launch import ExtendedArgument, AddArgumentParser
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch.actions import OpaqueFunction


def substitute_param_context(param, context):
    """Resolve a parameter if it is a LaunchConfiguration."""
    if isinstance(param, LaunchConfiguration):
        return param.perform(context)
    return param

def generate_webots_launch_description(context, params):

    ret = []
    
    webots = WebotsLauncher(
        world=substitute_param_context(params['world_path'], context),
        ros2_supervisor=True,
        additional_env={
        '__NV_PRIME_RENDER_OFFLOAD': '1',
        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'
        },
        gui=substitute_param_context(params['gui'], context),
    )

    ret.append(webots)
    ret.append(webots._supervisor)
    ret.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    )
    
    return ret


def generate_launch_description():
    raw_args = [
        ("world", "World Name", "demo", ""),
        ("world_path", "Path to World File", [FindPackageShare('robotnik_webots'), '/worlds/', LaunchConfiguration('world'), '.wbt'], ""),
        ("gui", "Launch GUI", "true", ""),
    ]

    ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ld)
    for arg in raw_args:
        extended_arg = ExtendedArgument(
            name=arg[0],
            description=arg[1],
            default_value=arg[2],
            use_env=True,
            environment=arg[3],
        )
        add_to_launcher.add_arg(extended_arg)
    params = add_to_launcher.process_arg()
    ld.add_action(OpaqueFunction(function=generate_webots_launch_description, args=[params]))
    return ld