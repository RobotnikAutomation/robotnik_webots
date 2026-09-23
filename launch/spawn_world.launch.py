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
import os
import re
import tempfile

from launch import LaunchDescription
from pathlib import Path

from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from robotnik_common.launch import ExtendedArgument, AddArgumentParser
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher, WebotsLauncher


def substitute_param_context(param, context):
    """Resolve a parameter if it is a LaunchConfiguration."""
    if isinstance(param, LaunchConfiguration):
        return param.perform(context)
    return param

def generate_webots_launch_description(context, params):
    world_path = substitute_param_context(params['world_path'], context)
    if not Path(world_path).is_file():
        raise RuntimeError(f"Webots world does not exist: {world_path}")

    gui = substitute_param_context(params['gui'], context).strip().lower()
    if gui not in ('true', 'false'):
        raise RuntimeError("gui must be either 'true' or 'false'")

    temporary_world = None
    render_fps = os.environ.get('ROBOTNIK_RENDER_FPS')
    if gui == 'true' and render_fps:
        try:
            render_fps_value = int(render_fps)
        except ValueError as exc:
            raise RuntimeError(
                "ROBOTNIK_RENDER_FPS must be an integer between 1 and 1000"
            ) from exc
        if not 1 <= render_fps_value <= 1000:
            raise RuntimeError(
                "ROBOTNIK_RENDER_FPS must be an integer between 1 and 1000"
            )
        content = Path(world_path).read_text(encoding='utf-8')
        content, replacements = re.subn(
            r'(?m)^(\s*FPS\s+)\d+(?:\.\d+)?',
            rf'\g<1>{render_fps_value}', content, count=1,
        )
        if replacements != 1:
            raise RuntimeError(f"WorldInfo.FPS not found in {world_path}")
        temporary = tempfile.NamedTemporaryFile(
            # Keep the temporary world beside the original. Worlds reference
            # local PROTOs with relative paths (for example ../protos), which
            # would be broken if this copy lived in /tmp.
            mode='w', suffix='.wbt', prefix='.robotnik-fps-',
            dir=str(Path(world_path).parent), delete=False,
            encoding='utf-8',
        )
        temporary.write(content)
        temporary.close()
        temporary_world = Path(temporary.name)
        world_path = str(temporary_world)

    webots = WebotsLauncher(
        world=world_path,
        mode='realtime',
        ros2_supervisor=True,
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            # A shell opened from a snap-packaged IDE inherits SNAP variables.
            # Webots then chooses the IDE's private temp directory while the
            # external controllers look in /tmp, so IPC discovery never works.
            'SNAP': '',
        },
        gui=gui,
    )
    # The upstream launcher enables respawn by default.  A supervisor killed
    # during an intentional benchmark shutdown must not be restarted while
    # Webots itself is already going away.
    webots._supervisor = Ros2SupervisorLauncher(respawn=False)

    def remove_temporary_world(_context):
        if temporary_world is not None:
            temporary_world.unlink(missing_ok=True)
        return []

    return [
        webots,
        webots._supervisor,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[
                    OpaqueFunction(function=remove_temporary_world),
                    EmitEvent(event=Shutdown()),
                ],
            )
        ),
    ]


def generate_launch_description():
    raw_args = [
        ("world", "World Name", "demo", ""),
        ("world_path", "Path to World File", [FindPackageShare('robotnik_webots'), '/worlds/', LaunchConfiguration('world'), '.wbt'], ""),
        ("gui", "Launch GUI", "true", ""),
    ]

    ld = LaunchDescription()
    # The supervisor is a separate action and needs the same environment as
    # Webots for local IPC discovery.
    ld.add_action(SetEnvironmentVariable(name='SNAP', value=''))
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
