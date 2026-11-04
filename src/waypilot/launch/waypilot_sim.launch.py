"""
TurtleBot3 in an empty Gazebo world plus all WayPilot nodes.

Usage:
  ros2 launch waypilot waypilot_sim.launch.py [model:=burger] [gui:=true] [port:=8080]
"""

from glob import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, UnsetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SNAP_GUI_VARIABLES = (
    'GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GIO_MODULE_DIR',
    'GSETTINGS_SCHEMA_DIR', 'LOCPATH', 'GDK_PIXBUF_MODULE_FILE', 'GDK_PIXBUF_MODULEDIR',
)
SNAP_SEARCH_PATHS = ('XDG_DATA_DIRS', 'XDG_CONFIG_DIRS')


def _is_snap_path(path):
    return path.startswith(('/snap/', os.path.expanduser('~/snap/')))


def snap_environment_fixes():
    """
    Undo the GUI environment injected by Snap-packaged terminals.

    Terminals of Snap apps (e.g. VS Code installed via snap) point GTK/GIO
    and the XDG search paths into the Snap runtime. The Gazebo window then
    loads incompatible GSettings schemas and aborts with
    "GLib-GIO-ERROR ... does not contain a key named 'antialiasing'".
    """
    actions = [UnsetEnvironmentVariable(name) for name in SNAP_GUI_VARIABLES
               if _is_snap_path(os.environ.get(name, ''))]
    for name in SNAP_SEARCH_PATHS:
        entries = os.environ.get(name, '').split(':')
        kept = [entry for entry in entries if entry and not _is_snap_path(entry)]
        if len(kept) != len([entry for entry in entries if entry]):
            actions.append(SetEnvironmentVariable(name, ':'.join(kept)))
    return actions


def generate_launch_description():
    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    waypilot = get_package_share_directory('waypilot')

    def include(path, arguments=None, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            launch_arguments=(arguments or {}).items(), **kwargs)

    world = os.path.join(tb3_gazebo, 'worlds', 'empty_world.world')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='burger',
                              description='TurtleBot3 model: burger | waffle | waffle_pi'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo client window'),
        DeclareLaunchArgument('port', default_value='8080',
                              description='HTTP port of the web UI'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', LaunchConfiguration('model')),
        # Resolve ground_plane/sun from the local Gazebo install instead of the
        # online model database, which hangs startup on machines without
        # internet access.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        *[AppendEnvironmentVariable('GAZEBO_MODEL_PATH', path)
          for path in sorted(glob('/usr/share/gazebo*/models'))],

        *snap_environment_fixes(),

        include(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'), {'world': world}),
        include(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'),
                condition=IfCondition(LaunchConfiguration('gui'))),
        include(os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py'),
                {'use_sim_time': 'true'}),
        include(os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py'),
                {'x_pose': '0.0', 'y_pose': '0.0'}),

        include(os.path.join(waypilot, 'launch', 'waypilot.launch.py'),
                {'use_sim_time': 'true', 'port': LaunchConfiguration('port')}),
    ])
