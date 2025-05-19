from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_px4_sitl(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    num = int(LaunchConfiguration('num_drones').perform(context))
    world = LaunchConfiguration('world').perform(context)

    return [ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c',
            f'bash ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh '
            f'-m {model} -n {num} -w {world}; exec bash'
        ],
        output='screen'
    )]

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('model', default_value='dropla_450_drone_stereo_camera', description='Drone model to spawn'),
        DeclareLaunchArgument('num_drones', default_value='2', description='Number of drones to launch'),
        DeclareLaunchArgument('world', default_value='empty', description='Gazebo world to launch'),

        # Launch PX4 SITL
        OpaqueFunction(function=launch_px4_sitl),

        # Launch Micro XRCE Agent after 5 seconds
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c', 'MicroXRCEAgent udp4 -p 8888; exec bash'],
                    output='screen'
                )
            ]
        ),

        # Launch QGroundControl after 10 seconds
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c', '~/Downloads/QGroundControl.AppImage; exec bash'],
                    output='screen'
                )
            ]
        )
    ])

