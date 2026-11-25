import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sar = get_package_share_directory('sar_drone_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_sar, 'worlds', 'earthquake_disaster.sdf')
    models_path = os.path.join(pkg_sar, 'models')
    bridge_config = os.path.join(pkg_sar, 'config', 'ros_gz_bridge.yaml')
    rviz_config = os.path.join(pkg_sar, 'config', 'usar_gcs.rviz')

    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = f'{models_path}:{existing_gz_path}' if existing_gz_path else models_path

    headless = LaunchConfiguration('headless')
    show_window = LaunchConfiguration('show_window')
    dual_windows = LaunchConfiguration('dual_windows')
    rviz = LaunchConfiguration('rviz')
    battery_drain_scale = LaunchConfiguration('battery_drain_scale')

    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path,
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
        condition=UnlessCondition(headless),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-s -r --headless-rendering {world_path}'}.items(),
        condition=IfCondition(headless),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    flight_manager = Node(
        package='sar_drone_sim',
        executable='flight_manager',
        name='flight_manager',
        parameters=[{'battery_drain_scale': battery_drain_scale}],
        output='screen',
    )

    thermal_detector = Node(
        package='sar_drone_sim',
        executable='thermal_detector',
        name='thermal_detector',
        parameters=[{'show_window': False}],
        output='screen',
    )

    gcs_hud = Node(
        package='sar_drone_sim',
        executable='gcs_hud',
        name='gcs_hud',
        parameters=[{
            'show_window': show_window,
            'dual_windows': dual_windows,
        }],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo server headless without GUI.',
        ),
        DeclareLaunchArgument(
            'show_window',
            default_value='true',
            description='Display OpenCV USAR Ground Control Station HUD window(s).',
        ),
        DeclareLaunchArgument(
            'dual_windows',
            default_value='false',
            description='Split GCS into separate Pilot FPV and Observer Thermal+Map windows.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 3D tactical view.',
        ),
        DeclareLaunchArgument(
            'battery_drain_scale',
            default_value='1.5',
            description='Battery discharge speed multiplier for demos.',
        ),
        set_gz_resource,
        gz_sim_gui,
        gz_sim_headless,
        bridge,
        flight_manager,
        thermal_detector,
        gcs_hud,
        rviz_node,
    ])
