import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Package name
    package_name = 'smart_t_ai_v1'

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Set Gazebo resource path
    gazebo_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_share_directory(package_name), 'worlds')
    )

    # Include the Gazebo Sim launch file (ros_gz_sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn the robot in Gazebo Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'smart_trolley',
                   '-z', '0.1'],
        output='screen'
    )

    # Bridge for cmd_vel (ROS to Gazebo only)
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen'
    )

    # Bridge for odometry
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )

    # Bridge for laser scan
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    # Bridge for camera
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    # Bridge for joint states
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/empty/model/smart_trolley/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen',
        remappings=[
            ('/world/empty/model/smart_trolley/joint_state', '/joint_states')
        ]
    )

    # Launch them all!
    return LaunchDescription([
        gazebo_resource_path,
        rsp,
        gazebo,
        spawn_entity,
        bridge_cmd_vel,
        bridge_odom,
        bridge_scan,
        bridge_camera,
        bridge_joint_states,
    ])
