import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    pkg_name = 'tello_description'
    urdf_file_subpath = 'urdf/tello.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), urdf_file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Launch world
    gz_start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args' : '-r ' + 'empty.sdf'
            }.items(),
    )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-name', 'tello',
                                   '-topic', '/robot_description',
                                   '-z', '1.0'],
                        output='screen')

    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # node_controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #         {'robot_description': robot_description_raw},
    #         os.path.join(get_package_share_directory(pkg_name), 'config', 'tello_controllers.yaml')
    #     ],
    #     output='screen'
    # )
    # node_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['propeller_velocity_controller'],
    #     output='screen'
    # )

    # node_joint_state_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    #     output='screen'
    # )

    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz', 'ryze_view.rviz')]
    )

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
                    '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'gz.msgs.Clock',
                    '/world/empty/model/tello/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'gz.msgs.Model',
                    '/model/tello/imu'                  + '@sensor_msgs/msg/Imu'       + '[' + 'gz.msgs.IMU',
                    '/tello/cmd_vel'                    + '@geometry_msgs/msg/Twist'   + '@' + 'gz.msgs.Twist',
                    ],
        parameters= [{'qos_overrides./tello.subscriber.reliability': 'reliable'},{'qos_overrides./tello.subscriber.durability': 'transient_local'}],
        remappings= [
                    ('/world/empty/model/tello/joint_state', 'joint_states'),
                    ('/model/tello/imu', '/imu'),
                    ('/tello/cmd_vel', '/cmd_vel'),
                    ],
        output='screen'
    )

    
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(gz_start_world)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_ros_gz_bridge)
    # ld.add_action(node_joint_state_publisher)
    ld.add_action(node_rviz)
    # ld.add_action(node_controller_manager)
    # ld.add_action(node_spawner)
    # ld.add_action(node_joint_state_broadcaster)
    return ld