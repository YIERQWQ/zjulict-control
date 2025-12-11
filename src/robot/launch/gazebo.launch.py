import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    robot_package_dir = get_package_share_directory('robot')
    urdf_path = os.path.join(robot_package_dir, 'urdf', 'robot.urdf')
    
    # 1. 启动 Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 2. 生成机器人
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-string', open(urdf_path).read(),
            '-z', '0.02'
        ],
        output='screen'
    )

    # 3. 桥接节点（修改为桥接Odometry消息）
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. 速度指令 (ROS -> Gazebo)
            '/cmd_vel_internal@geometry_msgs/msg/Twist]gz.msgs.Twist',
            
            # 2. 真值里程计 (Gazebo -> ROS)
            '/model/robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            # 3. IMU数据 (Gazebo -> ROS)
            '/imu_internal@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        remappings=[
            ('/cmd_vel_internal', '/cmd_vel'),
            ('/imu_internal', '/imu_data'),
            ('/model/robot/odometry', '/odom_ground_truth')  # 映射为ROS话题
        ],
        output='screen'
    )

    # 4. Robot State Publisher (保留)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        bridge,
        robot_state_publisher
    ])
