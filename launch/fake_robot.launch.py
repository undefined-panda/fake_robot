import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    fake_robot_dir = FindPackageShare(package='fake_robot')
    
    # Declare launch arguments
    landmarks_file_arg = DeclareLaunchArgument(
        'landmarks_file',
        default_value=PathJoinSubstitution([fake_robot_dir, 'data', 'landmarks.csv']),
        description='Path to the landmarks CSV file'
    )
    
    circle_radius_arg = DeclareLaunchArgument(
        'circle_radius',
        default_value='5.0',
        description='Radius of the circle trajectory in meters'
    )
    
    circle_speed_arg = DeclareLaunchArgument(
        'circle_speed',
        default_value='1.0',
        description='Speed of the robot along the circle in m/s'
    )
    
    circle_center_x_arg = DeclareLaunchArgument(
        'circle_center_x',
        default_value='0.0',
        description='X coordinate of the circle center'
    )
    
    circle_center_y_arg = DeclareLaunchArgument(
        'circle_center_y',
        default_value='0.0',
        description='Y coordinate of the circle center'
    )
    
    observation_radius_arg = DeclareLaunchArgument(
        'observation_radius',
        default_value='3.0',
        description='Radius around robot to observe landmarks in meters'
    )
    
    measurement_noise_variance_arg = DeclareLaunchArgument(
        'measurement_noise_variance',
        default_value='0.1',
        description='Variance of Gaussian noise added to landmark measurements'
    )

    # Robot odometry noise parameters (for noisy odometry)
    # Noise now affects traveled distance (along motion direction) and orientation
    robot_noise_variance_x_arg = DeclareLaunchArgument(
        'robot_noise_variance_x',
        default_value='0.1',
        description='Variance of accumulated noise on traveled distance (m^2)'
    )

    robot_noise_variance_y_arg = DeclareLaunchArgument(
        'robot_noise_variance_y',
        default_value='0.01',
        description='Variance of accumulated noise on traveled distance (m^2) [currently unused, x used for both]'
    )

    robot_noise_variance_theta_arg = DeclareLaunchArgument(
        'robot_noise_variance_theta',
        default_value='0.0001',
        description='Variance of accumulated noise on robot orientation (rad^2)'
    )
    
    # Create the node
    fake_robot_node = Node(
        package='fake_robot',
        executable='fake_robot',
        name='fake_robot_node',
        output='screen',
        parameters=[
            {
                'landmarks_file': LaunchConfiguration('landmarks_file'),
                'circle_radius': LaunchConfiguration('circle_radius'),
                'circle_speed': LaunchConfiguration('circle_speed'),
                'circle_center_x': LaunchConfiguration('circle_center_x'),
                'circle_center_y': LaunchConfiguration('circle_center_y'),
                'observation_radius': LaunchConfiguration('observation_radius'),
                'measurement_noise_variance': LaunchConfiguration('measurement_noise_variance'),
                'robot_noise_variance_x': LaunchConfiguration('robot_noise_variance_x'),
                'robot_noise_variance_y': LaunchConfiguration('robot_noise_variance_y'),
                'robot_noise_variance_theta': LaunchConfiguration('robot_noise_variance_theta'),
            }
        ]
    )

    # Create the node
    mcl_node = Node(
        package='fake_robot',
        executable='mcl_node',
        name='mcl_node',
        output='screen',
        parameters=[
            {
                'measurement_noise_variance': LaunchConfiguration('measurement_noise_variance'),
                'robot_noise_variance_x': LaunchConfiguration('robot_noise_variance_x'),
                'robot_noise_variance_y': LaunchConfiguration('robot_noise_variance_y'),
                'robot_noise_variance_theta': LaunchConfiguration('robot_noise_variance_theta'),
            }
        ]
    )
    
    return LaunchDescription([
        landmarks_file_arg,
        circle_radius_arg,
        circle_speed_arg,
        circle_center_x_arg,
        circle_center_y_arg,
        observation_radius_arg,
        measurement_noise_variance_arg,
        robot_noise_variance_x_arg,
        robot_noise_variance_y_arg,
        robot_noise_variance_theta_arg,
        fake_robot_node,
        mcl_node
    ])
