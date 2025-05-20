import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
import xacro  # Add this import

def generate_launch_description():
    package_name = 'klr750'
    package_share_directory = get_package_share_directory(package_name)
    
    # Process URDF
    urdf_file = os.path.join(package_share_directory, 'robot', 'klr750_robot.urdf.xacro')
    robot_description_content = xacro.process_file(urdf_file).toxml()
    
    # Print for debugging
    print(f"URDF file path: {urdf_file}")
    print("Robot description content available: ", len(robot_description_content) > 0)

    # Parameters for robot_state_publisher
    robot_state_publisher_params = {
        'robot_description': robot_description_content,
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    # Declare the use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Set GZ_SIM_RESOURCE_PATH
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=package_share_directory
    )

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r -v4', 'on_exit_shutdown': 'true'}.items()
    )

    # Start Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_state_publisher_params]
    )

    # Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'KLR750',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        set_gz_sim_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
