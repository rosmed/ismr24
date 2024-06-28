import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    rviz_arg = DeclareLaunchArgument('rviz',
                                     default_value='false',
                                     choices=['true', 'false'])

    rviz_config = LaunchConfiguration('rviz')
    
    pluslib_data_path = SetEnvironmentVariable(name='PLUSLIB_DATA_DIR', value=[os.path.join('ignition_gazebo_us_system', 'config')])
    pluslib_image_path = SetEnvironmentVariable(name='PLUCONFIG_IMAGE_DIR', value=[os.path.join('ignition_gazebo_us_system', 'config')])
    pluslib_model_path = SetEnvironmentVariable(name='PLUSCONFIG_MODEL_DIR', value=[os.path.join('ignition_gazebo_us_system', 'config')])

    pkg_ur5_us_bringup = get_package_share_directory('ur5_us_bringup')
    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
                                               value=[os.path.join(pkg_ur5_us_bringup, 'worlds'), ':' +
                                                      os.path.join(pkg_ur5_us_bringup, 'models'), ':' ] )

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ur5_us_bringup"), "config", "rsp_ur_controllers.yaml"]
    )

    initial_positions = PathJoinSubstitution(
        [FindPackageShare("ur5_us_bringup"), "config", "initial_positions.yaml"]
    )

    rviz_config_file = os.path.join( get_package_share_directory('ur5_us_bringup'), 'rviz', 'ur_us.rviz' )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur5_us_bringup"), "urdf", "ur5_us.urdf.xacro"]
            ),
            " ",
            "name:=ur",
            " ",
            "ur_type:=ur5e",
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
            " ",
            "initial_positions_file:=",
            initial_positions,            
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["joint_velocities", "--controller-manager", "/controller_manager"],
    )

    # Ignition nodes
    ignition_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 4 ismr.sdf"}.items(),
    )

    ignition_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-string", robot_description_content, "-name", "ur", ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/ft_probe@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench',
        ],
        output='screen',
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz_config)
    )

    rrmc = Node(
        package="rrmc",
        executable="rrmc_node",
        parameters=[robot_description],
        output="both",
        remappings=[
            ('command', '/spacenav/twist'),
            ('velocities', '/joint_velocities/commands')
        ],
    )
    
    nodes_to_start = [
        rviz_arg,
        pluslib_data_path,
        pluslib_image_path,
        pluslib_model_path,
        ign_resource_path,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        ignition_launch_description,
        ignition_spawn_robot,
        bridge,
        rrmc,
        rviz
    ]

    return LaunchDescription(nodes_to_start)
