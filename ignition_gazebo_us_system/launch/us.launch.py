import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete, OnProcessStart
                                   
def generate_launch_description():

  pkg_ignition_us = get_package_share_directory('ignition_gazebo_us_system')
  ign_resource_path = SetEnvironmentVariable(
    name='IGN_GAZEBO_RESOURCE_PATH',
    value=[
      os.path.join(pkg_ignition_us, 'worlds'), ':' +
      os.path.join(pkg_ignition_us, 'models'), ':' ] )
  
  world_file = PathJoinSubstitution([FindPackageShare('ignition_gazebo_us_system'),'worlds','us.sdf'])
  
  ignition_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          PathJoinSubstitution([FindPackageShare('ros_gz_sim'),'launch','gz_sim.launch.py'])
          ),
      launch_arguments={"gz_args": [world_file, " -v 4"]}.items()
  )
  
  #edumip_description = IncludeLaunchDescription(
  #    PythonLaunchDescriptionSource(
  #        PathJoinSubstitution([FindPackageShare('edumip_description'),'launch','edumip_startup.launch.py'])
  #    )
  #)

  #edumip_create = Node(
  #    package="ros_ign_gazebo",
  #    executable="create",
  #    name="create",
  #    arguments=["-topic /robot_description -x 4.0 -z 0.05"],
  #    output="screen"
  #)

  bridge = Node(
      package="ros_ign_bridge",
      executable="parameter_bridge",
      name="parameter_bridge",
      arguments=["/pose@sensor_msgs/msg/Pose[ignition.msgs.Pose"]
      )

  return LaunchDescription([
    ign_resource_path,
    #edumip_description,
    ignition_launch,
    bridge
  ]
  )

