import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    ############# GZ-ROS-LAUNCH INITIAL SETUP #############
    package_name = 'tow_tractor_description'
    package_share_path = os.path.join(get_package_share_directory(package_name))


    ############################## Robot URDF Proccessing Unit ##############################
    #########################################################################################

    ############# ROBOT STATE PUBLISHER NODE #############
    # Path to URDF file
    urdf_file_name = 'stupid_robot.urdf.xacro'
    xacro_file = os.path.join(package_share_path,"urdf", "stupid_robot", urdf_file_name)
    robot_description_config = xacro.process_file(xacro_file)
    # Create a robot_state_publisher node
    params=[
        {'use_sim_time': True},
        {'robot_description': robot_description_config.toxml()},
        ]
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=params,
    )
    #########################################################################################
    #########################################################################################


    ############################## Robot Joint State Publisher ##############################
    #########################################################################################

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output=['screen']
    )

    ##################  VISUALIZE IN RVIZ ##################
    node_rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(package_share_path, 'config', 'tow_description.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    declare_rviz = DeclareLaunchArgument('rviz', default_value='true',
                        description='Open RViz.')
    #########################################################################################
    #########################################################################################


    return LaunchDescription([

        ################ PACKAGE SETUP ACTIONS ################
        set_gz_sim_resource_path,
        declare_rviz,

        ############# LAUNCH NODES AND LAUNCH FILES #############
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz,
    ])