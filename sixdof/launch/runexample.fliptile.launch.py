"""
launch file to start up basic movement example
"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import Shutdown
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Define the package.
    package = 'threeDOF'

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir(package), 'rviz/viewcomp.rviz')

    # Locate/load the robot's URDF file (XML).
    urdf = os.path.join(pkgdir('sixdof'), 'urdf/example.urdf')
    with open(urdf, 'r') as file:
        robot_description = file.read()



    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())

    # Configure a node for Robot URDF
    node_urdf = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        arguments  = [urdf],
        on_exit    = Shutdown())

    node_robot_state_publisher_COMMAND = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}]
        # , remappings = [('/joint_states', '/joint_commands')]
        )

    # Configure a node for the joint_publisher.

    node_sim = Node(
        name       = 'sixdof',
        package    = 'sixdof',
        executable = 'flip',
        output     = 'screen',
        on_exit    = Shutdown())

    # Configure a node for the hebi interface.
    node_hebi = Node(
        name       = 'hebi', 
        package    = 'hebiros',
        executable = 'hebinode',
        output     = 'screen',
        parameters = [{'family': 'robotlab'},
                      {'motors': ['3.6', '6.7', '4.1', '3.4', '3.1', '4.3']},
                      {'joints': ['theta1', 'theta2', 'grip', 'theta3', 'theta4', 'theta5']}])

    # Configure a node for Robot Joint GUI
    node_joint_gui = Node(
        name       = 'joint_state_publisher_gui', 
        package    = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        output     = 'screen',
        on_exit    = Shutdown(),
        remappings = [('/joint_states', '/joint_commands')])

    node_rqt = Node(
        name='rqt_plots',
        package = 'rqt_plot',
        executable = 'rqt_plot',
        output     = 'screen',
        arguments =[
            '/joint_states/effort[1]',
                    # '/joint_states/position[1]',
            '/joint_states/effort[3]'
        ],
                    # '/joint_states/position[2]',
                    # '/joint_states/effort[4]',
                    # '/joint_states/position[4]',],
        on_exit    = Shutdown())


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Start the demo and RVIZ
        node_rviz,
        node_urdf,
        node_robot_state_publisher_COMMAND,
        node_sim,
        node_hebi,
        node_rqt,
        # node_joint_gui
    ])
