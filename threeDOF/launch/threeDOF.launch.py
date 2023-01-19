"""
launch file to start up threeDOF simulation
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
    urdf = os.path.join(pkgdir(package), 'urdf/threeDOF.urdf')
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
        parameters = [{'robot_description': robot_description}],
        remappings = [('/joint_states', '/joint_commands')])

    # Configure a node for the joint_publisher.
    node_sim = Node(
        name       = 'threeDOF',
        package    = 'threeDOF',
        executable = 'threeDOF',
        output     = 'screen',
        arguments  = ['--dnoe','hello'],
        on_exit    = Shutdown())

    # Configure a node for the hebi interface.
    node_hebi = Node(
        name       = 'hebi', 
        package    = 'hebiros',
        executable = 'hebinode',
        output     = 'screen',
        parameters = [{'family': 'robotlab'},
                      {'motors': ['3.6', '3.2', '3.5']},
                      {'joints': ['theta1', 'theta2', 'theta3']}])
    # Configure a node for Robot Joint GUI
    # node_joint_gui = Node(
    #     name       = 'joint_state_publisher_gui', 
    #     package    = 'joint_state_publisher_gui',
    #     executable = 'joint_state_publisher_gui',
    #     output     = 'screen',
    #     on_exit    = Shutdown())



    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Start the demo and RVIZ
        node_rviz,
        # node_urdf,
        node_robot_state_publisher_COMMAND,
        node_sim,
        node_hebi,
        # node_joint_gui
    ])
