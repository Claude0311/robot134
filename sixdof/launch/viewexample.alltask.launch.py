"""Launch the basic movement example

This launch file is intended only to help visualize the example robot.
To use, run:

   ros2 launch sixdof viewexample.launch.py

This should start
  1) RVIZ, ready to view the robot
  2) The robot_state_publisher to broadcast the robot model
  3) The GUI to move the joints

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import Shutdown
from launch_ros.actions                import Node

from launch.actions                    import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('basic134'), 'rviz/viewurdf.rviz')

    # Locate/load the robot's URDF file (XML).
    urdf = os.path.join(pkgdir('sixdof'), 'urdf/example.urdf')
    with open(urdf, 'r') as file:
        robot_description = file.read()


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())

    # Configure a node for the joint_publisher.
    node_sim = Node(
        name       = 'sixdof',
        package    = 'sixdof',
        executable = 'alltask',
        output     = 'screen',
        parameters = [{'simu':True}],
        on_exit    = Shutdown(),
        remappings = [('/do_action','/brain/do_action')])

    node_ctrl = Node(
        name       = 'sixdof',
        package    = 'sixdof',
        executable = 'ctrl',
        output     = 'screen',
        parameters = [{'simu':True}],
        on_exit    = Shutdown(),
        remappings = [
            ('/lettertarget', '/letter/lettertarget')
    ])

    ######################################################################
    # PREPARE THE CAMERA LAUNCH ELEMENTS

    # Use the standard realsense launch description.  But override
    # what we need.
    rsfile = os.path.join(pkgdir('realsense2_camera'), 'launch/rs_launch.py')

    # Profile is Width x Height x FPS.  0 is default.
    rsargs = {'camera_name':             'camera',  # camera unique name
              'depth_module.profile':    '0,0,0',   # depth W, H, FPS
              'rgb_camera.profile':      '0,0,0',   # color W, H, FPS
              'enable_color':            'true',    # enable color stream
              'enable_infra1':           'false',   # enable infra1 stream
              'enable_infra2':           'false',   # enable infra2 stream
              'enable_depth':            'true',    # enable depth stream
              'align_depth.enable':      'false',   # enabled aligned depth
              'pointcloud.enable':       'false',   # Turn on point cloud
              'allow_no_texture_points': 'true'}    # All points without texture

    incl_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsfile),
        launch_arguments=rsargs.items())

    # Configure the aruco detector node
    node_aruco = Node(
        name       = 'letters', 
        package    = 'detectors',
        executable = 'letters',
        output     = 'screen',
        remappings = [
            ('/image_raw', '/camera/color/image_raw'),
            ('/settarget','/brain/settarget')
        ])


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Start the robot_state_publisher, RVIZ, the GUI, and the demo.
        node_robot_state_publisher,
        node_rviz,
        incl_realsense,
        node_aruco,
        node_sim,
        node_ctrl
    ])
