# Run everytime you turn on NCU
cd ~/robotws
colcon build --symlink-install
source ~/robotws/install/setup.bash

# Rerun everytime you change the launch file
cd ~/robotws
colcon build --symlink-install

# Show camera image
rqt

# Run launch file
ros2 launch <package_name> <launch_file>

# Launch files
# Plot torque or set robot position 
ros2 launch sixdof runexample.float.launch.py

# Grap tile with aruco
ros2 launch detectors realsense_perstrans.launch.py # publish position of aruco #47 
ros2 launch sixdof runexample.launch.py             # grap tile based on aruco pos

# Detect piles
ros2 launch detectors realsense_pile.launch.py

# Flip a tile
# publish position of aruco #47 
ros2 launch detectors realsense_perstrans.launch.py
# view rviz
ros2 launch sixdof viewexample.fliptile.launch.py
# run hebi
ros2 launch sixdof runexample.fliptile.launch.py