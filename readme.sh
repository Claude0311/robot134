# Run everytime you turn on NCU
cd ~/robotws
colcon build --symlink-install
source ~/robotws/install/setup.bash

# Rerun everytime you change the launch file
cd ~/robotws
colcon build --symlink-install
colcon build --symlink-install --packages-select sixdof
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

# combine all
ros2 launch sixdof viewexample.alltask.launch.py
ros2 launch sixdof runexample.alltask.launch.py

# ros2 topic pub -1 /brain/settarget /std_msgs/string '{"data":"0"}'
ros2 topic pub -1 /cur_phase std_msgs/Int8 '{"data":"0"}'
ros2 topic pub -1 /set_letter std_msgs/String '{"data":"boot"}'
