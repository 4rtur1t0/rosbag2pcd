# rosbag2pcd
Convert a rosbag to pcd files and csv files with an ASL/EUROC format.


# USAGE
Use convert.sh to convert from rosbag to PCD. the script runs roscore and publishes
the point clouds. The rosbag2pcd.py script subscribes to the topics and saves data in an EUROC like format.

Edit convert.sh to include the OUTPUT DIRECTORY and rosbag filename.
Edit config.yaml to include the topics in the rosbag

CAUTION: The conversion uses the rosbag application to publish data in the ROS ecosystem. Thus, only one conversion 
should be carried out at any time. 

# INSTALL

pip install pandas
pip install argparse

Install ROS
sudo apt-get install python3-sensor-msgs ros-noetic-sensor-msgs
sudo apt-get install python3-nav-msgs ros-noetic-nav-msgs
