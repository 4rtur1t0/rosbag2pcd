PYTHON_INTERPRETER=/home/arvc/Applications/venv/bin/python
OUTPUTDIRECTORY='/media/arvc/INTENSO/DATASETS/dos_vueltas_long_range'
ROSBAGFILENAME=$OUTPUTDIRECTORY/dos_vueltas_long_range.bag
echo "CONVERTING ROSBAG FILENAME: "
echo $ROSBAGFILENAME
echo "RESTARTING ROSCORE"
killall roscore
sleep 3
roscore &
sleep 3
echo "LAUNCHING ROSBAG!"
rosbag play --clock -d 2 $ROSBAGFILENAME &
sleep 3
echo "LAUNCHING ROSBAG TO EUROC SCRIPT!"
$PYTHON_INTERPRETER rosbag2pcd.py $OUTPUTDIRECTORY &