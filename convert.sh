PYTHON_INTERPRETER=/home/arvc/Applications/venv/bin/python
OUTPUTDIRECTORY='/media/arvc/INTENSO/DATASETS/dos_vueltas_long_range'
ROSBAGFILENAME=$OUTPUTDIRECTORY/dos_vueltas_long_range.bag
echo $ROSBAGFILENAME
killall roscore
roscore &
rosbag play --clock -d 2 $ROSBAGFILENAME &
$PYTHON_INTERPRETER rosbag2pcd.py $OUTPUTDIRECTORY &