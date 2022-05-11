PYTHON_INTERPRETER=/home/arvc/Applications/venv/bin/python
DIRECTORY='/media/arvc/INTENSO/DATASETS/dos_vueltas'
FILENAME=$DIRECTORY/dos_vueltas.bag
killall roscore
roscore &
rosbag play --clock -d 2 $FILENAME &
$PYTHON_INTERPRETER rosbag2pcd.py $DIRECTORY &
$PYTHON_INTERPRETER rosbag2groundtruth.py $DIRECTORY &
