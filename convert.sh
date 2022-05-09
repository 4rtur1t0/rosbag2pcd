PYTHON_INTERPRETER=/home/arvc/Applications/venv/bin/python
DATASETS='/media/arvc/INTENSO/DATASETS'
killall roscore
roscore &
rosbag play -d 2 $DATASETS/husky_playpen_1loop/husky_playpen_1loop.bag &
$PYTHON_INTERPRETER rosbag2pcd.py $DATASETS/husky_playpen_1loop
