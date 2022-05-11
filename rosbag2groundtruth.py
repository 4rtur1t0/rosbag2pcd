import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
# from tf2_msgs.msg import TFMessage
# from tf2_msgs.msg import TFMessage
from gazebo_msgs.msg import LinkStates
# from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from euroc.eurocsaver import EurocSaver
import argparse
import yaml

# Parse output directory
parser = argparse.ArgumentParser()
parser.add_argument("directory")
args = parser.parse_args()
print("OUTPUT DIRECTORY IS. ")
print(args.directory)
EXPERIMENT_PATH = args.directory

# create EurocSaver object with output directory
eurocsaver = EurocSaver(euroc_directory=EXPERIMENT_PATH)

# lists of messages
# odom_msgs = []
ground_truth_msgs = []
ground_truth_msgs_times = []
last_clock_received = 0

with open(r'config.yaml') as file:
    param_list = yaml.load(file, Loader=yaml.FullLoader)
    print(param_list)
GROUND_TRUTH_BASE_LINK = param_list.get('ground_truth_base_link')


# def callback_clock(clock):
#     """
#     saving last clock
#     """
#     global last_clock_received
#     print(".", end='')
#     last_clock_received = clock
#     return True


def callback_ground_truth(model_states):
    # global last_clock_received
    global GROUND_TRUTH_BASE_LINK
    # if last_clock_received == 0:
    #     return

    now = rospy.get_rostime()
    rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

    idx = 0
    for i in range(len(model_states.name)):
        if model_states.name[i] == GROUND_TRUTH_BASE_LINK:
            idx = i
            break
    print("ROS GROUND TRUTH received", end='')
    ground_truth_msgs.append(model_states.pose[idx])
    # ground_truth_msgs_times.append(last_clock_received)
    ground_truth_msgs_times.append(now)
    # if len(ground_truth_msgs) == 15000:
    #     save_data()
    return True


def save_data():
    """
    Saving data to csvs
    """
    if len(ground_truth_msgs) > 0:
        eurocsaver.save_ground_truth(ground_truth_msgs, ground_truth_msgs_times)


if __name__ == "__main__":
    """
    Use argv to get topic and path
    """
    with open(r'config.yaml') as file:
        param_list = yaml.load(file, Loader=yaml.FullLoader)
        print(param_list)
    topic_name_ground_truth = param_list.get('topic_name_ground_truth')
    print('GROUND TRUTH TOPIC: ', topic_name_ground_truth)

    rospy.init_node('ROSBAG2GROUNDTRUTH', anonymous=True)
    # rospy.Subscriber('/clock', Clock, callback_clock)
    rospy.Subscriber(topic_name_ground_truth, LinkStates, callback_ground_truth)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(5.0)
        topics = rospy.get_published_topics()
        flat_list = [item for sublist in topics for item in sublist]
        if topic_name_ground_truth not in flat_list:
            break

    print('Ok, rosbag play is finished.')
    print('Saving GROUND TRUTH CSVs to files.')
    save_data()
    print('Ended writing to files.')