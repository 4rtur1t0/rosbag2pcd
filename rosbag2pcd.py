""""
The script subscribes to a number of topics published in ROS.
The topics may be published by a ROS/Gazebo system or by issuing a rosbag play instance.
The script produces as output data in EUROC format.
"""
import rospy
import argparse
import yaml
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from euroc.eurocsaver import EurocSaver


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
odom_msgs = []
ground_truth_msgs = []
imu_msgs = []
gps_msgs = []
point_cloud_epochs = []


def callback_ground_truth(gt):
    print("ROS GROUND TRUTH received", end='')
    ground_truth_msgs.append(gt)
    return True


def callback_odometry(odometry):
    print("ROS ODOMETRY received", end='')
    odom_msgs.append(odometry)
    return True


def callback_roscloud(ros_cloud):
    """
    Saving pointcloud to file.
    Saving epoch to a list that is saved later.
    """
    point_cloud_epochs.append(str(ros_cloud.header.stamp))
    print("-- Write result point cloud to: ")
    eurocsaver.save_cloud(ros_cloud)
    return True


def callback_gps(gps):
    print("GPS received", end='')
    gps_msgs.append(gps)
    return True


def callback_imu(imu):
    print("IMU received", end='')
    imu_msgs.append(imu)
    return True


def save_data():
    """
    Saving all data to csv files.
    Point clouds are saved in pcd format.
    """
    if len(odom_msgs) > 0:
        eurocsaver.save_odometry(odom_msgs)
    if len(point_cloud_epochs) > 0:
        eurocsaver.save_cloud_times(ros_cloud_epoch_times=point_cloud_epochs)
    if len(ground_truth_msgs) > 0:
        eurocsaver.save_ground_truth(ground_truth_msgs)
    if len(gps_msgs) > 0:
        eurocsaver.save_gps(gps_msgs)
    if len(imu_msgs) > 0:
        eurocsaver.save_imu(imu_msgs)


if __name__ == "__main__":
    """
    Use argv to get topic and path
    """
    with open(r'config.yaml') as file:
        param_list = yaml.load(file, Loader=yaml.FullLoader)
        print(param_list)

    topic_name_ground_truth = param_list.get('topic_name_ground_truth')
    topic_name_point_cloud = param_list.get('topic_name_point_cloud')
    topic_name_odometry = param_list.get('topic_name_odometry')
    topic_name_gps = param_list.get('topic_name_gps')
    topic_name_imu = param_list.get('topic_name_imu')

    print('POINT CLOUD TOPIC: ', topic_name_point_cloud)
    print('ODOMETRY TOPIC: ', topic_name_odometry)
    print('GPS TOPIC: ', topic_name_gps)
    rospy.init_node('ROSBAG2PCD', anonymous=True)
    rospy.Subscriber(topic_name_point_cloud, PointCloud2, callback_roscloud)
    rospy.Subscriber(topic_name_odometry, Odometry, callback_odometry)
    rospy.Subscriber(topic_name_ground_truth, Odometry, callback_ground_truth)
    rospy.Subscriber(topic_name_gps, NavSatFix, callback_ground_truth)
    rospy.Subscriber(topic_name_imu, Imu, callback_imu)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(5.0)
        topics = rospy.get_published_topics()
        flat_list = [item for sublist in topics for item in sublist]
        if topic_name_point_cloud not in flat_list:
            break

    print('Ok, rosbag play is finished.')
    print('Saving Odometry and other CSVs to files.')
    save_data()
    print('Ended writing to files.')