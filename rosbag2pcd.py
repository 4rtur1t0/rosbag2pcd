import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
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

odom_msgs = []
imu_msgs = []
gps_msgs = []
point_cloud_epochs = []


def callback_odometry(odometry):
    print("ROS ODOMETRY received")
    odom_msgs.append(odometry)
    return True


def callback_roscloud(ros_cloud):
    """
    Saving pointcloud to file.
    Saving epoch to a list that is saved later.
    """
    point_cloud_epochs.append(str(ros_cloud.header.stamp))
    eurocsaver.save_cloud(ros_cloud)
    rospy.loginfo("-- Write result point cloud to: ")
    return True


def save_data():
    """
    Saving odometry and point cloud epochs to csv file.
    """
    if len(odom_msgs) > 0:
        eurocsaver.save_odometry(odom_msgs)
    if len(point_cloud_epochs) > 0:
        eurocsaver.save_cloud_times(ros_cloud_epoch_times=point_cloud_epochs)


if __name__ == "__main__":
    """
    Use argv to get topic and path
    """
    with open(r'config.yaml') as file:
        param_list = yaml.load(file, Loader=yaml.FullLoader)
        print(param_list)

    # topic_name_point_cloud = "/points"
    # topic_name_odometry = "/odometry/filtered"
    topic_name_point_cloud = param_list.get('topic_name_point_cloud')
    topic_name_odometry = param_list.get('topic_name_odometry')
    print('POINT CLOUD TOPIC: ', topic_name_point_cloud)
    print('ODOMETRY TOPIC: ', topic_name_odometry)

    rospy.init_node('ROSBAG2PCD', anonymous=True)
    rospy.Subscriber(topic_name_point_cloud, PointCloud2, callback_roscloud)
    rospy.Subscriber(topic_name_odometry, Odometry, callback_odometry)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(.5)
        topics = rospy.get_published_topics()
        flat_list = [item for sublist in topics for item in sublist]
        if topic_name_point_cloud not in flat_list:
            break

    print('Ok, rosbag play is finished.')
    print('Saving Odometry and other CSVs to files.')
    save_data()
    print('Ended to files.')