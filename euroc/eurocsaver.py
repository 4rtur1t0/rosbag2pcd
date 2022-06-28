import os
import pandas as pd
import open3d as o3d
import numpy as np
import sensor_msgs.point_cloud2 as pc2


class EurocSaver():
    """
    Class that saves FIT information (fit file and video file) to EUROC format.
    """
    def __init__(self, euroc_directory=None):
        self.euroc_directory = euroc_directory
        self.odometry_directory = euroc_directory + '/robot0/odom'
        self.imu_directory = euroc_directory + '/robot0/imu0'
        self.gps_directory = euroc_directory + '/robot0/gps0'
        self.lidar_directory = euroc_directory + '/robot0/lidar'
        self.ground_truth_directory = euroc_directory + '/robot0/ground_truth'

        try:
            os.makedirs(self.lidar_directory + '/data')
        except OSError:
            print("Directory exists or creation failed", self.lidar_directory)
        try:
            os.makedirs(self.odometry_directory)
        except OSError:
            print("Directory exists or creation failed", self.odometry_directory)
        try:
            os.makedirs(self.gps_directory)
        except OSError:
            print("Directory exists or creation failed", self.gps_directory)
        try:
            os.makedirs(self.ground_truth_directory)
        except OSError:
            print("Directory exists or creation failed", self.ground_truth_directory)

    # def save_gps(self, gps_data):
    #     lat_list = []
    #     lng_list = []
    #     altitude_list = []
    #     epoch_list = []
    #     for i in range(0, len(gps_data.data_list), 1):
    #         print('Completed: ', 100.0 * i / len(gps_data.data_list), '%', end='\r')
    #         tiempo = int(gps_data.epoch_list[i]*1000*1000*1000)
    #         s1 = f'{tiempo:013d}'
    #         epoch_list.append(s1)
    #         lat_list.append(gps_data.data_list[i].lat)
    #         lng_list.append(gps_data.data_list[i].lng)
    #         altitude_list.append(gps_data.data_list[i].altitude)
    #
    #     raw_data = {'timestamp': epoch_list,
    #                 'lat': lat_list,
    #                 'lng': lng_list,
    #                 'altitude': altitude_list}
    #     df = pd.DataFrame(raw_data, columns=['timestamp', 'lat', 'lng', 'altitude'])
    #     df.to_csv(self.gps_directory + '/data.csv', index=False, header=['#timestamp [ns]', 'lat', 'lng', 'altitude'])
    #     print('\n---')

    def save_odometry(self, odom_msgs):
        """
        Save a list of odometry msgs to csv in EUROC format.
        :param odom_msgs:
        :return:
        """
        i = 0
        epoch_list = []
        # list of xyz positions and quaternions
        xyz_list = []
        q_list = []

        for odo_msg in odom_msgs:
            print('Completed: ', 100.0 * i / len(odom_msgs), '%', end='\r')
            time_str = str(odo_msg.header.stamp)
            # Odometry.
            epoch_list.append(time_str)
            xyz_list.append([odo_msg.pose.pose.position.x, odo_msg.pose.pose.position.y, odo_msg.pose.pose.position.z])
            q_list.append([odo_msg.pose.pose.orientation.x, odo_msg.pose.pose.orientation.y, odo_msg.pose.pose.orientation.z,
                           odo_msg.pose.pose.orientation.w])
            i += 1
        xyz_list = np.array(xyz_list)
        q_list = np.array(q_list)
        raw_data = {'timestamp': epoch_list,
                    'x': xyz_list[:, 0],
                    'y': xyz_list[:, 1],
                    'z': xyz_list[:, 2],
                    'qx': q_list[:, 0],
                    'qy': q_list[:, 1],
                    'qz': q_list[:, 2],
                    'qw': q_list[:, 3]
                    }
        df = pd.DataFrame(raw_data, columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        df.to_csv(self.odometry_directory + '/data.csv', index=False, header=['#timestamp [ns]',
                                                                                       'x', 'y', 'z',
                                                                                       'qx', 'qy', 'qz', 'qw'])
        print('\n---')

    def save_cloud(self, ros_cloud):
        print("ROS POINT CLOUD received")
        time_str = str(ros_cloud.header.stamp)
        # Get cloud data from ros_cloud
        field_names = [field.name for field in ros_cloud.fields]
        points = list(pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names))
        if len(points) == 0:
            print("Converting an empty cloud")
            return None
        pcd_array = np.asarray(points)
        pcd_array = pcd_array[:, 0:3]
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(pcd_array)

        output_directory = self.lidar_directory + '/data/'
        output_filename = output_directory + time_str + ".pcd"

        o3d.io.write_point_cloud(output_filename, pointcloud)
        print('Wrote PCD: ', output_filename)
        # o3d.visualization.draw_geometries([pointcloud])

    def save_cloud_times(self, ros_cloud_epoch_times):
        raw_data = {'timestamp': ros_cloud_epoch_times}
        df = pd.DataFrame(raw_data, columns=['timestamp'])
        df.to_csv(self.lidar_directory + '/data.csv', index=False, header=['#timestamp [ns]'])
        print('\n---')

    def save_ground_truth(self, ground_truth_msgs):
        """
        Save a list of odometry msgs to csv in EUROC format.
        :param odom_msgs:
        :return:
        """
        i = 0
        epoch_list = []
        # list of xyz positions and quaternions
        xyz_list = []
        q_list = []

        for odo_msg in ground_truth_msgs:
            print('Completed: ', 100.0 * i / len(ground_truth_msgs), '%', end='\r')
            time_str = str(odo_msg.header.stamp)
            # Odometry.
            epoch_list.append(time_str)
            xyz_list.append([odo_msg.pose.pose.position.x, odo_msg.pose.pose.position.y, odo_msg.pose.pose.position.z])
            q_list.append([odo_msg.pose.pose.orientation.x, odo_msg.pose.pose.orientation.y, odo_msg.pose.pose.orientation.z,
                           odo_msg.pose.pose.orientation.w])
            i += 1
        xyz_list = np.array(xyz_list)
        q_list = np.array(q_list)
        raw_data = {'timestamp': epoch_list,
                    'x': xyz_list[:, 0],
                    'y': xyz_list[:, 1],
                    'z': xyz_list[:, 2],
                    'qx': q_list[:, 0],
                    'qy': q_list[:, 1],
                    'qz': q_list[:, 2],
                    'qw': q_list[:, 3]
                    }
        df = pd.DataFrame(raw_data, columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        df.to_csv(self.ground_truth_directory + '/data.csv', index=False, header=['#timestamp [ns]',
                                                                                  'x', 'y', 'z',
                                                                                  'qx', 'qy', 'qz', 'qw'])
        print('\n---')

    # def save_gps(self, gps_msgs):
    #     """
    #     Save a list of gps
    #     :param odom_msgs:
    #     :return:
    #     """
    #     i = 0
    #     epoch_list = []
    #     # list of xyz positions and quaternions
    #     xyz_list = []
    #     q_list = []
    #
    #     for odo_msg in odom_msgs:
    #         print('Completed: ', 100.0 * i / len(odom_msgs), '%', end='\r')
    #         time_str = str(odo_msg.header.stamp)
    #         # Odometry.
    #         epoch_list.append(time_str)
    #         xyz_list.append([odo_msg.pose.pose.position.x, odo_msg.pose.pose.position.y, odo_msg.pose.pose.position.z])
    #         q_list.append([odo_msg.pose.pose.orientation.x, odo_msg.pose.pose.orientation.y, odo_msg.pose.pose.orientation.z,
    #                        odo_msg.pose.pose.orientation.w])
    #         i += 1
    #     xyz_list = np.array(xyz_list)
    #     q_list = np.array(q_list)
    #     raw_data = {'timestamp': epoch_list,
    #                 'x': xyz_list[:, 0],
    #                 'y': xyz_list[:, 1],
    #                 'z': xyz_list[:, 2],
    #                 'qx': q_list[:, 0],
    #                 'qy': q_list[:, 1],
    #                 'qz': q_list[:, 2],
    #                 'qw': q_list[:, 3]
    #                 }
    #     df = pd.DataFrame(raw_data, columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    #     df.to_csv(self.gps_directory + '/data.csv', index=False, header=['#timestamp [ns]',
    #                                                                                    'x', 'y', 'z',
    #                                                                                    'qx', 'qy', 'qz', 'qw'])
    #     print('\n---')

    def save_gps(self, gps_msgs):
        """
        Save a list of gps
        :param gps_msgs:
        :return:
        """
        i = 0
        epoch_list = []
        # list of xyz positions and quaternions
        gps_coords_list = []
        covariance_list = []

        for gps_msg in gps_msgs:
            print('Completed: ', 100.0 * i / len(gps_msgs), '%', end='\r')
            time_str = str(gps_msg.header.stamp)
            # GPS coords.
            epoch_list.append(time_str)
            gps_coords_list.append([gps_msg.latitude, gps_msg.longitude, gps_msg.altitude])
            covariance = np.reshape(gps_msg.position_covariance, (3, 3))
            covariance_list.append(np.diag(covariance))
            i += 1
        gps_coords_list = np.array(gps_coords_list)
        covariance_list = np.array(covariance_list)
        raw_data = {'timestamp': epoch_list,
                    'latitude': gps_coords_list[:, 0],
                    'longitude': gps_coords_list[:, 1],
                    'altitude': gps_coords_list[:, 2],
                    'covariance_d1': covariance_list[:, 0],
                    'covariance_d2': covariance_list[:, 1],
                    'covariance_d3': covariance_list[:, 2]
                    }
        df = pd.DataFrame(raw_data,
                          columns=['timestamp', 'latitude', 'longitude', 'altitude', 'covariance_d1', 'covariance_d2',
                                   'covariance_d3'])
        df.to_csv(self.gps_directory + '/data.csv', index=False,
                  header=['#timestamp [ns]', 'latitude', 'longitude', 'altitude',
                          'covariance_d1', 'covariance_d2', 'covariance_d3'])
        print('\n---')

    def save_imu(self, imu_msgs):
        i = 0
        epoch_list = []
        # list quaternions and covariance orientation
        q_list = []
        covariance_list = []

        for imu_msg in imu_msgs:
            print('Completed: ', 100.0 * i / len(imu_msgs), '%', end='\r')
            time_str = str(imu_msg.header.stamp)
            # Orientation.
            epoch_list.append(time_str)
            q_list.append([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])
            covariance = np.reshape(imu_msg.orientation_covariance, (3, 3))
            covariance_list.append(np.diag(covariance))
            i += 1
        q_list = np.array(q_list)
        covariance_list = np.array(covariance_list)
        raw_data = {'timestamp': epoch_list,
                    'qx': q_list[:, 0],
                    'qy': q_list[:, 1],
                    'qz': q_list[:, 2],
                    'qw': q_list[:, 3],
                    'covariance_d1': covariance_list[:, 0],
                    'covariance_d2': covariance_list[:, 1],
                    'covariance_d3': covariance_list[:, 2]
                    }
        df = pd.DataFrame(raw_data, columns=['timestamp', 'qx', 'qy', 'qz', 'qw', 'covariance_d1', 'covariance_d2',
                                             'covariance_d3'])
        df.to_csv(self.imu_directory + '/data.csv', index=False, header=['#timestamp [ns]',
                                                                         'qx', 'qy', 'qz', 'qw',
                                                                         'covariance_d1', 'covariance_d2',
                                                                         'covariance_d3'])
        print('\n---')