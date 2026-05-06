import matplotlib.pyplot as plt
import numpy as np
import rclpy
import rclpy.serialization
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu


class Bag_Analyzer():
    def __init__(self, path, topic_dict):
        self.path = path
        self.topic_dict = topic_dict
        self.topic_list = list(topic_dict.keys())
        self.data = self.setup_data_fields()

    def read_rosbag2_mcap(self):
        storage_options = StorageOptions(uri=self.path, storage_id='mcap')
        converter_options = ConverterOptions('', '')  # Use default serialization

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        # Filter by topic
        reader.set_filter(StorageFilter(topics=self.topic_list))

        while reader.has_next():
            (topic, topic_data, timestamp) = reader.read_next()
            if topic in self.topic_list:
                timestamp_seconds = timestamp * 1e-9
                self.update_data(topic, topic_data, timestamp_seconds)

    def setup_data_fields(self):
        data = {}
        for topic in self.topic_dict:
            data[topic] = {}
            data[topic]['timestamps']=[]
            if self.topic_dict[topic]['plot'][0]:
                data[topic]['x'] = []
            if self.topic_dict[topic]['plot'][1]:
                data[topic]['y'] = []
            if self.topic_dict[topic]['plot'][2]:
                data[topic]['z'] = []
            if self.topic_dict[topic]['plot'][3]:
                data[topic]['roll'] = []
            if self.topic_dict[topic]['plot'][4]:
                data[topic]['pitch'] = []
            if self.topic_dict[topic]['plot'][5]:
                data[topic]['yaw'] = []
            if self.topic_dict[topic]['plot'][6]:
                data[topic]['vel_x'] = []
            if self.topic_dict[topic]['plot'][7]:
                data[topic]['vel_y'] = []
            if self.topic_dict[topic]['plot'][8]:
                data[topic]['vel_z'] = []
            if self.topic_dict[topic]['plot'][9]:
                data[topic]['omega_x'] = []
            if self.topic_dict[topic]['plot'][10]:
                data[topic]['omega_y'] = []
            if self.topic_dict[topic]['plot'][11]:
                data[topic]['omega_z'] = []
            if self.topic_dict[topic]['plot'][12]:
                data[topic]['accel_x'] = []
            if self.topic_dict[topic]['plot'][13]:
                data[topic]['accel_y'] = []
            if self.topic_dict[topic]['plot'][14]:
                data[topic]['accel_z'] = []
        return data


    def update_data(self, topic, topic_data, timestamp_seconds):
                # Each message type has to be read differently :(
                if self.topic_dict[topic]['type'] == 'nav_msgs::msg::Odometry':
                    # this outputs in odometry messages
                    try:
                        msg = rclpy.serialization.deserialize_message(
                            topic_data,
                            Odometry)
                    except Exception as e:
                        print(f'ERROR WITH DESERIALIZATION OF {topic}! {e}')
                    self.data[topic]['timestamps'].append(timestamp_seconds)
                    if self.topic_dict[topic]['plot'][0]:
                        x = msg.pose.pose.position.x
                        self.data[topic]['x'].append(x)
                    if self.topic_dict[topic]['plot'][1]:
                        y = msg.pose.pose.position.y
                        self.data[topic]['y'].append(y)
                    if self.topic_dict[topic]['plot'][2]:
                        z = msg.pose.pose.position.z
                        self.data[topic]['z'].append(z)
                    qx = msg.pose.pose.orientation.x
                    #self.data[topic]['qx'].append(qx)
                    qy = msg.pose.pose.orientation.y
                    #self.data[topic]['qy'].append(qy)
                    qz = msg.pose.pose.orientation.z
                    #self.data[topic]['qz'].append(qz)
                    qw = msg.pose.pose.orientation.w
                    #self.data[topic]['qw'].append(qw)
                    eulers = R.from_quat([qx, qy, qz, qw]).as_euler('xyz', degrees=True)
                    if self.topic_dict[topic]['plot'][3]:
                        self.data[topic]['roll'].append(eulers[0])
                    if self.topic_dict[topic]['plot'][4]:
                        self.data[topic]['pitch'].append(eulers[1])
                    if self.topic_dict[topic]['plot'][5]:
                        self.data[topic]['yaw'].append(eulers[2])
                    if self.topic_dict[topic]['plot'][6]:
                        vel_x = msg.twist.twist.linear.x
                        self.data[topic]['vel_x'].append(vel_x)
                    if self.topic_dict[topic]['plot'][7]:
                        vel_y = msg.twist.twist.linear.y
                        self.data[topic]['vel_y'].append(vel_y)
                    if self.topic_dict[topic]['plot'][8]:
                        vel_z = msg.twist.twist.linear.z
                        self.data[topic]['vel_z'].append(vel_z)
                    if self.topic_dict[topic]['plot'][9]:
                        omega_x = msg.twist.twist.angular.x
                        self.data[topic]['omega_x'].append(omega_x)
                    if self.topic_dict[topic]['plot'][10]:
                        omega_y = msg.twist.twist.angular.y
                        self.data[topic]['omega_y'].append(omega_y)
                    if self.topic_dict[topic]['plot'][11]:
                        omega_z = msg.twist.twist.angular.z
                        self.data[topic]['omega_z'].append(omega_z)

                elif self.topic_dict[topic]['type'] == 'sensor_msgs::msg::Imu':
                    # this outputs in Imu Messages
                    try:
                        msg = rclpy.serialization.deserialize_message(topic_data, Imu)
                    except Exception as e:
                        print(f'ERROR WITH DESERIALIZATION OF {topic}! {e}')
                    self.data[topic]['timestamps'].append(timestamp_seconds)
                    qx = msg.orientation.x
                    #self.data[topic]['qx'].append(qx)
                    qy = msg.orientation.y
                    #self.data[topic]['qy'].append(qy)
                    qz = msg.orientation.z
                    #self.data[topic]['qz'].append(qz)
                    qw = msg.orientation.w
                    #self.data[topic]['qw'].append(qw)
                    eulers = R.from_quat([qx, qy, qz, qw]).as_euler('xyz', degrees=True)
                    if self.topic_dict[topic]['plot'][3]:
                        self.data[topic]['roll'].append(eulers[0])
                    if self.topic_dict[topic]['plot'][4]:
                        self.data[topic]['pitch'].append(eulers[1])
                    if self.topic_dict[topic]['plot'][5]:
                        self.data[topic]['yaw'].append(eulers[2])
                    if self.topic_dict[topic]['plot'][9]:
                        omega_x = msg.angular_velocity.x
                        self.data[topic]['omega_x'].append(omega_x)
                    if self.topic_dict[topic]['plot'][10]:
                        omega_y = msg.angular_velocity.y
                        self.data[topic]['omega_y'].append(omega_y)
                    if self.topic_dict[topic]['plot'][11]:
                        omega_z = msg.angular_velocity.z
                        self.data[topic]['omega_z'].append(omega_z)
                    if self.topic_dict[topic]['plot'][12]:
                        accel_x = msg.linear_acceleration.x
                        self.data[topic]['accel_x'].append(accel_x)
                    if self.topic_dict[topic]['plot'][13]:
                        accel_y = msg.linear_acceleration.y
                        self.data[topic]['accel_y'].append(accel_y)
                    if self.topic_dict[topic]['plot'][14]:
                        accel_z = msg.linear_acceleration.z
                        self.data[topic]['accel_z'].append(accel_z)
                elif self.topic_dict[topic]['type'] == (
                        'geometry_msgs::msg::TwistWithCovarianceStamped'):
                    # dvl velocity is a twist message
                    try:
                        msg = rclpy.serialization.deserialize_message(
                            topic_data,
                            TwistWithCovarianceStamped)
                    except Exception as e:
                        print(f'ERROR WITH DESERIALIZATION OF {topic}! {e}')
                    self.data[topic]['timestamps'].append(timestamp_seconds)
                    if self.topic_dict[topic]['plot'][6]:
                        x = msg.twist.twist.linear.x
                        self.data[topic]['vel_x'].append(x)
                    if self.topic_dict[topic]['plot'][7]:
                        y = msg.twist.twist.linear.y
                        self.data[topic]['vel_y'].append(y)
                    if self.topic_dict[topic]['plot'][8]:
                        z = msg.twist.twist.linear.z
                        self.data[topic]['vel_z'].append(z)
                    if self.topic_dict[topic]['plot'][9]:
                        omega_x = msg.twist.twist.angular.x
                        self.data[topic]['omega_x'].append(omega_x)
                    if self.topic_dict[topic]['plot'][10]:
                        omega_y = msg.twist.twist.angular.y
                        self.data[topic]['omega_y'].append(omega_y)
                    if self.topic_dict[topic]['plot'][11]:
                        omega_z = msg.twist.twist.angular.z
                        self.data[topic]['omega_z'].append(omega_z)
                elif self.topic_dict[topic]['type'] == (
                        'geometry_msgs::msg::PoseWithCovarianceStamped'):
                    try:
                        msg = rclpy.serialization.deserialize_message(
                            topic_data,
                            PoseWithCovarianceStamped)
                    except Exception as e:
                        print(f'ERROR WITH DESERIALIZATION OF {topic}! {e}')
                    self.data[topic]['timestamps'].append(timestamp_seconds)
                    if self.topic_dict[topic]['plot'][0]:
                        self.data[topic]['x'].append(msg.pose.pose.position.x)
                    if self.topic_dict[topic]['plot'][1]:
                        self.data[topic]['y'].append(msg.pose.pose.position.y)
                    if self.topic_dict[topic]['plot'][2]:
                        self.data[topic]['z'].append(msg.pose.pose.position.z)
                    qx = msg.pose.pose.orientation.x
                    #self.data[topic]['qx'].append(qx)
                    qy = msg.pose.pose.orientation.y
                    #self.data[topic]['qy'].append(qy)
                    qz = msg.pose.pose.orientation.z
                    #self.data[topic]['qz'].append(qz)
                    qw = msg.pose.pose.orientation.w
                    #self.data[topic]['qw'].append(qw)
                    eulers = R.from_quat([qx, qy, qz, qw]).as_euler(
                        'xyz',
                        degrees=True)
                    if self.topic_dict[topic]['plot'][3]:
                        self.data[topic]['roll'].append(eulers[0])
                    if self.topic_dict[topic]['plot'][4]:
                        self.data[topic]['pitch'].append(eulers[1])
                    if self.topic_dict[topic]['plot'][5]:
                        self.data[topic]['yaw'].append(eulers[2])

                else:
                    print('WARNING: TOPIC TYPE NOT SUPPORTED CURRENTLY. MAY '
                          'NEED TO ADD NEW TYPE.')

    def plot_data(self):
        #Before plotting, actually read the data
        self.read_rosbag2_mcap()

        #initial step, get start time:
        start_times = []
        for topic in self.data:
            if self.data[topic]['timestamps']:
                start_times.append(min(self.data[topic]['timestamps']))

        if start_times:
            time0 = min(start_times)
            print(f'Program start time: {time0} seconds')
        else:
            print('No timestamp data found')

        # First plot. Pose:
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Position x,y,z for all topics')
        pos_variables = ['x', 'y', 'z']

        for i, var in enumerate(pos_variables):
            for topic in self.topic_list:
                if self.topic_dict[topic]['plot'][i]:
                    axes[i].plot(
                        np.array(self.data[topic]['timestamps']) - time0,
                        self.data[topic][var],
                        label=f'{topic}')
            axes[i].set_xlabel('Time [s]')
            axes[i].set_ylabel(f'{var} [m]')
            axes[i].set_title(f'Position: {var}')
            axes[i].grid(True)
            axes[i].legend()
        plt.tight_layout()

        # Second Plot Group. Orientation:
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Orientation rpy for all topics')
        orient_variables = ['roll', 'pitch', 'yaw']

        for i, var in enumerate(orient_variables):
            for topic in self.topic_list:
                if self.topic_dict[topic]['plot'][i + 3]:
                    axes[i].plot(
                        np.array(self.data[topic]['timestamps']) - time0,
                        self.data[topic][var],
                        label=f'{topic}')
            axes[i].set_xlabel('Time [s]')
            axes[i].set_ylabel(f'{var} [deg]')
            axes[i].set_title(f'Orientation: {var}')
            axes[i].grid(True)
            axes[i].legend()
        plt.tight_layout()

        # Third Plot Group: Angular Velocity
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Angular Velocity for all topics')
        ang_vel_variables = ['omega_x', 'omega_y', 'omega_z']

        for i, var in enumerate(ang_vel_variables):
            for topic in self.topic_list:
                if self.topic_dict[topic]['plot'][i+9]:
                    try:
                        axes[i].plot(
                            np.array(self.data[topic]['timestamps']) - time0,
                            self.data[topic][var],
                            label=(f'{topic}, avg value: '
                                   f'{np.mean(self.data[topic][var])}'),
                        )
                    except Exception as e:
                        print(f'message type {topic} had issue in reading: {e}')
                        break
            axes[i].set_xlabel('Time [s]')
            axes[i].set_ylabel(f'{var} [rad/s]')
            axes[i].set_title(f'Angular Velocity: {var}')
            axes[i].grid(True)
            axes[i].legend()
        plt.tight_layout()

        # Fourth Plot Group: Linear Velocity
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Linear Velocity for all topics')
        lin_vel_variables = ['vel_x', 'vel_y', 'vel_z']

        for i, var in enumerate(lin_vel_variables):
            for topic in self.topic_list:
                if self.topic_dict[topic]['plot'][i+6]:
                    axes[i].plot(
                        np.array(self.data[topic]['timestamps']) - time0,
                        self.data[topic][var],
                        label=f'{topic}',
                    )
            axes[i].set_xlabel('Time [s]')
            axes[i].set_ylabel(f'{var} [m/s]')
            axes[i].set_title(f'Linear Velocity: {var}')
            axes[i].grid(True)
            axes[i].legend()
        plt.tight_layout()

        # Fifth Plot Group: Acceleration
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Acceleration for all topics')
        accel_variables = ['accel_x', 'accel_y', 'accel_z']

        for i, var in enumerate(accel_variables):
            for topic in self.topic_list:
                if self.topic_dict[topic]['plot'][i+12]:
                    axes[i].plot(
                        np.array(self.data[topic]['timestamps']) - time0,
                        self.data[topic][var],
                        label=f'{topic}, avg value: {np.mean(self.data[topic][var])}',
                    )
            axes[i].set_xlabel('Time [s]')
            axes[i].set_ylabel(f'{var} [m/s^2]')
            axes[i].set_title(f'Acceleration: {var}')
            axes[i].grid(True)
            axes[i].legend()

        splits_x = np.array_split(
            self.data['/vectornav/Imu_body']['accel_x'],
            10)
        averages_x = [np.mean(segment) for segment in splits_x]
        print(f'Average for each of the 10 segments of x:\n '
              f'{np.round(averages_x,6)}\n\n')

        splits_y = np.array_split(
            self.data['/vectornav/Imu_body']['accel_y'],
            10)
        averages_y = [np.mean(segment) for segment in splits_y]
        print(f'Average for each of the 10 segments of y:\n '
              f'{np.round(averages_y,6)}\n\n')

        splits_z = np.array_split(
            self.data['/vectornav/Imu_body']['accel_z'],
            10)
        averages_z = [np.mean(segment) for segment in splits_z]
        print(f'Average for each of the 10 segments of z:\n '
              f'{np.round(averages_z,6) }\n\n')

        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    rclpy.init()

    # Assuming that the
    bag_path = input('Type the full path from the current Directory to the bag')

    # Specify the topics to read and the data from those topics to plot.
    # Plot field is in the form of the robot_localization ekf specification:
    #                 [x_pos, y_pos, z_pos,
    #                 roll, pitch, yaw,
    #                 x_vel, y_vel, z_vel,
    #                 roll_vel, pitch_vel, yaw_vel
    #                 x_accel, y_accel, z_accel]
    # Where true plots that variable, and false does not

    topic_dictionary = {
        '/odometry/filtered':
            {'type': 'nav_msgs::msg::Odometry',
             'plot': [True, True, True,
                      True, True, True,
                      True, True, True,
                      True, True, True,
                      False, False, False]},

        '/mavros/local_position/odom':
            {'type': 'nav_msgs::msg::Odometry',
             'plot': [True, True, True,
                      True, True, True,
                      True, True, True,
                      True, True, True,
                      False, False, False]},

        '/dvl/twist_data':
            {'type': 'geometry_msgs::msg::TwistWithCovarianceStamped',
             'plot': [False, False, False,
                      False, False, False,
                      True, True, True,
                      False, False, False,
                      False, False, False]},

        '/vectornav/Imu_body':
            {'type': 'sensor_msgs::msg::Imu',
             'plot': [False, False, False,
                      True, True, True,
                      False, False, False,
                      True, True, True,
                      True, True, True]},

        '/vectornav/filterred_orientation':
            {'type': 'geometry_msgs::msg::PoseWithCovarianceStamped',
             'plot': [False, False, False,
                      True, True, True,
                      False, False, False,
                      False, False, False,
                      False, False, False]},

        '/mavros/vision_pose/pose_cov':
            {'type': 'geometry_msgs::msg::PoseWithCovarianceStamped',
             'plot': [True, True, True,
                      True, True, True,
                      False, False, False,
                      False, False, False,
                      False, False, False]}
        }

    #initialize analyzer and plot data
    analyzer = Bag_Analyzer(bag_path, topic_dictionary)
    analyzer.plot_data()

    rclpy.shutdown()
