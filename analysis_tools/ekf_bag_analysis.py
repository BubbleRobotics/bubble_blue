import matplotlib.pyplot as plt
import rclpy
import rclpy.serialization
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu


def read_rosbag2_mcap(path, topics_to_read):
    storage_options = StorageOptions(uri=path, storage_id='mcap')
    converter_options = ConverterOptions('', '')  # Use default serialization

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # topic_types = reader.get_all_topics_and_types()
    # type_dict = {t.name: t.type for t in topic_types}

    # Filter by topic
    reader.set_filter(StorageFilter(topics=topics_to_read))

    # setting up the data dictionary for plotting everything
    data = {
        '/odometry/filtered': {
            'timestamps': [],
            'x': [],
            'y': [],
            'z': [],
            'qx': [],
            'qy': [],
            'qz': [],
            'qw': [],
            'roll': [],
            'pitch': [],
            'yaw': [],
            'vel_x': [],
            'vel_y': [],
            'vel_z': [],
            'omega_x': [],
            'omega_y': [],
            'omega_z': [],
        },
        '/model/bluerov2/odometry': {
            'timestamps': [],
            'x': [],
            'y': [],
            'z': [],
            'qx': [],
            'qy': [],
            'qz': [],
            'qw': [],
            'roll': [],
            'pitch': [],
            'yaw': [],
            'vel_x': [],
            'vel_y': [],
            'vel_z': [],
            'omega_x': [],
            'omega_y': [],
            'omega_z': [],
        },
        '/mavros/local_position/odom': {
            'timestamps': [],
            'x': [],
            'y': [],
            'z': [],
            'qx': [],
            'qy': [],
            'qz': [],
            'qw': [],
            'roll': [],
            'pitch': [],
            'yaw': [],
            'vel_x': [],
            'vel_y': [],
            'vel_z': [],
            'omega_x': [],
            'omega_y': [],
            'omega_z': [],
        },
        '/vn100/gz_data': {
            'timestamps': [],
            'qx': [],
            'qy': [],
            'qz': [],
            'qw': [],
            'roll': [],
            'pitch': [],
            'yaw': [],
            'accel_x': [],
            'accel_y': [],
            'accel_z': [],
            'omega_x': [],
            'omega_y': [],
            'omega_z': [],
        },
        '/dvl/velocity': {'timestamps': [], 'vel_x': [], 'vel_y': [], 'vel_z': []},
    }

    while reader.has_next():
        (topic, topic_data, timestamp) = reader.read_next()
        if topic in topics_to_read:
            timestamp_seconds = timestamp * 1e-9

            # Each message type has to be read differently :(
            if topic == '/odometry/filtered':
                # this outputs in odometry messages
                msg = rclpy.serialization.deserialize_message(topic_data, Odometry)
                data[topic]['timestamps'].append(timestamp_seconds)
                x = msg.pose.pose.position.x
                data[topic]['x'].append(x)
                y = msg.pose.pose.position.y
                data[topic]['y'].append(y)
                z = msg.pose.pose.position.z
                data[topic]['z'].append(z)
                qx = msg.pose.pose.orientation.x
                data[topic]['qx'].append(qx)
                qy = msg.pose.pose.orientation.y
                data[topic]['qy'].append(qy)
                qz = msg.pose.pose.orientation.z
                data[topic]['qz'].append(qz)
                qw = msg.pose.pose.orientation.w
                data[topic]['qw'].append(qw)
                eulers = R.from_quat([qx, qy, qz, qw]).as_euler('xyz', degrees=True)
                data[topic]['roll'].append(eulers[0])
                data[topic]['pitch'].append(eulers[1])
                data[topic]['yaw'].append(eulers[2])
                vel_x = msg.twist.twist.linear.x
                data[topic]['vel_x'].append(vel_x)
                vel_y = msg.twist.twist.linear.y
                data[topic]['vel_y'].append(vel_y)
                vel_z = msg.twist.twist.linear.z
                data[topic]['vel_z'].append(vel_z)
                omega_x = msg.twist.twist.angular.x
                data[topic]['omega_x'].append(omega_x)
                omega_y = msg.twist.twist.angular.y
                data[topic]['omega_y'].append(omega_y)
                omega_z = msg.twist.twist.angular.z
                data[topic]['omega_z'].append(omega_z)
            if topic == '/model/bluerov2/odometry':
                # this outputs in odometry messages
                msg = rclpy.serialization.deserialize_message(topic_data, Odometry)
                data[topic]['timestamps'].append(timestamp_seconds)
                x = msg.pose.pose.position.x
                data[topic]['x'].append(x)
                y = msg.pose.pose.position.y
                data[topic]['y'].append(y)
                z = msg.pose.pose.position.z
                data[topic]['z'].append(z)
                qx = msg.pose.pose.orientation.x
                data[topic]['qx'].append(qx)
                qy = msg.pose.pose.orientation.y
                data[topic]['qy'].append(qy)
                qz = msg.pose.pose.orientation.z
                data[topic]['qz'].append(qz)
                qw = msg.pose.pose.orientation.w
                data[topic]['qw'].append(qw)
                eulers = R.from_quat([qx, qy, qz, qw]).as_euler('xyz', degrees=True)
                data[topic]['roll'].append(eulers[0])
                data[topic]['pitch'].append(eulers[1])
                data[topic]['yaw'].append(eulers[2])
                vel_x = msg.twist.twist.linear.x
                data[topic]['vel_x'].append(vel_x)
                vel_y = msg.twist.twist.linear.y
                data[topic]['vel_y'].append(vel_y)
                vel_z = msg.twist.twist.linear.z
                data[topic]['vel_z'].append(vel_z)
                omega_x = msg.twist.twist.angular.x
                data[topic]['omega_x'].append(omega_x)
                omega_y = msg.twist.twist.angular.y
                data[topic]['omega_y'].append(omega_y)
                omega_z = msg.twist.twist.angular.z
                data[topic]['omega_z'].append(omega_z)
            if topic == '/mavros/local_position/odom':
                # this outputs in odometry messages
                msg = rclpy.serialization.deserialize_message(topic_data, Odometry)
                data[topic]['timestamps'].append(timestamp_seconds)
                x = msg.pose.pose.position.x
                data[topic]['x'].append(x)
                y = msg.pose.pose.position.y
                data[topic]['y'].append(y)
                z = msg.pose.pose.position.z
                data[topic]['z'].append(z)
                qx = msg.pose.pose.orientation.x
                data[topic]['qx'].append(qx)
                qy = msg.pose.pose.orientation.y
                data[topic]['qy'].append(qy)
                qz = msg.pose.pose.orientation.z
                data[topic]['qz'].append(qz)
                qw = msg.pose.pose.orientation.w
                data[topic]['qw'].append(qw)
                eulers = R.from_quat([qx, qy, qz, qw]).as_euler('xyz', degrees=True)
                data[topic]['roll'].append(eulers[0])
                data[topic]['pitch'].append(eulers[1])
                data[topic]['yaw'].append(eulers[2])
                vel_x = msg.twist.twist.linear.x
                data[topic]['vel_x'].append(vel_x)
                vel_y = msg.twist.twist.linear.y
                data[topic]['vel_y'].append(vel_y)
                vel_z = msg.twist.twist.linear.z
                data[topic]['vel_z'].append(vel_z)
                omega_x = msg.twist.twist.angular.x
                data[topic]['omega_x'].append(omega_x)
                omega_y = msg.twist.twist.angular.y
                data[topic]['omega_y'].append(omega_y)
                omega_z = msg.twist.twist.angular.z
                data[topic]['omega_z'].append(omega_z)
            if topic == '/vn100/gz_data':
                # this outputs in Imu Messages
                msg = rclpy.serialization.deserialize_message(topic_data, Imu)
                data[topic]['timestamps'].append(timestamp_seconds)
                qx = msg.orientation.x
                data[topic]['qx'].append(qx)
                qy = msg.orientation.y
                data[topic]['qy'].append(qy)
                qz = msg.orientation.z
                data[topic]['qz'].append(qz)
                qw = msg.orientation.w
                data[topic]['qw'].append(qw)
                eulers = R.from_quat([qx, qy, qz, qw]).as_euler('xyz', degrees=True)
                data[topic]['roll'].append(eulers[0])
                data[topic]['pitch'].append(eulers[1])
                data[topic]['yaw'].append(eulers[2])
                omega_x = msg.angular_velocity.x
                data[topic]['omega_x'].append(omega_x)
                omega_y = msg.angular_velocity.y
                data[topic]['omega_y'].append(omega_y)
                omega_z = msg.angular_velocity.z
                data[topic]['omega_z'].append(omega_z)
                accel_x = msg.linear_acceleration.x
                data[topic]['accel_x'].append(accel_x)
                accel_y = msg.linear_acceleration.y
                data[topic]['accel_y'].append(accel_y)
                accel_z = msg.linear_acceleration.z
                data[topic]['accel_z'].append(accel_z)
            if topic == '/dvl/velocity':
                # dvl velocity is a twist message
                msg = rclpy.serialization.deserialize_message(
                    topic_data, TwistWithCovarianceStamped
                )
                data[topic]['timestamps'].append(timestamp_seconds)
                x = msg.twist.twist.linear.x
                data[topic]['vel_x'].append(x)
                y = msg.twist.twist.linear.y
                data[topic]['vel_y'].append(y)
                z = msg.twist.twist.linear.z
                data[topic]['vel_z'].append(z)

    return data


def plot_data(data_dictionary):
    # First plot. Pose:
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Position x,y,z for all topics')
    pos_topics = [
        '/odometry/filtered',
        '/model/bluerov2/odometry',
        '/mavros/local_position/odom',
    ]
    pos_variables = ['x', 'y', 'z']

    for i, var in enumerate(pos_variables):
        for topic in pos_topics:
            axes[i].plot(
                data_dictionary[topic]['timestamps'],
                data_dictionary[topic][var],
                label=f'{topic}',
            )
        axes[i].set_xlabel('Time [s]')
        axes[i].set_ylabel(f'{var} [m]')
        axes[i].set_title(f'Position: {var}')
        axes[i].grid(True)
        axes[i].legend()

    # Second Plot Group. Orientation:
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Orientation rpy for all topics')
    orient_topics = [
        '/odometry/filtered',
        '/model/bluerov2/odometry',
        '/mavros/local_position/odom',
        '/vn100/gz_data',
    ]
    orient_variables = ['roll', 'pitch', 'yaw']

    for i, var in enumerate(orient_variables):
        for topic in orient_topics:
            axes[i].plot(
                data_dictionary[topic]['timestamps'],
                data_dictionary[topic][var],
                label=f'{topic}',
            )
        axes[i].set_xlabel('Time [s]')
        axes[i].set_ylabel(f'{var} [deg]')
        axes[i].set_title(f'Orientation: {var}')
        axes[i].grid(True)
        axes[i].legend()

    # Third Plot Group: Angular Velocity
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Angular Velocity for all topics')
    ang_vel_topics = [
        '/odometry/filtered',
        '/model/bluerov2/odometry',
        '/mavros/local_position/odom',
        '/vn100/gz_data',
    ]
    ang_vel_variables = ['omega_x', 'omega_y', 'omega_z']

    for i, var in enumerate(ang_vel_variables):
        for topic in ang_vel_topics:
            axes[i].plot(
                data_dictionary[topic]['timestamps'],
                data_dictionary[topic][var],
                label=f'{topic}',
            )
        axes[i].set_xlabel('Time [s]')
        axes[i].set_ylabel(f'{var} [rad/s]')
        axes[i].set_title(f'Angular Velocity: {var}')
        axes[i].grid(True)
        axes[i].legend()

    # Fourth Plot Group: Linear Velocity
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Linear Velocity for all topics')
    lin_vel_topics = [
        '/odometry/filtered',
        '/model/bluerov2/odometry',
        '/mavros/local_position/odom',
        '/dvl/velocity',
    ]
    lin_vel_variables = ['vel_x', 'vel_y', 'vel_z']

    for i, var in enumerate(lin_vel_variables):
        for topic in lin_vel_topics:
            axes[i].plot(
                data_dictionary[topic]['timestamps'],
                data_dictionary[topic][var],
                label=f'{topic}',
            )
        axes[i].set_xlabel('Time [s]')
        axes[i].set_ylabel(f'{var} [m/s]')
        axes[i].set_title(f'Linear Velocity: {var}')
        axes[i].grid(True)
        axes[i].legend()

    # Fifth Plot Group: Acceleration
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Acceleration for all topics')
    accel_topics = ['/vn100/gz_data']
    accel_variables = ['accel_x', 'accel_y', 'accel_z']

    for i, var in enumerate(accel_variables):
        for topic in accel_topics:
            axes[i].plot(
                data_dictionary[topic]['timestamps'],
                data_dictionary[topic][var],
                label=f'{topic}',
            )
        axes[i].set_xlabel('Time [s]')
        axes[i].set_ylabel(f'{var} [m/s^2]')
        axes[i].set_title(f'Acceleration: {var}')
        axes[i].grid(True)
        axes[i].legend()

    plt.show()


if __name__ == '__main__':
    rclpy.init()

    # Assuming that the
    bag_path = input('Type the full path from the current Directory to the bag')

    topics_to_read = [
        '/odometry/filtered',
        '/model/bluerov2/odometry',
        '/mavros/local_position/odom',
        '/vn100/gz_data',
        '/dvl/velocity',
    ]

    bag_data = read_rosbag2_mcap(bag_path, topics_to_read)
    plot_data(bag_data)

    rclpy.shutdown()
