import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import csv
import copy
import numpy as np

def extract_fields_to_csv(bag_path, topic_name, field_paths, output_csv):
    rclpy.init()

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    if topic_name not in type_map:
        print(f"Topic '{topic_name}' not found in the bag.")
        return

    msg_type = get_message(type_map[topic_name])

    with open(output_csv, mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        # writer.writerow(field_paths[:5] + ['x stdev', 'y stdev', 'z stdev'] + field_paths[6:-1] + ['x stdev', 'y stdev', 'z stdev'])
        # writer.writerow(field_paths[:-1] + ['lat stdev', 'lon stdev', 'alt stdev'])
        writer.writerow(field_paths)
        # i = 0
        while reader.has_next():
            # i += 1
            topic, data, timestamp = reader.read_next()
            if topic == topic_name:
                msg = deserialize_message(data, msg_type)
                # Navigate through the nested fields
                field_values = []
                for field_path in field_paths:
                    value = msg
                    for attr in field_path.split('.'):
                        value = getattr(value, attr)
                    if isinstance(value, (list, np.ndarray)) and len(value) == 9:
                        cov_matrix = np.array(value).reshape((3, 3))
                        variances = np.diag(cov_matrix)
                        stdevs = np.sqrt(np.abs(variances))  # take only position x,y,z
                        field_values.extend(stdevs.tolist())
                    if isinstance(value, (list, np.ndarray)) and len(value) == 36:
                        cov_matrix = np.array(value).reshape((6, 6))
                        variances = np.diag(cov_matrix)
                        stdevs = np.sqrt(np.abs(variances[:3]))  # take only position x,y,z
                        field_values.extend(stdevs.tolist())
                    else:
                        field_values.append(value)
                # for field_path in field_paths:
                #     msg_this = copy.deepcopy(msg)
                #     for attr in field_path.split('.'):
                #         field_values.append(getattr(msg_this, field_path))
                # stdedvs_np = np.sqrt(field_values[-1])

                # stdevs = [np.sqrt(field_values[-1][0]), np.sqrt(field_values[-1][4]), np.sqrt(field_values[-1][8])]
                # writer.writerow(field_values[:-1] + stdevs)
                writer.writerow(field_values)

    print(f"Fields from topic '{topic_name}' have been written to '{output_csv}'.")

    rclpy.shutdown()

# Example usage
if __name__ == '__main__':
    # bag_path = '/home/annika/Documents/LEIDOS/GWMP_Data_Collection/Initial_runs/rosbag2_2025_05_14-17_08_15/'  # Replace with your bag file path
    # bag_path = '/home/annika/Documents/LEIDOS/GWMP_Data_Collection/Initial_runs/rosbag2_2025_05_13-15_11_33/'  # Replace with your bag file path
    name_prefix = 'run6'
    bag_path = '/media/annika/Storage1/test_2025-June-11_6/rosbag2_2025_06_11-16_42_24/'  # Replace with your bag file path
    # topic_name = '/hardware_interface/gnss_fix_fused'          # Replace with your topic name
    # field_paths = ['header.stamp.sec', 'header.stamp.nanosec', 'latitude', 'longitude', 'altitude', 'position_covariance']      # Replace with the desired field path
    # output_csv = f'{name_prefix}_gnss_fix_fused.csv'           # Replace with your desired output CSV file name
    # extract_fields_to_csv(bag_path, topic_name, field_paths, output_csv)
    topic_name = '/hardware_interface/novatel/oem7/inspvax'          # Replace with your topic name
    field_paths = ['header.stamp.sec', 'header.stamp.nanosec', 'latitude', 'longitude', 'height', 'latitude_stdev', 'longitude_stdev']      # Replace with the desired field path
    output_csv = f'{name_prefix}_inspvax.csv'           # Replace with your desired output CSV file name
    extract_fields_to_csv(bag_path, topic_name, field_paths, output_csv)
    topic_name = '/hardware_interface/novatel/oem7/gps'          # Replace with your topic name
    field_paths = ['header.stamp.sec', 'header.stamp.nanosec', 'latitude', 'longitude', 'speed', 'err_horz']      # Replace with the desired field path
    output_csv = f'{name_prefix}_gps.csv'           # Replace with your desired output CSV file name
    extract_fields_to_csv(bag_path, topic_name, field_paths, output_csv)
    # topic_name = '/hardware_interface/novatel/oem7/odom'          # Replace with your topic name
    # field_paths = ['header.stamp.sec', 'header.stamp.nanosec', 'pose.pose.position.x', 'pose.pose.position.y', 'pose.pose.position.z', 'pose.covariance', 'twist.twist.linear.x', 'twist.twist.linear.y', 'twist.twist.linear.z', 'twist.covariance']      # Replace with the desired field path
    # output_csv = f'{name_prefix}_odom.csv'           # Replace with your desired output CSV file name
    # extract_fields_to_csv(bag_path, topic_name, field_paths, output_csv)

# pose.pose.orientation.x
# pose.pose.orientation.y
# pose.pose.orientation.z
# pose.pose.orientation.w
# twist.twist.angular.x
# twist.twist.angular.y
# twist.twist.angular.z
