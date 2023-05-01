import pandas as pd
import csv

# Round to decimal the timestamp column for TF
with open('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/table_tf.csv', newline='') as infile:
    reader = csv.reader(infile)

    with open('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/TF.csv', 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        header = next(reader)
        writer.writerow(header)
        for row in reader:
            rounded_value = round(float(row[0]), 1)
            row[0] = rounded_value
            writer.writerow(row)

# Round to decimal the timestamp column for SCAN
with open('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/table_scan.csv', newline='') as infile:
    reader = csv.reader(infile)

    with open('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/SCAN.csv', 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        header = next(reader)
        writer.writerow(header)
        for row in reader:
            rounded_value = round(float(row[0]), 1)
            row[0] = rounded_value
            writer.writerow(row)


# Read the CSV files into pandas DataFrames
data_tf = pd.read_csv('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/TF.csv')
data_scan = pd.read_csv('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/SCAN.csv')

# Merge the DataFrames based on the timestamp column
merged_data = pd.merge(data_tf, data_scan, on='timestamp')

# Write the merged data to a new CSV file
merged_data.to_csv('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/MERGED.csv', index=False)