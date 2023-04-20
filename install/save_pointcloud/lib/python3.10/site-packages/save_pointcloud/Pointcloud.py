import pandas as pd
import numpy as np

# Convert csv to xlsx
df = pd.read_csv('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/MERGED.csv', delimiter=",")
writer = pd.ExcelWriter('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/MERGED.xlsx')
df.to_excel(writer, index=False)
writer.save()

# Load data from xlsx file
data = pd.read_excel('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/MERGED.xlsx')

# Extract transformation parameters
tx = data['tx'].values
ty = data['ty'].values
tz = data['tz'].values
qx = data['qx'].values
qy = data['qy'].values
qz = data['qz'].values
qw = data['qw'].values

# Extract scan data
x = data['x'].values
y = data['y'].values
z = data['z'].values

x = x.astype(float)
y = y.astype(float)
z = z.astype(float)

# Combine translation and rotation into transformation matrix
T = np.zeros((len(data), 4, 4))
T[:, 0, 0] = 1 - 2 * (qy**2 + qz**2)
T[:, 0, 1] = 2 * (qx * qy - qz * qw)
T[:, 0, 2] = 2 * (qx * qz + qy * qw)
T[:, 1, 0] = 2 * (qx * qy + qz * qw)
T[:, 1, 1] = 1 - 2 * (qx**2 + qz**2)
T[:, 1, 2] = 2 * (qy * qz - qx * qw)
T[:, 2, 0] = 2 * (qx * qz - qy * qw)
T[:, 2, 1] = 2 * (qy * qz + qx * qw)
T[:, 2, 2] = 1 - 2 * (qx**2 + qy**2)
T[:, 0, 3] = tx
T[:, 1, 3] = ty
T[:, 2, 3] = tz
T[:, 3, 3] = 1

# Apply transformation to scan data
scan = np.column_stack((x, y, z))
transformed_scan = np.zeros(scan.shape)
for i in range(len(data)):
    transformed_scan[i] = np.matmul(T[i], np.append(np.array([x[i], y[i], z[i], 1.0]), 1.0))[:3]

# Save transformed scan as binary pcd file
with open('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/pcd_files/Map.pcd', 'wb') as f:
    # Write header
    header = [
        b'# .PCD v0.7 - Point Cloud Data\n',
        b'VERSION 0.7\n',
        b'FIELDS x y z timestamp\n',
        b'SIZE 4 4 4 4\n',
        b'TYPE F F F F\n',
        b'COUNT 1 1 1 1\n',
        b'WIDTH ' + str(len(data)).encode() + b'\n',
        b'HEIGHT 1\n',
        b'VIEWPOINT 0 0 0 1 0 0 0\n',
        b'POINTS ' + str(len(data)).encode() + b'\n',
        b'DATA binary\n',
    ]
    f.writelines(header)
    # Write data
    for i in range(len(data)):
        f.write(transformed_scan[i].tobytes())
