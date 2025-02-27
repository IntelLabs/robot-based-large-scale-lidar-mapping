#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

import atexit

import numpy as np

image_rows_full = 1024
image_cols = 8192

ang_res_x = 360.0/float(image_cols) # horizontal resolution
ang_res_y = 90/float(image_rows_full-1) # vertical resolution
ang_start_y = 45 # bottom beam angle
max_range = 80.0
min_range = 2.0

range_image_array = np.empty([0, image_rows_full, image_cols, 1], dtype=np.float32)

def shutdown_callback():
    global range_image_array
    np.save('data_full.npy', range_image_array)
    print('Saved')


class Pc2Numpy(Node):

    def __init__(self):
        super().__init__('pc2numpy')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/scan/full',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        

    def listener_callback(self, msg):
        global range_image_array
        print('processing {}th point cloud message...\r'.format(range_image_array.shape[0]))
        #points_obj = point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        points_obj = point_cloud2.read_points_numpy(msg,("x","y","z"),True)
        #print(points_obj)
        points_array = np.array(list(points_obj), dtype=np.float32)
        #print('point_array')
        range_image = np.zeros((1, image_rows_full, image_cols, 1), dtype=np.float32)
        #print('range')
        x = points_array[:,0]
        y = points_array[:,1]
        z = points_array[:,2]
        vertical_angle = np.arctan2(z, np.sqrt(x * x + y * y)) * 180.0 / np.pi
        relative_vertical_angle = vertical_angle + ang_start_y
        rowId = np.int_(np.round_(relative_vertical_angle / ang_res_y))
        # find column id
        horitontal_angle = np.arctan2(x, y) * 180.0 / np.pi
        colId = -np.int_((horitontal_angle-90.0)/ang_res_x) + image_cols/2;
        shift_ids = np.where(colId>=image_cols)
        colId[shift_ids] = colId[shift_ids] - image_cols
        # filter range
        thisRange = np.sqrt(x * x + y * y + z * z)
        thisRange[thisRange > max_range] = 0
        thisRange[thisRange < min_range] = 0
        # save range info to range image
        for i in range(len(thisRange)):
            if rowId[i] < 0 or rowId[i] >= image_rows_full or colId[i] < 0 or colId[i] >= image_cols:
                continue
            range_image[0, int(rowId[i]), int(colId[i]), 0] = thisRange[i]
        # append range image to array
        range_image_array = np.append(range_image_array, range_image, axis=0)




def main(args=None):
    rclpy.init(args=args)

    
    pc2numpy = Pc2Numpy()

    atexit.register(shutdown_callback)

    rclpy.spin(pc2numpy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pc2numpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()