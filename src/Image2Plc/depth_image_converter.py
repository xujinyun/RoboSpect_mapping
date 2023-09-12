import rospy
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np
input_bag_filepath = "/home/jinyun/L515_data/data1.bag"
output_bag_filepath = "/home/jinyun/L515_data/data1_plc.bag"

bridge = CvBridge()
input_bag  = rosbag.Bag(input_bag_filepath)
output_bag  = rosbag.Bag(output_bag_filepath, 'w')

fx = 465.394531
fy = 464.890625
cx = 287.734375
cy = 236.113281

img_height = 480
img_width = 640

depth_image_topic = "/camera/depth/image_rect_raw/compressed_realsense_depth"
rgb_image_topic = "/camera/color/image_raw/compressed_realsense_rgb"
new_pc_topic = "/camera/depth/color/points"

rgb_count = 0
depth_count = 0
rgb_topic_list = []
depth_topic_list = []

for topic, msg, t in input_bag.read_messages():

    
    if topic == depth_image_topic:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_header = msg.header

        point_list = []

        for i in range(img_height):
            for j in range(img_width):
                cur_depth = depth_image[i][j] * 0.001
                cur_x = (j - cx) * cur_depth / fx
                cur_y = (i - cy) * cur_depth / fy
                # print(cur_x, cur_y, cur_depth)

                point_list.append([cur_x, cur_y, cur_depth])
        points_np = np.array(point_list, dtype=np.float32)
        new_msg = PointCloud2()
        new_msg.header = depth_header
        new_msg.header.frame_id = "l515"

        if len(points_np.shape) == 3:
            new_msg.height = points_np.shape[1]
            new_msg.width = points_np.shape[0]
        else:
            new_msg.height = 1
            new_msg.width = len(points_np)

        new_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]

        new_msg.is_bigendian = False
        new_msg.point_step = 12
        new_msg.row_step = new_msg.point_step * points_np.shape[0]
        new_msg.is_dense = False
        new_msg.data = np.asarray(points_np, np.float32).tostring()

        output_bag.write(new_pc_topic, new_msg, t)
    else:
        output_bag.write(topic, msg, t)


output_bag.close()
