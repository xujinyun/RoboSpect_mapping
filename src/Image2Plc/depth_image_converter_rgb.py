import rospy
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np
input_bag_filepath = "/home/jinyun/L515_data/conllection_with_info.bag"
output_bag_filepath = "/home/jinyun/L515_data/convert_test.bag"

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
    
    if topic == rgb_image_topic:
        # print("topic: ", topic)
        rgb_topic_list.append([topic, msg, t])
        rgb_count += 1
    elif topic == depth_image_topic:
        # print("topic: ", topic)
        depth_topic_list.append([topic, msg, t])
        depth_count += 1
    else:
        output_bag.write(topic, msg, t)
# print(rgb_count, depth_count)
# print(len(rgb_topic_list))
# print(len(depth_topic_list))

count = min(len(rgb_topic_list), len(depth_topic_list))
for x in range(count):
    # print(len(depth_topic_list[x]))
    # print(len(rgb_topic_list[x]))
    print(x)
    depth_msg = depth_topic_list[x][1]
    rgb_msg = rgb_topic_list[x][1]

    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    depth_header = depth_msg.header

    point_list = []

    for i in range(img_height):
        for j in range(img_width):
            cur_depth = depth_image[i][j] * 0.001
            cur_x = (j - cx) * cur_depth / fx
            cur_y = (i - cy) * cur_depth / fy
            # print(cur_x, cur_y, cur_depth)

            # cur_a = 255
            cur_r = rgb_msg.data[1]
            cur_g = rgb_msg.data[2]
            cur_b = rgb_msg.data[3]



            point_list.append([cur_x, cur_y, cur_depth, cur_r, cur_g, cur_b])
    points_np = np.array(point_list)
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
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 16, PointField.FLOAT32, 1)]
        

    new_msg.is_bigendian = False
    new_msg.point_step = 16
    new_msg.row_step = new_msg.point_step * points_np.shape[0]
    new_msg.is_dense = False
    new_msg.data = np.asarray(points_np, np.float32).tostring()

    print("write new topic: ", new_pc_topic)
    output_bag.write(new_pc_topic, new_msg, t)

output_bag.close()