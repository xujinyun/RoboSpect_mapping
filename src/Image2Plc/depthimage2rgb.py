import rospy
import rosbag
from cv_bridge import CvBridge
import cv2 as cv
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
        output_bag.write(topic, msg, t)
    elif topic == depth_image_topic:
        # print("topic: ", topic)
        depth_topic_list.append([topic, msg, t])
        depth_count += 1
        output_bag.write(topic, msg, t)
    else:
        output_bag.write(topic, msg, t)

count = min(len(rgb_topic_list), len(depth_topic_list))
for x in range(count):
    # print(len(depth_topic_list[x]))
    # print(len(rgb_topic_list[x]))
    print(x)
    depth_msg = depth_topic_list[x][1]
    rgb_msg = rgb_topic_list[x][1]
    t = depth_topic_list[x][2]
    print("t: ", t)

    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    depth_header = depth_msg.header

    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
    cv.resize(rgb_image, (img_width, img_height))

    xv, yv = np.meshgrid(np.arange(img_height), np.arange(img_width))
    cur_depth = (depth_image * 0.001).reshape((-1, 1))
    cur_x = (xv.reshape((-1, 1)) - cx) * cur_depth / fx
    cur_y = (yv.reshape((-1, 1)) - cy) * cur_depth / fy
    points_np = np.concatenate((cur_x, cur_y, cur_depth), axis=1)

    # cur_a = 255
    cur_r = rgb_image[:, :, 0].reshape((-1, 1))
    cur_g = rgb_image[:, :, 1].reshape((-1, 1))
    cur_b = rgb_image[:, :, 2].reshape((-1, 1))
    color_np = np.concatenate((cur_r, cur_g, cur_b), axis=1)

    valid_point = np.where(points_np[:, 2] > 0)
    points_np = points_np[valid_point]
    color_np = color_np[valid_point]


    C = np.zeros((color_np.shape[0], 4), dtype=np.uint8) + 255

    C[:, 0] = color_np[:, 0].astype(np.uint8)
    C[:, 1] = color_np[:, 1].astype(np.uint8)
    C[:, 2] = color_np[:, 2].astype(np.uint8)

    C = C.view("uint32")

    pointsColor = np.zeros( (points_np.shape[0], 1), \
        dtype={ 
            "names": ( "x", "y", "z", "rgba" ), 
            "formats": ( "f4", "f4", "f4", "u4" )} )

    points_np = points_np.astype(np.float32)

    pointsColor["x"] = points_np[:, 0].reshape((-1, 1))
    pointsColor["y"] = points_np[:, 1].reshape((-1, 1))
    pointsColor["z"] = points_np[:, 2].reshape((-1, 1))
    pointsColor["rgba"] = C



    # points_np = np.array(point_list, dtype=np.float32)
    new_msg = PointCloud2()
    new_msg.data
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
        PointField('rgb', 12, PointField.UINT32, 1)]
        # PointField('r', 9, PointField.UINT8, 1),
        # PointField('g', 10, PointField.UINT8, 1),
        # PointField('b', 11, PointField.UINT8, 1),
        # PointField('a', 12, PointField.UINT8, 1)]

    new_msg.is_bigendian = False
    new_msg.point_step = 16
    new_msg.row_step = new_msg.point_step * new_msg.width #points_np.shape[0]
    new_msg.is_dense = False
    # print(np.asarray(points_np, np.float32).tostring())
    new_msg.data = pointsColor.tostring() # np.asarray(points_np, np.float32).tostring()

    # print(new_msg.data)

    print("write new topic: ", new_pc_topic)
    output_bag.write(new_pc_topic, new_msg, t)

output_bag.close()