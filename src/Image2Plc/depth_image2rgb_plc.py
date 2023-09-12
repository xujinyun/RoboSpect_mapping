import rospy
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np
import cv2
input_bag_filepath = "/home/jinyun/L515_data/data1.bag"
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
    depth_msg = depth_topic_list[x][1]
    rgb_msg = rgb_topic_list[x][1]
    t = depth_topic_list[x][2]

    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    depth_header = depth_msg.header

    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
    rgb_header = depth_msg.header
    depth_dim = (img_width, img_height)
    # resize image
    resized_rgb_image = cv2.resize(rgb_image, depth_dim, interpolation = cv2.INTER_AREA)

    yv, xv = np.meshgrid(range(img_height), range(img_width) , indexing='ij')

    xv_vector = xv.reshape([img_height * img_width, 1])
    yv_vector = yv.reshape([img_height * img_width, 1])
    depth_vector = depth_image.reshape([img_height * img_width, 1]) * 0.001

    valid_xv_vector = xv_vector[depth_vector > 0].reshape([-1,1])
    valid_yv_vector = yv_vector[depth_vector > 0].reshape([-1,1])
    valid_depth_vector = depth_vector[depth_vector > 0].reshape([-1,1])

    # print(valid_depth_vector.shape)

    valid_point_vector = np.concatenate((valid_xv_vector, valid_yv_vector, valid_depth_vector), axis=1)
    # print(valid_point_vector.shape)

    valid_point_vector[:,0] = (valid_point_vector[:,0] - cx) * valid_point_vector[:,2] / fx
    valid_point_vector[:,1] = (valid_point_vector[:,1] - cy) * valid_point_vector[:,2] / fy

    r_vector = resized_rgb_image[:,:,0].reshape([img_height * img_width, 1])
    g_vector = resized_rgb_image[:,:,1].reshape([img_height * img_width, 1])
    b_vector = resized_rgb_image[:,:,2].reshape([img_height * img_width, 1])

    valid_r_vector = r_vector[depth_vector > 0].reshape([-1,1])
    valid_g_vector = g_vector[depth_vector > 0].reshape([-1,1])
    valid_b_vector = b_vector[depth_vector > 0].reshape([-1,1])

    point_size = valid_point_vector.shape[0]
    C = np.zeros((point_size, 4), dtype=np.uint8) + 255
    C[:, 0:1] = valid_r_vector.astype(np.uint8)
    C[:, 1:2] = valid_g_vector.astype(np.uint8)
    C[:, 2:3] = valid_b_vector.astype(np.uint8)

    C = C.view("uint32")

    # Structured array.
    pointsColor = np.zeros( (point_size, 1), \
        dtype={
            "names": ( "x", "y", "z", "rgba" ),
            "formats": ( "f4", "f4", "f4", "u4" )} )
   
    points = valid_point_vector.astype(np.float32)
    pointsColor["x"] = points[:, 0].reshape((-1, 1))
    pointsColor["y"] = points[:, 1].reshape((-1, 1))
    pointsColor["z"] = points[:, 2].reshape((-1, 1))
    pointsColor["rgba"] = C

    # print(pointsColor[0])

    msg = PointCloud2()
    msg.header = depth_msg.header
    msg.header.frame_id = "l515"
    msg.header.stamp = t

    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width  = points.shape[0]

    msg.fields = [
        PointField('x',  0, PointField.FLOAT32, 1),
        PointField('y',  4, PointField.FLOAT32, 1),
        PointField('z',  8, PointField.FLOAT32, 1),
        PointField('rgb', 12, PointField.UINT32, 1),
        ]
   
    msg.is_bigendian = False
    msg.point_step   = 16
    msg.row_step     = msg.point_step * points.shape[0]
    msg.is_dense     = int( np.isfinite(points).all() )
    msg.data         = pointsColor.tostring()


    output_bag.write(new_pc_topic, msg, depth_topic_list[x][2])
output_bag.close()