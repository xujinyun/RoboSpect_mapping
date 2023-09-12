//打开含有topic数据的bag
rosbag::Bag bag;
bag.open(argv[3], rosbag::bagmode::Read);
std::vector<std::string> topics;
//添加将要合并的topic名称
topics.push_back(std::string("/tf"));
topics.push_back(std::string("/zed/zed_node/left/camera_info"));
topics.push_back(std::string("/zed/zed_node/left/image_rect_color"));
rosbag::View view(bag, rosbag::TopicQuery(topics));
//遍历bag包
foreach(rosbag::MessageInstance const m, view)
{
    tf2_msgs::TFMessageConstPtr tf = m.instantiate<tf2_msgs::TFMessage>();
    sensor_msgs::ImageConstPtr image = m.instantiate<sensor_msgs::Image>();
    sensor_msgs::CameraInfoConstPtr info = m.instantiate<sensor_msgs::CameraInfo>();
    //赋值给所选择的类型，如果不为空，则添加到rosbag中
    if (tf != NULL){
        //std::cout<<m.getTopic()<<std::endl;
        bag_out.write(m.getTopic(), m.getTime(), *tf);
    }
    if (image != NULL){
        //std::cout<<m.getTopic()<<std::endl;
        bag_out.write(m.getTopic(), m.getTime(), *image);
    }
    if (info != NULL){
        //std::cout<<m.getTopic()<<std::endl;
        bag_out.write(m.getTopic(), m.getTime(), *info);
    }
}
//关闭写入的rosbag包
bag_out.close();
————————————————
版权声明：本文为CSDN博主「秃头队长」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/qq_39266065/article/details/113255176