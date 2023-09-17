// Author of SSL_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"


LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    printf("recieved pointcloud\n");
    mutex_lock.unlock();
   
}

double total_time =0;
int total_frame=0;
int frame_count =0;
void laser_processing(){
    while(1){
        // printf("Empty pointcloud %d\n", pointCloudBuf.empty());
        if(!pointCloudBuf.empty()){
            //read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            mutex_lock.unlock();
            frame_count++;
            if(frame_count%3!=0)
                continue;



            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZRGB>());          
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZRGB>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            printf("pointcloud size %d\n", pointcloud_in->size());
            laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            printf("edge size %d\n", pointcloud_edge->size());
            printf("surf size %d\n", pointcloud_surf->size());
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;


            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            if(total_frame%100==0)
                ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_in, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "base_link";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "base_link";
            pubEdgePoints.publish(edgePointsMsg);


            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "base_link";
            pubSurfPoints.publish(surfPointsMsg);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 100, velodyneHandler);

    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);

    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100);

    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100); 

    std::thread laser_processing_process{laser_processing};

    ros::spin();

    return 0;
}
