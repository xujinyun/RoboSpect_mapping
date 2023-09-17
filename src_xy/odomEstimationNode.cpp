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
#include "odomEstimationClass.h"

#include <gazebo_msgs/ModelStates.h>

OdomEstimationClass odomEstimation;
std::mutex mutex_lock, odom_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry;
void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

bool is_odom_inited = false;
double total_time =0;
int total_frame=0;

// Initilize with 0
Eigen::Vector3d t_current = Eigen::Vector3d::Zero();
Eigen::Quaterniond q_current = Eigen::Quaterniond::Identity();


void odom_estimation(){
    while(1){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()&& !pointCloudBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudBuf.empty() && (pointCloudBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || pointCloudBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                ROS_WARN("time stamp unaligned error and odom discarded, pls check your data --> odom correction"); 
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;              
            }

            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec()<pointCloudBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || pointCloudSurfBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec()<pointCloudBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || pointCloudEdgeBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }
            //if time aligned 

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            pointCloudBuf.pop();
            mutex_lock.unlock();

            if(is_odom_inited == false){
                // check number of points in pointcloud_edge_in and pointcloud_surf_in
                
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
            }else{
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();

                //Set odometry
                Eigen::Isometry3d odom_in;
                odom_lock.lock();
                odom_in.translation() = t_current;
                odom_in.linear() = q_current.toRotationMatrix();
                // if(pointcloud_edge_in->points.size()<100 || pointcloud_surf_in->points.size()<100){
                //     ROS_WARN("pointcloud_edge_in or pointcloud_surf_in is too small, pls check your data --> odom correction");
                //     continue;
                // }
                printf("pointcloud_edge_in size: %d, pointcloud_surf_in size: %d \n", pointcloud_edge_in->points.size(), pointcloud_surf_in->points.size());
                odomEstimation.updateStateToMap(pointcloud_edge_in, pointcloud_surf_in, odom_in);
                odom_lock.unlock();

                // odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                if(total_frame%100==0)
                    ROS_INFO("average odom estimation time %f ms \n \n", total_time/total_frame);
            }

            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            //q_current.normalize();
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            // // publish odometry
            // nav_msgs::Odometry laserOdometry;
            // laserOdometry.header.frame_id = "map"; 
            // laserOdometry.child_frame_id = "base_link"; 
            // // laserOdometry.header.stamp = pointcloud_time;
            // laserOdometry.header.stamp = ros::Time::now();
            // laserOdometry.pose.pose.orientation.x = q_current.x();
            // laserOdometry.pose.pose.orientation.y = q_current.y();
            // laserOdometry.pose.pose.orientation.z = q_current.z();
            // laserOdometry.pose.pose.orientation.w = q_current.w();
            // laserOdometry.pose.pose.position.x = t_current.x();
            // laserOdometry.pose.pose.position.y = t_current.y();
            // laserOdometry.pose.pose.position.z = t_current.z();
            // pubLaserOdometry.publish(laserOdometry);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void odomHandler(const gazebo_msgs::ModelStatesConstPtr &state)
{
    double position_x = state->pose[2].position.x;
    double position_y = state->pose[2].position.y;
    double position_z = state->pose[2].position.z;
    double orientation_x = state->pose[2].orientation.x;
    double orientation_y = state->pose[2].orientation.y;
    double orientation_z = state->pose[2].orientation.z;
    double orientation_w = state->pose[2].orientation.w;
    // printf("recieved odom \n");
    
    //set odom
    odom_lock.lock();
    t_current = Eigen::Vector3d(position_x, position_y, position_z);
    q_current = Eigen::Quaterniond(orientation_w, orientation_x, orientation_y, orientation_z).toRotationMatrix();
    odom_lock.unlock();


    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin( tf::Vector3(position_x, position_y, position_z) );
    // tf::Quaternion q(orientation_x,orientation_y, orientation_z,orientation_w);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));


    // publish odometry
    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "map"; 
    laserOdometry.child_frame_id = "base_link"; 
    laserOdometry.header.stamp = ros::Time::now();
    laserOdometry.pose.pose.orientation.x = orientation_x;
    laserOdometry.pose.pose.orientation.y = orientation_y;
    laserOdometry.pose.pose.orientation.z = orientation_z;
    laserOdometry.pose.pose.orientation.w = orientation_w;
    laserOdometry.pose.pose.position.x = position_x;
    laserOdometry.pose.pose.position.y = position_y;
    laserOdometry.pose.pose.position.z = position_z;
    pubLaserOdometry.publish(laserOdometry);
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
    double map_resolution = 0.4;
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, map_resolution);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100, velodyneHandler);
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);

    // Subscribe to the odometry topic from Gazebo directly
    ros::Subscriber subOdom = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 100, odomHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}
