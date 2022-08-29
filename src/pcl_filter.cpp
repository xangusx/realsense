#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;

ros::Publisher pub;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PCLPointCloud2* cloud = new(pcl::PCLPointCloud2);
    //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl_conversions::toPCL(*msg,*cloud);

    
    (*cloud).header.frame_id = "map";
    pub.publish(*cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"pcl_filter");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points",10,cloud_callback);
    pub = nh.advertise<pcl::PCLPointCloud2>("pcl_filtering",10);
    ros::spin();
}