#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree.h>//kdtree algorithm
#include <pcl/search/kdtree.h>
#include <time.h>

#include <pcl/keypoints/iss_3d.h>//match points
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <std_msgs/Header.h>

using pcl::NormalEstimation;
using pcl::search::KdTree;

typedef pcl::PointXYZ pointxyz;
typedef pcl::PointCloud<pointxyz> PointCloud;
using namespace std;
ros::Publisher pub;

void cloud_data(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr)
{
    pcl::PointCloud<pointxyz>::Ptr current_pcl_ptr(new pcl::PointCloud<pointxyz>);
    pcl::fromROSMsg(*cloud_ptr,*current_pcl_ptr);

    clock_t start = clock();

    //leaf
    double leaf_size = 0.3;
    PointCloud::Ptr pclsrc_leaf_ptr(new PointCloud);
    pcl::VoxelGrid<pointxyz> filter;
    filter.setInputCloud(current_pcl_ptr);
    filter.setLeafSize(leaf_size,leaf_size,leaf_size);
    filter.filter(*pclsrc_leaf_ptr);

    //iss
    double iss_size = 0.3;
    PointCloud::Ptr pclsrc_temp_ptr(new PointCloud);
    pcl::ISSKeypoint3D<pointxyz,pointxyz> iss_det;
    pcl::search::KdTree<pointxyz>::Ptr tree_1(new pcl::search::KdTree<pointxyz>());

    double model_sol = 0.2;

    iss_det.setSearchMethod(tree_1);
    iss_det.setSalientRadius(iss_size);
    iss_det.setNonMaxRadius(0.5);
    iss_det.setThreshold21(0.975);
    iss_det.setThreshold32(0.975);
    iss_det.setNumberOfThreads(4);
    iss_det.setInputCloud(pclsrc_leaf_ptr);
    iss_det.compute(*pclsrc_temp_ptr);

    clock_t end = clock();

    PointCloud::Ptr cloud_key(new PointCloud);
    pcl::copyPointCloud(*pclsrc_temp_ptr, *cloud_key);

    sensor_msgs::PointCloud2 pcl_keypoint_data;
    pcl::toROSMsg(*cloud_key, pcl_keypoint_data);

    pcl_keypoint_data.header = cloud_ptr->header;

    pub.publish(pcl_keypoint_data);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"detectpclFeatures");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("pcl_filtering",10,cloud_data);
    pub = nh.advertise<pcl::PCLPointCloud2>("pcl_keypoint",10);

    ros::spin();
}
