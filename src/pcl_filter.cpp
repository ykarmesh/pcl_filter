#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl/filters/radius_outlier_removal.h>

ros::Publisher pcl_pub;
tf::TransformListener* listener = NULL;

void PointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-2.5, -0.01);

  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (20);
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.4);
  outrem.setMinNeighborsInRadius (4);
  // apply filter
  outrem.filter (*cloud_filtered);

  sensor_msgs::PointCloud2 pc, pct;
  pcl::toROSMsg(*cloud_filtered, pc);
  pc.header.frame_id = msg->header.frame_id;
  pc.header.stamp = ros::Time::now();
  pct.header.frame_id = "/camera_rgb_optical_frame";
  pct.header.stamp = ros::Time::now();
  try
  {
    listener->waitForTransform("/camera_rgb_optical_frame", msg->header.frame_id, ros::Time::now(), ros::Duration(0.1));
  }
  catch (tf::TransformException &ex)
  {
    //ROS_ERROR("%s",ex.what());
    std::cout<< " Transform unavailable " <<std::endl;
    return;
  }
  pcl_ros::transformPointCloud("/camera_rgb_optical_frame", pc, pct, *listener);
  pcl_pub.publish(pct);
  return;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv,"pcl_filter");
  ros::NodeHandle nh;

  listener = new(tf::TransformListener);

  ros::Subscriber map_sub = nh.subscribe<sensor_msgs::PointCloud2>("/ORB_SLAM2/Pointcloud", 10, &PointCloudCB);

  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>( "rl/Pointcloud", 1 );

  ros::spin();
  return (0);
}
