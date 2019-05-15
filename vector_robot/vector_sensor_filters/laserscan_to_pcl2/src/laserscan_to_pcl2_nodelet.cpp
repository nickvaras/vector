#include <ros/ros.h>
#include "nodelet/nodelet.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pluginlib/class_list_macros.h>

namespace laserscan_to_pcl2
{

  class LaserScanToPCL2Nodelet : public nodelet::Nodelet
  {

  public:
    LaserScanToPCL2Nodelet(){};

  private:
    virtual void onInit()
    {
        nh = getNodeHandle();
        // private_nh = getPrivateNodeHandle();
        sub_ = nh.subscribe("scan", 1, &LaserScanToPCL2Nodelet::scanCallback, this);
        pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2", 10);
    };
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        sensor_msgs::PointCloud2 cloud2;
        projector_.projectLaser(*scan_in, cloud2);
        pub_.publish(cloud2);
    };

    ros::NodeHandle nh, private_nh;
    laser_geometry::LaserProjection projector_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
  };

};  // namespace laserscan_to_pcl2

PLUGINLIB_EXPORT_CLASS(laserscan_to_pcl2::LaserScanToPCL2Nodelet, nodelet::Nodelet)
