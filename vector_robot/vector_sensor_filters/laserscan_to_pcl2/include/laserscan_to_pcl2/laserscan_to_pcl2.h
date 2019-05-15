
#ifndef LASERSCAN_TO_PCL2_NODELET_H
#define LASERSCAN_TO_PCL2_NODELET_H

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

namespace laserscan_to_pcl2
{

  class LaserScanToPCL2Nodelet : public nodelet::Nodelet
  {

  public:
    LaserScanToPCL2Nodelet();

  private:
    virtual void onInit();
    void scanCallback();

    ros::NodeHandle nh, private_nh;
    laser_geometry::LaserProjection projector_;
    ros::Subscriber sub_;
    ros::Publisher pub_;


  };

}  // laserscan_to_pcl2

#endif  // LASERSCAN_TO_PCL2_NODELET_H