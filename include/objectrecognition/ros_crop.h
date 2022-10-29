
#ifndef IMAGE_CONTROL_H
#define IMAGE_CONTROL_H
#include <iostream>
#include <string>
#include <fstream>
#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include <unordered_set>
#include <chrono>
#include <thread>
#include <pcl/common/common.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>



#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>


#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/ModelCoefficients.h>
#include <nav_msgs/OccupancyGrid.h>
#include "newkdtree.h"
#include <dynamic_reconfigure/server.h>
#include <objectrecognition/TuningConfig.h>

using namespace std;
typedef  pcl::PointXYZ PointType;

class Controller
{
    public:

        explicit Controller(ros::NodeHandle nh);

        void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg );
       



        template
        <typename PointT>
        bool project_to_costmap(boost::shared_ptr<pcl::PointCloud<PointT> > input_cloud_ptr);
        template
        <typename PointT>
        std::vector<std::vector<double>> search_cloud(boost::shared_ptr<pcl::PointCloud<PointT> > input_cloud_ptr);

        void drCallback(objectrecognition::TuningConfig& config,uint32_t level);

    private:
        ros::NodeHandle m_nh;
        ros::NodeHandle pr_nh;

        double _xmin , _xmax , _ymin , _ymax , _zmin , _zmax;
        double _orientation_x , _orientation_y , _orientation_z , _orientation_w;
        double _resolution , _thereshold , _radius;
  

        ros::Subscriber m_PontCloud_callback;
        ros::Subscriber marker_callback_sub;
        std::vector<int> nearby;
        boost::recursive_mutex dr_mutex_;
        objectrecognition::TuningConfig config_;
        typedef dynamic_reconfigure::Server<objectrecognition::TuningConfig> ReconfigureServer;
        boost::shared_ptr< ReconfigureServer > server_;


        double x_min,x_max, y_min, y_max,z_min,z_max;

        pcl::PointCloud<PointType>::Ptr cropped_cloud_ptr;
        pcl::PointCloud<PointType>::Ptr crop_box_cloud_ptr;
        pcl::PointCloud<PointType>::Ptr received_cloud_ptr;
        sensor_msgs::PointCloud2 cropped_cloud_msg;


        ros::Publisher pub1;


        const std::string point_cloud = "/mapcloud";


    nav_msgs::OccupancyGrid grid;
    ros::Publisher grid_pub;


};

#endif