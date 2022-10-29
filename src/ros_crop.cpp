#include "../include/objectrecognition/ros_crop.h"

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

typedef  pcl::PointXYZ PointType;

KdTree* tree;
std::ofstream fw;
const std::string filename{"/home/wasp/catkin_ws/src/ros-pcl-ml/text_file/data_collected.txt"};


Controller::Controller(ros::NodeHandle nh):m_nh(nh),pr_nh("~"){

    m_PontCloud_callback = m_nh.subscribe(point_cloud , 1 , &Controller::callback , this);
   
    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_topic", 1);
    
    server_.reset(new ReconfigureServer(dr_mutex_));  
    dynamic_reconfigure::Server<objectrecognition::TuningConfig>::CallbackType cbt =
            boost::bind(&Controller::drCallback, this, boost::placeholders::_1, boost::placeholders::_2);
    server_->setCallback(cbt);
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("Non_plane", 1);

    pr_nh.getParam("xmin", _xmin);
    pr_nh.getParam("xmax", _xmax);
    pr_nh.getParam("ymin", _ymin);
    pr_nh.getParam("ymax", _ymax);
    pr_nh.getParam("zmin", _zmin);
    pr_nh.getParam("zmax", _zmax);
    
    pr_nh.getParam("orientation_x", _orientation_x);
    pr_nh.getParam("orientation_y", _orientation_y);
    pr_nh.getParam("orientation_z", _orientation_z);
    pr_nh.getParam("orientation_w", _orientation_w);

    pr_nh.getParam("resolution", _resolution);
    pr_nh.getParam("thereshold", _thereshold);
    pr_nh.getParam("radius", _radius);




    fw.open (filename);



}
void Controller::callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg){
    cropped_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*pointcloud_msg.get(), *cropped_cloud_ptr.get());


    std::cout << "callback ho raha hai kya" << std::endl;

    pcl::PointXYZ cc_min, cc_max;
    if(cropped_cloud_ptr->points.size() > 0){
        cout << "zero toh nahi hai" << endl; 

    


















    
    pcl::getMinMax3D(*cropped_cloud_ptr, cc_min, cc_max);
    if(cc_min.x <= cc_max.x && cc_min.y<=cc_max.y && cc_min.z <= cc_max.z){
            pcl::toROSMsg(*cropped_cloud_ptr.get(),cropped_cloud_msg );
            cropped_cloud_msg.header.stamp = pointcloud_msg->header.stamp;
            cropped_cloud_msg.header.frame_id = pointcloud_msg->header.frame_id;
            pub1.publish(cropped_cloud_msg);
        }
    }
    if(config_.save_enable){
        pcl::io::savePCDFileASCII ("/home/sutd/catkin_ws/src/objectrecognition/pcd/test.pcd", *cropped_cloud_ptr.get());
    }

    project_to_costmap(cropped_cloud_ptr);


}





template
<typename PointT>
bool Controller::project_to_costmap(boost::shared_ptr<pcl::PointCloud<PointT> > input_cloud_ptr){
    pcl::PointXYZ bb_min, bb_max;
    pcl::getMinMax3D(*input_cloud_ptr, bb_min, bb_max);

     
    for (auto& point : *input_cloud_ptr)
    {
      if (point.z > bb_min.z and point.z < bb_max.z)
        point.z = 0;
      else
        cout << "have you reached here3" <<endl; 
        continue;
    }


    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = "map";
    if(config_.map_enable){
        grid.info.resolution = config_.resolution;
    }else{
        grid.info.resolution = _resolution;
    }
    



    if (bb_min.y < bb_max.y){
      grid.info.height = ((int)((bb_max.y - bb_min.y) / grid.info.resolution)) + 1;
      cout << "have you reached here5" <<endl; 
      cout << grid.info.height << "width" << endl;
    }
    else  
      grid.info.height = 1;


    if (bb_min.x < bb_max.x){
      grid.info.width = ((int)((bb_max.x - bb_min.x) / grid.info.resolution)) + 1;
      cout << grid.info.width << "height" <<endl;
      cout << "have you reached here6" <<endl; }
    else  
      grid.info.width = 1;
    int size_data = input_cloud_ptr->points.size();


    std::vector<std::vector<double>> total_data(size_data);
    for (int i = 0 ; i < size_data ; i++) {
        total_data[i].resize(2, 0.0);
    }
 
    total_data = search_cloud(input_cloud_ptr);
    KdTree* tree = new KdTree;
    for (int i=0; i<total_data.size(); i++)
    tree->insert(total_data[i],i);




    grid.info.origin.position.y = bb_min.y;
    grid.info.origin.position.x = bb_min.x;
    grid.info.origin.position.z = bb_min.z;

    if(config_.quaternion_enable){
    grid.info.origin.orientation.w = config_.orientation_w_w;
    grid.info.origin.orientation.x = config_.orientation_x_x;
    grid.info.origin.orientation.y = config_.orientation_y_y;
    grid.info.origin.orientation.z = config_.orientation_z_z; 
    }else{
    grid.info.origin.orientation.w = _orientation_w;
    grid.info.origin.orientation.x = _orientation_x;
    grid.info.origin.orientation.y = _orientation_y;
    grid.info.origin.orientation.z = _orientation_z;
       
    }

    size_t grid_size = grid.info.width * grid.info.height;
    grid.data = std::vector<int8_t>(grid_size, 0);
    int index = 0;
    if(config_.flip_enable){
        for (int index_y = 0; index_y < grid.info.width; index_y++)
        {
            for (int index_x = 0; index_x < grid.info.height; index_x++)
            {
            unsigned int i = index_x + (grid.info.height - index_y - 1) * grid.info.width;
            double y = bb_min.y + index_x * grid.info.resolution;
            double x = bb_min.x + index_y * grid.info.resolution;
            

            if(config_.map_enable){
            nearby = tree->search({y,x}, config_.radius);
            }else{
            nearby = tree->search({y,x}, _radius);
            }
        
            if (nearby.size() > _thereshold)
                grid.data[i] = 0;
            else
                grid.data[i] = 100;

            //index++;
            }
        }
    }else{
        for (int index_y = 0; index_y < grid.info.height; index_y++)
        {
            for (int index_x = 0; index_x < grid.info.width; index_x++)
            {
                //unsigned int i = index_x + (grid.info.height - index_y - 1) * grid.info.width;
                double y = bb_min.y + index_x * grid.info.resolution;
                double x = bb_min.x + index_y * grid.info.resolution;

            if(config_.map_enable){
                nearby = tree->search({y,x}, config_.radius);
            }else{
                nearby = tree->search({y,x}, _radius);
            }
        
            if (nearby.size() > _thereshold)
                grid.data[index] = 0;
            else
                grid.data[index] = 100;

            index++;
            }
        }
    }
    grid_pub.publish(grid);

    return true;
}


template
<typename PointT>
std::vector<std::vector<double>> Controller::search_cloud(boost::shared_ptr<pcl::PointCloud<PointT> > input_cloud_ptr){

   
    int size_data = input_cloud_ptr->points.size();
    std::vector<std::vector<double>> points(size_data);
    for (int i = 0 ; i < size_data ; i++) {
        points[i].resize(2, 0.0);
    }



    for(size_t i=0;i<input_cloud_ptr->points.size();i++){
        //point.x = input_cloud_ptr.points[i].x;
        points[i][0] = input_cloud_ptr->points[i].y;
        points[i][1] = input_cloud_ptr->points[i].x;

    }
    return points;
}




void Controller::drCallback(objectrecognition::TuningConfig& config,
        uint32_t level)
{       
        config_ = config;
        ROS_INFO("Reconfigure called");
}