#include "../include/objectrecognition/ros_control.h"


int main(int argc , char **argv)
{
    ros::init(argc , argv , "control");
    ros::NodeHandle nh;
    
    Controller controller(nh);

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    return 0;
}