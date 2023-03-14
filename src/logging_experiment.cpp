#include <string>

#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char **argv) {
    std::string NODE_NAME = "logging_experiment";

    ros::init(argc, argv, NODE_NAME);
    
    ROS_INFO("starting logs ...");
    ROS_DEBUG("ending logs ...");

    return 1;

}
