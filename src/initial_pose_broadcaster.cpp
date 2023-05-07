#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "init_tf_broadcaster");
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
    ROS_INFO_STREAM("HERRRRREEEEEEE");

    //Broadcast initial position and link map and base_link frames
    broadcaster.sendTransform(transformStamped);

    // ros::spin();
}