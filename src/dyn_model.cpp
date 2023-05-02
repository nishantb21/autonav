#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

//used for keeping track of the state of the robot
struct state{  
        float x;
        float y;
        float yaw;

        // derivative states
        float x_dot;
        float y_dot;
        float yaw_dot;

        //wheel angle
        float wheel_ang;
};

class DynModel {
    public:
        DynModel(float dtime) : nh_("~") {
            this->dtime = dtime;

            // Initialize publisher
            pub_ = nh_.advertise<std_msgs::String>("my_topic_out", 10);
            // topic to publish: predicted position of the robot
            
            // Initialize subscriber
            // sub_ = nh_.subscribe("/known_state", 10, &MyNode::callback, this);
            sub_imu = nh_.subscribe("/imu/data", 10, &DynModel::callback_imu, this);
            // topics to sub to:
            // commmands: 
        }

        void callback_imu(const sensor_msgs::Imu::ConstPtr& msg) {
            // Process message received on the subscriber
            // ROS_INFO_STREAM("Received message: " << msg->header);
            
            //update known state

            // Create and publish a response message
            std_msgs::String response_msg;
            response_msg.data = "recieved imu data";
            pub_.publish(response_msg);
        }

        void updateKnownState(state cur_state){
            last_known_state = cur_state;
        }

    private:

        float dtime;

        state last_known_state;
        state cur_modeled_state;
        
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber sub_imu;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "DynModel");
  DynModel node(1);
  ros::spin();
  return 0;
}