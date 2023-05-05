#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>

//used for keeping track of the state of the robot
struct state{  
        float x;
        float y;
        float theta;

        // derivative states
        float x_dot;
        float y_dot;
        float theta_dot;

        //wheel angle
        float steering_angle;
};

class DynModel {
    public:
        DynModel(float dtime) : nh_("~") {
            this->dtime = dtime;
            this->reset = false;

            // Initialize publisher
            pub_ = nh_.advertise<std_msgs::String>("modeled_pose", 10);
            // topic to publish: predicted position of the robot
            
            // Initialize subscribers
            sub_imu = nh_.subscribe("/imu/data", 10, &DynModel::callback_imu, this);
            sub_known_state = nh_.subscribe("/known_state", 10, &DynModel::callback_known_state, this); 
            sub_velocity_raw = nh_.subscribe("/velocity_raw", 10, &DynModel::callback_imu, this);
            sub_servo_raw = nh_.subscribe("/servo_raw", 10, &DynModel::callback_imu, this);

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

        // update this->modeled_state.x/y/theta with pose
        void callback_known_state(const  geometry_msgs::Pose2D::ConstPtr& msg){
            modeled_state.x = msg->x;
            modeled_state.y = msg->y;
            modeled_state.theta = msg->theta;
            return;
        }

        // update this->modeled_state->x_dot with pose
        void callback_velocity_raw(const  std_msgs::UInt32::ConstPtr& msg){
            //modeled_state->x_dot = pwm2vel(msg->data);
            return;
        }

        void callbacK_servo_raw(const std_msgs::UInt32::ConstPtr& msg){
            //modeled_state->steering_angle = pwm2steeringang(msg->data);
            return;
        }

        float pwm2steeringang(std_msgs::UInt32 pwm){
            //TODO
            return float(pwm.data);
        }

        void updateKnownState(state cur_state){
            this->modeled_state = cur_state;
        }

        void modelDynamics(){
            while (ros::ok){

                if (reset){
                    this->modeled_state = resetState(this->modeled_state);
                }

            }
        }

        state resetState(state s){
            return s;
        }

    private:
        volatile bool reset;
        float dtime;
        float turn_radius; //how wide of a turn to take
        float turn_length; //how long of a turn to take

        state modeled_state;
        
        ros::NodeHandle nh_;
        ros::Publisher pub_;

        // subscribers
        ros::Subscriber sub_imu;
        ros::Subscriber sub_known_state;
        ros::Subscriber sub_velocity_raw;
        ros::Subscriber sub_servo_raw;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dyn_model");
  DynModel node(1);
  ros::spin();
  return 0;
}