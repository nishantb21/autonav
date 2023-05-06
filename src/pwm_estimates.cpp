#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>

class PWMOdom {
    public:
        const static int vel_vec_len = 5;
        const static int steer_vec_len = 3;

        float vel_curve_x[vel_vec_len] = {1400, 1500, 1600, 1700, 1800};
        float vel_curve_y[vel_vec_len] = {-2.5, 0,    2.5,  3.5,  4.5};

        float steer_curve_x[steer_vec_len] = {1000, 1500, 2000};
        float steer_curve_y[steer_vec_len] = {0.785, 0, -0.785}; //45 deg in radians

        PWMOdom() : nh_("~") {
            wheel_base = .15; //check this?
            cur_vel = 0;

            // Initialize publisher
            pub_vel = nh_.advertise<std_msgs::Float32>("pwm_to_velocity", 10);
            pub_steering_ang = nh_.advertise<std_msgs::Float32>("pwm_to_steering_ang", 10);
            pub_twist = nh_.advertise<geometry_msgs::TwistStamped>("pwm_twist", 10);

            // Initialize subscriber
            sub_vel = nh_.subscribe("/velocity_raw", 10, &PWMOdom::callback_pwm_to_vel, this);
            sub_steering = nh_.subscribe("/servo_raw", 10, &PWMOdom::callback_pwm_to_steer, this);

        }   

        void callback_pwm_to_vel(const std_msgs::UInt32::ConstPtr& msg) {
            // Process message received on the subscriber
            // ROS_INFO_STREAM("Received message: " << msg->data);

            // Create and publish a response message
            std_msgs::Float32 response_msg;
            response_msg.data = interp(vel_curve_x, vel_curve_y, vel_vec_len, msg->data);

            cur_vel = response_msg.data;

            // ROS_INFO_STREAM("Received return from interp: " << response_msg.data);
            pub_vel.publish(response_msg);
            return;
        } 

        void callback_pwm_to_steer(const std_msgs::UInt32::ConstPtr& msg) {
            // Process message received on the subscriber
            //ROS_INFO_STREAM("callback_pwm_to_steer Received message: " << msg->data);

            // Create and publish a response message
            std_msgs::Float32 response_msg;
            response_msg.data = interp(steer_curve_x, steer_curve_y, steer_vec_len, msg->data);
            // ROS_INFO_STREAM("Received return from interp: " << response_msg.data);
            pub_steering_ang.publish(response_msg);

            float ang_vel = calcAngVel(response_msg.data);

            geometry_msgs::TwistStamped twist_msg;

            // linear vel
            twist_msg.twist.linear.x = cur_vel;
            twist_msg.twist.linear.y = 0;
            twist_msg.twist.linear.z = 0;

            //angular 
            twist_msg.twist.angular.y = 0;
            twist_msg.twist.angular.x = 0;
            twist_msg.twist.angular.z = ang_vel;
            
            //header
            twist_msg.header.frame_id = "base_link";
            twist_msg.header.stamp = ros::Time::now();
            
            pub_twist.publish(twist_msg); //publish the angular velocity based on velocity, wheel angle, and wheel base
            return;
        } 

        float interp(float* data_x, float* data_y, int len, float sample_x){
            // check if sample is out of bounds
            // ROS_INFO_STREAM("in interp() with sample=" << sample_x);
            if (sample_x <= data_x[0]){
                //ROS_INFO_STREAM("pwm_to_velocity: commanded pwm is below data range");
                return data_y[0];
            }else if (sample_x >= data_x[len-1]){
                //ROS_INFO_STREAM("pwm_to_velocity: commanded pwm is above data range");
                return data_y[len-1];
            }

            //iterate through until location of sample_x is found within data_x
            int left_ind = 1;
            while (data_x[left_ind] < sample_x){
                // ROS_INFO_STREAM("while loop: data_x[left_ind++]: " << data_x[left_ind]);
                left_ind++;
            }
            left_ind--;

            // use point slope form to get point
            float val =  (((data_y[left_ind]-data_y[left_ind+1])/(data_x[left_ind]-data_x[left_ind+1]))*(sample_x-data_x[left_ind])) + data_y[left_ind];
            //ROS_INFO_STREAM("sample_x: "<<sample_x<<", left_ind: " << left_ind<< ", val: " << val <<", data_y[left_ind]: " << data_y[left_ind] << ", data_y[left_ind+1]: " << data_y[left_ind+1] << ", data_x[left_ind]: " << data_x[left_ind] << ", data_x[left_ind+1]: " << data_x[left_ind+1]);
            return val;
        }

        float calcAngVel(float wheelAngle){
            float turn_radius = wheel_base/tan(wheelAngle);
            return cur_vel/turn_radius;
        }



    private:
        volatile float cur_vel;
        float wheel_base;

        ros::NodeHandle nh_;
        ros::Publisher pub_vel;
        ros::Publisher pub_twist;
        ros::Publisher pub_steering_ang;
        ros::Subscriber sub_vel;
        ros::Subscriber sub_steering;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pwm_estimates");
    PWMOdom node;
    ros::spin();
    return 0;
}
