#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>

class PWM2Velocity {
    public:
        const static int vec_len = 5;
        float curve_x[vec_len] = {1400, 1500, 1600, 1700, 1800};
        float curve_y[vec_len] = {-2.5, 0,    2.5,  3.5,  4.5};

        PWM2Velocity() : nh_("~") {
                // Initialize publisher
                pub_ = nh_.advertise<std_msgs::Float32>("pwm_to_velocity", 10);

                // Initialize subscriber
                sub_ = nh_.subscribe("/velocity_raw", 10, &PWM2Velocity::callback, this);
        }

        void callback(const std_msgs::UInt32::ConstPtr& msg) {
            // Process message received on the subscriber
            // ROS_INFO_STREAM("Received message: " << msg->data);

            // Create and publish a response message
            std_msgs::Float32 response_msg;
            response_msg.data = interp(curve_x,curve_y,vec_len,msg->data);
            pub_.publish(response_msg);
        }

        float interp(float* data_x, float* data_y, int len, float sample_x){
            // check if sample is out of bounds
            if (sample_x <= data_x[0]){
                ROS_INFO_STREAM("sample is less than data_x[0]. sample_x = " << sample_x);
                return data_x[0];
            }else if (sample_x >= data_x[len]){
                return data_x[len];
            }

            //iterate through until location of sample_x is found within data_x
            int left_ind = 0;
            for(left_ind=0; data_x[left_ind] < sample_x; left_ind++);

            // use point slope form to get point
            return ((data_y[left_ind+1]-data_y[left_ind])/(data_x[left_ind+1]-data_x[left_ind]))*(sample_x-data_x[left_ind]) + data_y[left_ind];
        }



    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pwm_to_velocity");
    PWM2Velocity node;
    ros::spin();
    return 0;
}