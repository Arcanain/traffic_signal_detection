#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <string>

void callback(const std_msgs::Int8::ConstPtr& state)
{
    std::string signal_state;
    if (state->data == 0) {
        signal_state = "UNKOWN";
    } else if (state->data == 1) {
        signal_state = "GREEN";
    } else if (state->data == 2) {
        signal_state = "RED";
    }
    
    std::string message = " signal state : " + signal_state;
    //ROS_INFO("I heard: [%s]", message);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "signal_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("signal_info", 10, callback);

    ros::spin();

    return 0;
}