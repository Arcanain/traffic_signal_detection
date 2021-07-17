#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char**argv)
{
	ros::init(argc, argv, "talker");
	
	RosWithClass ros_with_class;

	ros::Rate loop_rate(10);
	while(ros::ok()){
		ros_with_class.Publication();
		ros::spinOnce();
		loop_rate.sleep();
	}
}