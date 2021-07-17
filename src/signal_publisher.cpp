#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "signal_detector.h"

class SignalDetecorNode : public SignalDetector
{
private:
  ros::NodeHandle nh;
  ros::Publisher signal_pub;
  ros::Subscriber image_sub;
  cv_bridge::CvImagePtr cv_imgae;
public:
   SignalDetecorNode();
  ~SignalDetecorNode();
  void callback(const sensor_msgs::Image &msg_sub);
};

SignalDetecorNode::SignalDetecorNode()
{
	cout << "signal detection node start!" << endl;
    image_sub = nh.subscribe("/camera/color/image_raw", 10, &SignalDetecorNode::callback, this);
    signal_pub = nh.advertise<std_msgs::Int8>("signal_info", 10);
}

SignalDetecorNode::~SignalDetecorNode()
{
	cout << "signal detection node finish! Bye!" << endl;
}

void SignalDetecorNode::callback(const sensor_msgs::Image &data)
{
	std_msgs::Int8 state;
	cv_bridge::CvImagePtr cv_image;
    
	try {
        cv_image = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& ex) {
        ROS_ERROR("error");
        exit(-1);
	}

	// cv_image->imageがcv::Matフォーマット
	state.data = check_signal_state(cv_image->image);
	signal_pub.publish(state);
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "signal_publisher");
	
	//SignalDetector signaldetector;
	SignalDetecorNode signaldetecornode;

	ros::Rate loop_rate(10);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}