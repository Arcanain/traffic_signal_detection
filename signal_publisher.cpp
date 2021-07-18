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

struct hsv_threshold
{
    unsigned char hue_lower;
    unsigned char hue_upper;
    unsigned char saturation;
    unsigned char value;
};

cv::Mat extract_color(cv::Mat image, hsv_threshold threshold)
{
    cv::Mat gaussian, hsv, channels[3], temp;
    cv::Mat hue, saturation, value, binary;
    cv::GaussianBlur(image, gaussian, cv::Size(11,11), 10, 10);
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    cv::split(hsv, channels);

    if( threshold.hue_lower <= threshold.hue_upper)
    {
        cv::threshold(channels[0], temp, threshold.hue_lower,  180, CV_THRESH_TOZERO);
        cv::threshold(temp, hue, threshold.hue_upper, 180, CV_THRESH_TOZERO_INV);
    }
    else
    {
        cv::Mat temp_1, temp_2;
        cv::threshold(channels[0], temp_1, threshold.hue_upper, 180, CV_THRESH_TOZERO_INV);
        cv::threshold(channels[0], temp_2, threshold.hue_lower, 180, CV_THRESH_TOZERO);
        cv::bitwise_or(temp_1, temp_2, hue);
    }

    cv::threshold(channels[1], saturation, threshold.saturation, 255, CV_THRESH_BINARY);
    cv::threshold(channels[2], value     , threshold.value,      255, CV_THRESH_BINARY);
    cv::bitwise_and(hue,  saturation, temp);
    cv::bitwise_and(temp, value,      binary);
    cv::dilate(binary, temp, cv::Mat(), cv::Point(-1, -1), 1);
    cv::erode(temp, binary, cv::Mat(), cv::Point(-1, -1), 1);

    return binary;
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "signal_publisher");
	
	//SignalDetector signaldetector;
	SignalDetecorNode signaldetecornode;
    
	// https://hmatsudaiac.wixsite.com/venus-robotix/cv-color-extraction-c
	hsv_threshold threshold_blue  = {90,  110, 180, 30};
    hsv_threshold threshold_green = {50,  80,  50,  130};
    hsv_threshold threshold_red   = {175, 5,   240, 110};

	//cv::Mat image = cv::imread("templates/red2green.png");
	//cv::Mat image = cv::imread("templates/歩行者信号機.jpg");
	//cv::Mat image = cv::imread("歩行者信号機.jpg", 1);
	//cv::Mat image = cv::imread("~/catkin_ws/src/traffic_signal_detection/templates/red2black.png");
    std::string homepath = std::getenv("HOME");
    cv::Mat image = cv::imread(homepath + "/catkin_ws/src/traffic_signal_detection/templates/red2black.png", 0);
    if (image.empty()) {
        ROS_ERROR("map: unable to open the map");
    }

    /*
	if (image.empty()) {
		cout << "入力画像が見つかりません" << endl;
		return -1;
	}
	*/

    /*
	cv::Mat binary_blue, binary_green, binary_red;
	binary_blue  = extract_color(image, threshold_blue);
    binary_green = extract_color(image, threshold_green);
    binary_red   = extract_color(image, threshold_red);     
	*/
	cout << image.channels() << image.dims << image.elemSize1() << endl;
	cout << image.cols << image.rows << endl;

	//cout << binary_blue << endl;

	ros::Rate loop_rate(10);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}