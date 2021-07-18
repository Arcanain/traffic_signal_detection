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
    
    /*
	// https://hmatsudaiac.wixsite.com/venus-robotix/cv-color-extraction-c
	hsv_threshold threshold_blue  = {90,  110, 180, 30};
    hsv_threshold threshold_green = {50,  80,  50,  130};
    hsv_threshold threshold_red   = {175, 5,   240, 110};
    */

    std::string homepath = std::getenv("HOME");
    //cv::Mat image = cv::imread(homepath + "/catkin_ws/src/traffic_signal_detection/templates/red2black.png");
    cv::Mat image = cv::imread(homepath + "/catkin_ws/src/traffic_signal_detection/templates/歩行者信号機.jpg");
    if (image.empty()) {
        ROS_ERROR("map: unable to open the map");
    }
	
    hsv_threshold threshold_blue  = {90,  110, 180, 30};
    hsv_threshold threshold_green = {50,  80,  50,  130};
    hsv_threshold threshold_red   = {175, 5,   240, 110};

    // before
	cv::Mat binary_blue_before  = signaldetecornode.extract_color(image, threshold_blue);
    cv::Mat binary_green_before = signaldetecornode.extract_color(image, threshold_green);
    cv::Mat binary_red_before   = signaldetecornode.extract_color(image, threshold_red);   
    
    // after
    cv::Mat binary_blue_after  = signaldetecornode.extract_color(image, threshold_blue);
    cv::Mat binary_green_after = signaldetecornode.extract_color(image, threshold_green);
    cv::Mat binary_red_after   = signaldetecornode.extract_color(image, threshold_red);  
    
    cv::Mat blue_final  = binary_blue_after - binary_blue_before + 255;
    cv::Mat green_final = binary_green_after- binary_green_before + 255;
    cv::Mat red_final   = binary_red_after - binary_red_before + 255;
    
    // 別のパターン
    std::vector<cv::Mat> after;
    cv::split(image, after);

    std::vector<cv::Mat> before;
    cv::split(image, before);

    cv::Mat blue_diff = after[0] - before[0] + 255;
    cv::Mat green_diff = after[1] - before[1] + 255;
    cv::Mat red_diff = after[2] - before[2] + 255;    
    
    vector<cv::Mat> output2;
    cv::Mat test2;
    output2.push_back( blue_diff );
	output2.push_back( green_diff );
	output2.push_back( red_diff );
    cv::merge(output2, test2);
    cv::imwrite("fuga2.png", test2);

    //cout << binary_blue_before << endl;
    //cout << binary_green_before << endl;
    //cout << binary_red_before << endl;

    // 3つのチャネルB, G, Rに分離 (OpenCVではデフォルトでB, G, Rの順)
    // planes[0],planes[1],planes[2]に B・G・R が格納
    // https://minus9d.hatenablog.com/entry/20130204/1359989829
    // https://koshinran.hateblo.jp/entry/2018/03/11/231901    
    // 統合
    vector<cv::Mat> output;
    cv::Mat test;
    output.push_back( blue_final );
	output.push_back( green_final );
	output.push_back( red_final );
    cv::merge(output, test);
    cv::imwrite("fuga.png", test);

    //cout << image.depth() << endl;

	ros::Rate loop_rate(10);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}