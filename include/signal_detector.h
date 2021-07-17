#ifndef _SIGNAL_DETECTOR_H_
#define _SIGNAL_DETECTOR_H_

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

class SignalDetector
{
private:
    cv::Mat temp0;
    cv::Mat temp1;
    cv::Mat temp2;
    cv::Mat temp3;

    int frame4;
    int frame3;
    int frame2;
    int frame1;

    int ok_desu;
    cv::Mat output= image(600, 600, CV_LOAD_IMAGE_COLOR);

    int green_cnt;
    int red_cnt;

    float green_th;
    float red_th;

    int nd;
    int state;
public:
    SignalDetector();
    ~SignalDetector();
};

SignalDetector::SignalDetector()
{
    // load template images
    temp0 = cv2.imread('templates/red2green.png')
    temp1 = cv2.imread('templates/red2black.png')
    temp2 = cv2.imread('templates/black2green.png')
    temp3 = cv2.imread('templates/green2red.png')
    
    temp0_h = temp0.cols;
    temp0_w = temp0.rows;
    temp3_h = temp3.cols;
    temp3_w = temp3.rows;

    frame4 = 0;
    frame3 = 0;
    frame2 = 0;
    frame1 = 0;

    ok_desu = 0;
    //output=np.zeros((600,600,3))

    green_cnt = 0;
    red_cnt = 0;

    // threshold of template matching
    green_th = 0.65 // stable:0.65
    red_th = 0.7 // stable:0.7

    // number of detections
    nd = 3

    // signal states (0:unkown, 1:green, 2:red)
    state = 0

}

SignalDetector::~SignalDetector()
{
}

#endif // _SIGNAL_DETECTOR_H_