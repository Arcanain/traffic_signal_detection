#include <ros/ros.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/core/eigen.hpp>
//#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class SignalDetector
{
private:
    cv::Mat temp0;
    cv::Mat temp1;
    cv::Mat temp2;
    cv::Mat temp3;

    int temp0_h;
    int temp0_w;
    int temp3_h;
    int temp3_w;

    cv::Mat frame4;
    cv::Mat frame3;
    cv::Mat frame2;
    cv::Mat frame1;

    int ok_desu;
    cv::Mat output;

    int green_cnt;
    int red_cnt;

    float green_th;
    float red_th;

    int nd;
    int state;

    cv::Mat new_frame;
    cv::Mat display_frame;

    cv::Mat result0;
    cv::Mat result1;
    cv::Mat result2;
    cv::Mat result3;

    cv::Point max_pt0;
    cv::Point max_pt1;
    cv::Point max_pt2;
    cv::Point max_pt3;

    double maxVal0;
    double maxVal1;
    double maxVal2;
    double maxVal3;
public:
    SignalDetector();
    ~SignalDetector();
    cv::Mat diffimg(cv::Mat after, cv::Mat before);
    int check_signal_state(cv::Mat frame);
};

SignalDetector::SignalDetector()
{
    cout << "signal detection start!" << endl;

    // load template images
    temp0 = cv::imread("templates/red2green.png");
    temp1 = cv::imread("templates/red2black.png");
    temp2 = cv::imread("templates/black2green.png");
    temp3 = cv::imread("templates/green2red.png");
    
    temp0_h = temp0.cols;
    temp0_w = temp0.rows;
    temp3_h = temp3.cols;
    temp3_w = temp3.rows;

    /*
    frame4 = 0;
    frame3 = 0;
    frame2 = 0;
    frame1 = 0;
    */

    ok_desu = 0;
    //output = cv::image(600, 600, CV_LOAD_IMAGE_COLOR);

    green_cnt = 0;
    red_cnt = 0;

    // threshold of template matching
    green_th = 0.65; // stable:0.65
    red_th = 0.7; // stable:0.7

    // number of detections
    nd = 3;

    // signal states (0:unkown, 1:green, 2:red)
    state = 0;
}

SignalDetector::~SignalDetector()
{
    cout << "signal detection finish! Bye!" << endl;
}


// generating diff image
cv::Mat SignalDetector::diffimg(cv::Mat after, cv::Mat before) {
    /*
    blue_before  = before[:,:,0];
    green_before = before[:,:,1];
    red_before   = before[:,:,2];

    blue_after  = after[:,:,0];
    green_after = after[:,:,1];
    red_after   = after[:,:,2];
    */
}

/*
def diffimg(self, after, before):

    blue_before  = before[:,:,0]
    green_before = before[:,:,1]
    red_before   = before[:,:,2]

    blue_after  = after[:,:,0]
    green_after = after[:,:,1]
    red_after   = after[:,:,2]

    blue_final  = blue_after.astype(np.int32) - blue_before.astype(np.int32) + 255
    green_final = green_after.astype(np.int32)- green_before.astype(np.int32) + 255
    red_final   = red_after.astype(np.int32) - red_before.astype(np.int32) + 255

    output=np.zeros((after.shape[0],after.shape[1],3))
    output[:,:,0]=blue_final
    output[:,:,1]=green_final
    output[:,:,2]=red_final

    return (output/2.0).astype(np.uint8)
*/

int SignalDetector::check_signal_state(cv::Mat frame)
{
    new_frame = frame;
    display_frame = new_frame.clone();
    
    if (ok_desu > 4) {
        output = diffimg(new_frame, frame4);

        // テンプレートと，それに重なった画像領域とを比較
        // http://opencv.jp/opencv-2svn/cpp/imgproc_object_detection.html
        cv::matchTemplate(output, temp0, result0, cv::TM_CCOEFF_NORMED);
        cv::matchTemplate(output, temp1, result1, cv::TM_CCOEFF_NORMED);
        cv::matchTemplate(output, temp2, result2, cv::TM_CCOEFF_NORMED);
        cv::matchTemplate(output, temp3, result3, cv::TM_CCOEFF_NORMED);
        
        // 最大のスコアの場所を探す
        cv::Rect roi_rect0(0, 0, temp0.cols, temp0.rows);
        cv::Rect roi_rect1(0, 0, temp1.cols, temp1.rows);
        cv::Rect roi_rect2(0, 0, temp2.cols, temp2.rows);
        cv::Rect roi_rect3(0, 0, temp3.cols, temp3.rows);

        // 配列全体あるいは部分配列に対する，大域的最小値および最大値を求める
        cv::minMaxLoc(result0, NULL, &maxVal0, NULL, &max_pt0);
        cv::minMaxLoc(result1, NULL, &maxVal1, NULL, &max_pt1);
        cv::minMaxLoc(result2, NULL, &maxVal2, NULL, &max_pt2);
        cv::minMaxLoc(result3, NULL, &maxVal3, NULL, &max_pt3);

        // condition1: RED to GREEN
        if ( (maxVal0 > green_th) && (maxVal0 > maxVal2) ) {
            green_cnt += 1;
        } else {
            green_cnt = 0;
        }

        if (green_cnt == nd) {
            state = 1;
            cout << "The signal turned from RED to GREEN !" << endl;

            roi_rect0.x = max_pt0.x;
            roi_rect0.y = max_pt0.y;
        
            // 探索結果の場所に矩形を描画
            cv::rectangle(display_frame, roi_rect0, cv::Scalar(0,0,255), 3);

            green_cnt = 0;
        }

        // condition2: GREEN to RED
        if ( (maxVal3 > red_th) && (maxVal3 > maxVal1) ) {
            red_cnt += 1;
        } else {
            red_cnt = 0;
        }

        if (red_cnt == nd) {
            state = 2;
            cout << "The signal turned from GREEN to RED !" << endl;

            roi_rect3.x = max_pt3.x;
            roi_rect3.y = max_pt3.y;
        
            // 探索結果の場所に矩形を描画
            cv::rectangle(display_frame, roi_rect3, cv::Scalar(0,0,255), 3);

            red_cnt = 0;
        }

        // display the resulting frame
        cv::imshow("frame",display_frame);
        cv::imshow("diff4",output);
        cv::waitKey(3);
    }

    frame4 = frame3;
    frame3 = frame2;
    frame2 = frame1;
    frame1 = new_frame;
    ok_desu += 1;

    return state;
}

/*
int main(int argc, char**argv)
{
	ros::init(argc, argv, "signal_publisher");
	
	SignalDetector signaldetector;

    
	ros::Rate loop_rate(10);
	while(ros::ok()){
		//ros_with_class.Publication();
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
*/