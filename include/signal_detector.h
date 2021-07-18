#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

struct hsv_threshold
{
    unsigned char hue_lower;
    unsigned char hue_upper;
    unsigned char saturation;
    unsigned char value;
};

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
    
    hsv_threshold threshold_blue;
    hsv_threshold threshold_green;
    hsv_threshold threshold_red;

    cv::Mat binary_blue_before;
    cv::Mat binary_green_before;
    cv::Mat binary_red_before;

    cv::Mat binary_blue_after;
    cv::Mat binary_green_after;
    cv::Mat binary_red_after;

    cv::Mat blue_final;
    cv::Mat green_final;
    cv::Mat red_final;
public:
    SignalDetector();
    ~SignalDetector();
    cv::Mat diffimg(cv::Mat after, cv::Mat before);
    cv::Mat extract_color(cv::Mat image, hsv_threshold threshold);
    int check_signal_state(cv::Mat frame);
};

SignalDetector::SignalDetector()
{
    // load template images
    std::string homepath = std::getenv("HOME");
    temp0 = cv::imread(homepath + "/catkin_ws/src/traffic_signal_detection/templates/red2green.png");
    temp1 = cv::imread(homepath + "/catkin_ws/src/traffic_signal_detection/templates/red2black.png");
    temp2 = cv::imread(homepath + "/catkin_ws/src/traffic_signal_detection/templates/black2green.png");
    temp3 = cv::imread(homepath + "/catkin_ws/src/traffic_signal_detection/templates/green2red.png");
    if (temp0.empty() && temp1.empty() && temp2.empty() && temp3.empty()) {
        ROS_ERROR("map: unable to open the map");
    }
    
    temp0_h = temp0.cols;
    temp0_w = temp0.rows;
    temp3_h = temp3.cols;
    temp3_w = temp3.rows;

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

    cout << "signal detection start!" << endl;
}

SignalDetector::~SignalDetector()
{
    cout << "signal detection finish! Bye!" << endl;
}


// generating diff image
cv::Mat SignalDetector::diffimg(cv::Mat after, cv::Mat before) {
    // https://hmatsudaiac.wixsite.com/venus-robotix/cv-color-extraction-c
	threshold_blue  = {90,  110, 180, 30};
    threshold_green = {50,  80,  50,  130};
    threshold_red   = {175, 5,   240, 110};
    
    // before
	binary_blue_before  = extract_color(before, threshold_blue);
    binary_green_before = extract_color(before, threshold_green);
    binary_red_before   = extract_color(before, threshold_red);     

    // after
    binary_blue_after  = extract_color(after, threshold_blue);
    binary_green_after = extract_color(after, threshold_green);
    binary_red_after   = extract_color(after, threshold_red);  
    
    blue_final  = binary_blue_after - binary_blue_before + 255;
    green_final = binary_green_after- binary_green_before + 255;
    red_final   = binary_red_after - binary_red_before + 255;
    
    //prepare the output image:
    //cv::Mat imageReflectionFinal = cv::Mat::zeros( .size(), testMat.type() );
    // VGA(640x480)の配列を確保　
    // CV_8UC3は配列の要素の種類　8U：1Byte(8bit)，C3：3ch（つまりカラー画像）
    //output = cv::Mat::zeros(Size(after.cols, after.rows), CV_8UC3);

    // 統合
    vector<cv::Mat> output;
    cv::Mat dst;
    output.push_back( blue_final );
	output.push_back( green_final );
	output.push_back( red_final );
    cv::merge(output, dst);
    cv::imwrite("fuga2.png", dst);

    return dst;
    /*
    for (int v = 0; v < img.size().height; v++) {
        for (int u = 0; u < img.size().width; u++) {
            output.at<Vec3b>(v, u) = Vec3b(50, 100, 150); // B:50 G:100 R:150
            output.at<Vec3b>(v, u)[0] = blue_final;
        }
    }
    */

    // https://www.atmarkit.co.jp/ait/articles/1608/17/news115.html
    //output.val[0] = ;
    
    /*
    image.at<cv::Vec3b>[0] //Blue
    image.at<cv::Vec3b>[1] //Green
    image.at<cv::Vec3b>[2] //Red
    */

    //output = np.zeros((after.shape[0],after.shape[1],3));

    /*
    blue_before  = before[:,:,0];
    green_before = before[:,:,1];
    red_before   = before[:,:,2];

    blue_after  = after[:,:,0];
    green_after = after[:,:,1];
    red_after   = after[:,:,2];

    blue_final  = blue_after - blue_before + 255;
    green_final = green_after- green_before + 255;
    red_final   = red_after - red_before + 255;

    output=np.zeros((after.shape[0],after.shape[1],3));
    output[:,:,0]=blue_final;
    output[:,:,1]=green_final;
    output[:,:,2]=red_final;

    return (output/2.0).astype(np.uint8);
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

cv::Mat SignalDetector::extract_color(cv::Mat image, hsv_threshold threshold)
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
