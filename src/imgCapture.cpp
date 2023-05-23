// capture single frame image
#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <ctime>

using namespace std;
using namespace cv;
using namespace ros;

string priv_nh_str, image_tp, output_path;
cv::Mat image;

void callback(const sensor_msgs::ImageConstPtr image_msg)
{
    cv::Mat input_image;
    cv_bridge::CvImagePtr cv_ptr;

    try{
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e){
        ROS_ERROR_STREAM("cv_bridge Exception:" << e.what());
        return;
    }
    input_image = cv_ptr->image;
    image = input_image.clone();
    
    cv::imshow("image", image);
    return;
}

void getParameters() {
    // cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("~image_tp", image_tp)) {
        cout << "Can not get the value of image_tp" << endl;
        exit(1);
    }
    if (!ros::param::get("~output_img_path", output_path)) {
        cout << "Can not get the value of output_img_path" << endl;
        exit(1);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imgCapture");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    priv_nh_str = priv_nh.getNamespace();
    getParameters();

    ros::Subscriber sub = priv_nh.subscribe(image_tp, 1, callback);

    cout << "[" << priv_nh_str << "] Initialized......" << endl;
    cout << "Press 's' or 'S' to capture image. 'Esc', 'Q' or 'q' to exit." << endl;

    mutex mtx;
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        spinOnce();
        loop_rate.sleep();
        
        int key = cv::waitKey(10);
        if(key == 27 || key == 113 || key == 81) // `ESC` `q` `Q`
        {
            cv::destroyAllWindows();
            break;
        }
        else if(key == 83 || key == 115) // 'S' 's'
        {
            mtx.lock();
            string outputName = output_path + ".png";
            time_t rawtime;
            struct tm *ptminfo;
            time(&rawtime);
            ptminfo = localtime(&rawtime);
            string outputNameTime = output_path + "_" + std::to_string(ptminfo->tm_year + 1900) + std::to_string(ptminfo->tm_mon + 1) + std::to_string(ptminfo->tm_mday) + std::to_string(ptminfo->tm_hour) + std::to_string(ptminfo->tm_min) + std::to_string(ptminfo->tm_sec) + ".png";

            cv::imwrite(outputName, image);
            cv::imwrite(outputNameTime, image);
            cout << "<<<<< [" << priv_nh_str << "] image saved!" << endl;
            cv::destroyAllWindows();
            break;
        }
    }

    ros::param::set("~/imgCaptureFinished", true);
    // cout << "imgCapture End" << endl;
    ros::shutdown();
    return 0;
}