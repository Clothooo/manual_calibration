// manually select point in image
#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace cv;
using namespace ros;

string image_filepath;
string priv_nh_str;
ros::Publisher picked_points_pub;
boost::mutex image_mutex;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

CloudT::Ptr picked_pc_ptr;

struct callback_args{
    cv::Mat img;
    vector<cv::Point2i> clicked_cv_points;
    CloudT::Ptr clicked_points_3d;
    string win_name;
};

void drawClickPoint(cv::Mat& mat_, const cv::Point2i &center_)
{
    circle(mat_, center_, 3, Scalar(0,0,255),-1);
}

inline void myMouseEvent(int event, int x, int y, int flags, void* userdata)
{   
    // cout << "event: " << event << " flag: " << flags << endl;
    struct callback_args* data = (struct callback_args *)userdata;
    if(event == EVENT_LBUTTONDOWN && (flags == EVENT_FLAG_SHIFTKEY || flags == EVENT_FLAG_SHIFTKEY + EVENT_FLAG_ALTKEY))
    {
        Point2f P(x,y);
        data->clicked_cv_points.push_back(P);
        drawClickPoint(data->img, P);
        imshow(data->win_name, data->img);

        PointT tmp_P(x, y, 0.0);
        data->clicked_points_3d->points.push_back(tmp_P);

        std::cout << "[" << priv_nh_str << "] (" << P.x << ", " << P.y << ")" << std::endl;
    }
}

void publishPC(const ros::Publisher& pub, const CloudT& cloud)
{
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(cloud, ros_msg);
    ros_msg.header.frame_id = "result";
    ros_msg.header.stamp = ros::Time::now();
    pub.publish(ros_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "selectPointCamera");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    priv_nh_str = priv_nh.getNamespace();

    bool auto_mode = false;
    priv_nh.param("auto_mode", auto_mode, false);
    if(auto_mode)
    {
        string start_flag_str;
        priv_nh.param<string>("start_flag", start_flag_str, "/imgCaptureFinished");
        bool start_proc = false;
        do
        {
            ros::param::param(start_flag_str, start_proc, false);
        } while (!start_proc);
        // ros::param::set(start_flag_str, false);
    }

    if(!priv_nh.getParam("image_filepath", image_filepath))
    {
        cerr <<"[" << priv_nh_str << "] Can not get the value of image_filepath" << endl;
        exit(1);
    }

    picked_points_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("picked_points", 10);  

    cout << "[" << priv_nh_str << "] Initialized..." << endl;

    pcl::PointCloud<PointT>::Ptr clicked_points_3d(new pcl::PointCloud<PointT>);
    picked_pc_ptr = CloudT::Ptr(new CloudT);
    image_mutex.lock();

    cv::Mat image = imread(image_filepath);
    string win_title = "[" + priv_nh_str + "] image_view";

    struct callback_args cb_args;
    cb_args.img = image.clone();
    cb_args.win_name = win_title;
    cb_args.clicked_points_3d = clicked_points_3d;
    image_mutex.unlock();


    cout << "[" << priv_nh_str << "] Shift+click on feature points, then press 'Q'..." << endl;
    imshow(win_title, cb_args.img);
    setMouseCallback(win_title, myMouseEvent, (void*)&cb_args);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        int key = waitKey(10);
        // cout << "key: " << key << endl;

        if(key == 27 || key == 113 || key == 81) // `ESC` `q` `Q`
        {
            return 0;
        }

        image_mutex.lock();
        publishPC(picked_points_pub, *(cb_args.clicked_points_3d));
        loop_rate.sleep();
        image_mutex.unlock();
    }

    cout << "[" << priv_nh_str << "] done." << endl;
    cout << "[" << priv_nh_str << "] Select " << cb_args.clicked_points_3d->points.size() << " points" << endl;
    for(auto it = cb_args.clicked_points_3d->points.begin(); it < cb_args.clicked_points_3d->points.end(); it++)
    {
        cout << it->x << " " << it->y << " " << it->z << endl;
    }
    
    ros::shutdown();

    return 0;
}