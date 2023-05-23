#define PCL_NO_PRECOMPILE

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;
using namespace pcl;
using namespace ros;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

string pcd_filepath;
string priv_nh_str;
int lidar_idx = 0;
ros::Publisher picked_points_pub;
// Mutex: //
boost::mutex cloud_mutex;

struct callback_args{
    // structure used to pass arguments to the callback function
    CloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void
pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;
    if (event.getPointIndex() == -1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    // need to add removement of repeated point



    data->clicked_points_3d->points.push_back(current_point);
    // Draw clicked points in white:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> white(data->clicked_points_3d, 255, 255, 255);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, white, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    cout << "[" << priv_nh_str << "]";
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

void publishPC(const ros::Publisher& pub, const CloudT& cloud)
{
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(cloud, ros_msg);
    ros_msg.header.frame_id = "result";
    ros_msg.header.stamp = ros::Time::now();
    pub.publish(ros_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "selectPointLidar");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    priv_nh_str = priv_nh.getNamespace();
    
    bool auto_mode = false;
    priv_nh.param("auto_mode", auto_mode, false);
    if(auto_mode)
    {
        string start_flag_str;
        priv_nh.param<string>("start_flag", start_flag_str, "/pcdTransferFinished");
        bool start_proc = false;
        do
        {
            ros::param::param(start_flag_str, start_proc, false);
        } while (!start_proc);
        // ros::param::set(start_flag_str, false);
    }

    if(!priv_nh.getParam("pcd_filepath", pcd_filepath))
    {
        cerr <<"[" << priv_nh_str << "] Can not get the value of pcd_filepath" << endl;
        exit(1);
    }

    picked_points_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("picked_points", 10);

    CloudT::Ptr cloud(new CloudT);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("[" + priv_nh_str + "] viewer"));

    if(pcl::io::loadPCDFile(pcd_filepath, *cloud))
    {
        cerr <<"[" << priv_nh_str << "] ERROR: Cannot open file " << pcd_filepath << "! Aborting..." << endl;
        return 0;
    }
    // cout << "[" << priv_nh_str << "] cloud loaded size: " << cloud->points.size() << endl;
    cout << "[" << priv_nh_str << "] Initialized..." << endl;


    cloud_mutex.lock(); // for not overwriting the point cloud

    // Display pointcloud
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> i_color(cloud, "intensity");    
    viewer->addPointCloud<PointT>(cloud, i_color, "raw_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "raw_cloud");

    // Add point picking callback to viewer
    struct callback_args cb_args;
    CloudT::Ptr clicked_points_3d(new CloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
    cout << "[" << priv_nh_str << "] Shift+click on feature points, then press 'Q'..." << endl;

    // Spin until 'Q' is pressed
    // viewer->spin();
    // cout << "done." << endl;
    
    // cout << "Select " << cb_args.clicked_points_3d->points.size() << " points" << endl;
    // for(auto it = cb_args.clicked_points_3d->points.begin(); it < cb_args.clicked_points_3d->points.end(); it++)
    // {
    //     cout << it->x << " " << it->y << " " << it->z << endl;
    // }

    cloud_mutex.unlock();

    while (!viewer->wasStopped() && ros::ok())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        // if(cb_args.clicked_points_3d->points.size() >=3)
        cloud_mutex.lock();
        publishPC(picked_points_pub, *(cb_args.clicked_points_3d));
        cloud_mutex.unlock();
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