#define PCL_NO_PRECOMPILE

#include <thread>
#include <mutex>
#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>	
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace Eigen;
using namespace ros;
using namespace pcl;
using namespace sensor_msgs;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

bool cloud_updated = false, to_show = false;
CloudT::Ptr cloud;
string lidar_tp;

void PointViz()
{
    cout << "[PointViz]...." << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("aligned_viewer"));
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> i_color(cloud, "intensity");    
    viewer->addPointCloud<PointT>(cloud, i_color, "aligned_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "aligned_cloud");

    while(!viewer->wasStopped() && ros::ok())
    {
        if(cloud_updated)
        {
            pcl::visualization::PointCloudColorHandlerGenericField<PointT> i_color_(cloud, "intensity"); 
            viewer->updatePointCloud<PointT>(cloud, i_color_, "aligned_cloud");

            cout << "[viewer] cloud updated!" << endl;
            cout << "cloud.size = " << cloud->points.size() << endl;
            cloud_updated = false;
            ros::param::set("/to_update", cloud_updated);
        }
        viewer->spinOnce(10);
    }
}

void callback_pc(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_in)
{
    CloudT::Ptr cloud_in(new CloudT);
    fromROSMsg(*cloud_msg_in, *cloud_in);
    *cloud = *cloud_in;
    cout << "[callback] cloud.size = " << cloud->points.size() << endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "threadtest");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    if (!ros::param::get("~lidar_tp", lidar_tp)) {
        cout << "Can not get the value of lidar_tp" << endl;
        exit(1);
    }
    else {
        cout << "lidar_tp" << lidar_tp << endl;
    }

    cloud = CloudT::Ptr(new CloudT);
    ros::Subscriber sub = priv_nh.subscribe(lidar_tp, 5, callback_pc);


    cout << "initialized......" << endl;
    
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::param::get("/to_show", to_show);
        ros::param::get("/to_update", cloud_updated);
        ros::spinOnce();
        loop_rate.sleep();
        if(to_show)
        {
            cout << "to_show" << endl;
            std::thread th(PointViz);
            th.detach();
            to_show = false;
            ros::param::set("/to_show", false);
        }

    }
    ros::shutdown();
    return 0;
}