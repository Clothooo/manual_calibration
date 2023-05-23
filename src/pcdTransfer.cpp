#define PCL_NO_PRECOMPILE

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <ctime>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;
using namespace ros;
using namespace sensor_msgs;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

string lidar_tp, output_path;
int threshold_lidar, data_num;
int acc_count = 0;
CloudT::Ptr acc_pc(new CloudT);

void callback_pc(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_in)
{
    CloudT::Ptr cloud_in(new CloudT);
    fromROSMsg(*cloud_msg_in, *cloud_in);

    *acc_pc+=*cloud_in;
    ++acc_count;
}

void writeTitle(const string& filename, int point_num)
{
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else {
        outfile << "# .PCD v.7 - Point Cloud Data file format" << endl;
        outfile << "VERSION .7" << endl;
        outfile << "FIELDS x y z intensity" << endl;
        outfile << "SIZE 4 4 4 4" << endl;
        outfile << "TYPE F F F F" << endl;
        outfile << "COUNT 1 1 1 1" << endl;
        outfile << "WIDTH " << to_string(point_num) << endl;
        outfile << "HEIGHT 1" << endl;
        outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
        outfile << "POINTS " << to_string(point_num) << endl;
        outfile << "DATA ascii" << endl;
    }
    ROS_INFO("Save file %s", filename.c_str());
}

void writePointCloud(const string& filename, const CloudT& pc)
{
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else {
        for(auto it = pc.points.begin(); it < pc.points.end(); it++)
        {
            outfile << to_string(it->x) << " " << to_string(it->y) << " " << to_string(it->z) << " " << to_string(it->intensity) << endl;
        }
    }
}

void dataSave(string& path, CloudT& cloud)
{
    string outputName = path + ".pcd";
    time_t rawtime;
    struct tm *ptminfo;
    time(&rawtime);
    ptminfo = localtime(&rawtime);
    string outputNameTime = path + "_" + std::to_string(ptminfo->tm_year + 1900) + std::to_string(ptminfo->tm_mon + 1) + std::to_string(ptminfo->tm_mday) + std::to_string(ptminfo->tm_hour) + std::to_string(ptminfo->tm_min) + std::to_string(ptminfo->tm_sec) + ".pcd";
    // writeTitle(outputName, acc_pc->size());
    // writePointCloud(outputName, *acc_pc);
    pcl::io::savePCDFileASCII(outputName, cloud);
    pcl::io::savePCDFileASCII(outputNameTime, cloud);
    acc_pc->clear();
}




void getParameters() {
    // cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("~lidar_tp", lidar_tp)) {
        cout << "Can not get the value of lidar_tp" << endl;
        exit(1);
    }
    if (!ros::param::get("~output_pcd_path", output_path)) {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("~threshold_lidar", threshold_lidar)) {
        cout << "Can not get the value of threshold_lidar" << endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcdTransfer");
    ros::NodeHandle priv_nh("~");
    getParameters();

    ros::Subscriber sub = priv_nh.subscribe(lidar_tp, 5, callback_pc);

    string priv_nh_str = priv_nh.getNamespace();
    cout << "[" << priv_nh_str << "] Initialized......" << endl;
    
    ros::Rate loop_rate(10);
    int count = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if(acc_count >= threshold_lidar)
        {
            dataSave(output_path, *acc_pc);
            cout << "<<<<< [" << priv_nh_str << "] pcd file saved!" << endl;
            break;
        }
    }

    ros::param::set("~/pcdTransferFinished", true);
    ros::shutdown();
    return 0;
}