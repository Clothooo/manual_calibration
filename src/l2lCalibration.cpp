#define PCL_NO_PRECOMPILE

#include <thread>
#include <mutex>
#include <stdio.h>
#include <string>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>	
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#define DEBUG 0

using namespace std;
using namespace Eigen;
using namespace ros;
using namespace pcl;
using namespace sensor_msgs;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

string lidar1_point_tp, lidar2_point_tp;
string lidar1_pcd_filepath, lidar2_pcd_filepath, exParam_filepath;

bool lidar1_recieved, lidar2_recieved = false;
int lidar1_pre_size, lidar2_pre_size = 0;
CloudT::Ptr lidar1_pc, lidar2_pc;
Eigen::Matrix4d TR_1to2;
bool ext_got = false, ext_updated = false, to_show_fusion_cloud = true;
vector<double> align_rmse;

vector<double> calMacthingErr(CloudT& cloud1, CloudT& cloud2, Eigen::Matrix4d& TR_1to2);
vector<double> calExtrinsicParam(CloudT& feature_pc_1, CloudT& feature_pc_2, Eigen::Matrix4d& TR_1to2);

class CloudVisual
{
private:
    CloudT::Ptr cloud1, cloud2;
    bool to_update = false;
public:
    CloudVisual(){
        cloud1 = CloudT::Ptr(new CloudT);
        cloud2 = CloudT::Ptr(new CloudT);
    }
    CloudVisual(CloudT::Ptr cloud1_, CloudT::Ptr cloud2_):cloud1(cloud1_), cloud2(cloud2_){}
    void setCloud12(CloudT::Ptr cloud1_, CloudT::Ptr cloud2_){
        cloud1 = cloud1_;
        cloud2 = cloud2_;
    }
    bool getToUpdate(){return to_update;}
    void updatePointCloud(CloudT::Ptr& cloud1_new, CloudT::Ptr& cloud2_new){
        cloud1 = cloud1_new;
        cloud2 = cloud2_new;
        to_update = true;
    }
    void showPointCloud()
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("aligned_viewer"));
        pcl::visualization::PointCloudColorHandlerCustom<PointT> white(cloud1, 255, 255, 255);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud2, 255, 0, 0);
        viewer->addPointCloud<PointT>(cloud1, white, "aligned_cloud1");
        viewer->addPointCloud<PointT>(cloud2, red, "aligned_cloud2");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned_cloud1");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned_cloud2");

        while(!viewer->wasStopped() && ros::ok())
        {
            if(to_update)
            {
                pcl::visualization::PointCloudColorHandlerCustom<PointT> white_(cloud1, 255, 255, 255);
                pcl::visualization::PointCloudColorHandlerCustom<PointT> red_(cloud2, 255, 0, 0);
                viewer->updatePointCloud<PointT>(cloud1, white_, "aligned_cloud1");
                viewer->updatePointCloud<PointT>(cloud2, red_, "aligned_cloud2");

                cout << "[viewer] cloud updated!" << endl;
                to_update = false;
            }
            viewer->spinOnce(100);
        }
    }
};

class excalib3d_ceres
{
private:
    PointT target_p, source_p;
public:
    excalib3d_ceres(PointT target_p_, PointT source_p_);
    excalib3d_ceres(){};
    
    template <typename T>
    bool operator()(const T *_q, const T *_t, T *residuals)const{
        Eigen::Quaternion<T> q_incre{_q[ 3 ], _q[ 0 ], _q[ 1 ], _q[ 2 ]};
        Eigen::Matrix<T, 3, 1> t_incre{_t[ 0 ], _t[ 1 ], _t[ 2 ]};

        Eigen::Matrix<T, 3, 1> src_p(T(source_p.x), T(source_p.y), T(source_p.z));
        Eigen::Matrix<T, 3, 1> src_p_tgt = q_incre.toRotationMatrix() * src_p + t_incre;

        residuals[0] = src_p_tgt[0] - T(target_p.x);
        residuals[1] = src_p_tgt[1] - T(target_p.y);
        residuals[2] = src_p_tgt[2] - T(target_p.z);

        return true;
    }

    static ceres::CostFunction *Create(PointT target_p_, PointT source_p_)
    {
        return (new ceres::AutoDiffCostFunction<excalib3d_ceres, 3, 4, 3>(new excalib3d_ceres(target_p_, source_p_)));
    }
};

excalib3d_ceres::excalib3d_ceres(PointT target_p_, PointT source_p_)
{
    target_p = target_p_;
    source_p = source_p_;
}

vector<double> calMacthingErr(CloudT& cloud1, CloudT& cloud2, Eigen::Matrix4d& TR_1to2)
{
    double esum_x = 0, esum_y = 0, esum_z = 0;
    double rmse_total = 0, rmse_x = 0, rmse_y = 0, rmse_z = 0;
    CloudT::Ptr cloud1_trans_ptr(new CloudT);
    pcl::transformPointCloud(cloud1, *cloud1_trans_ptr, TR_1to2);
    size_t pc1_size = cloud1.points.size();
    size_t pc2_size = cloud2.points.size();
    size_t point_size = pc1_size < pc2_size ? pc1_size : pc2_size;
    for(int i = 0; i < point_size; i++)
    {
        esum_x += pow(cloud2.points[i].x - cloud1_trans_ptr->points[i].x, 2);
        esum_y += pow(cloud2.points[i].y - cloud1_trans_ptr->points[i].y, 2);
        esum_z += pow(cloud2.points[i].z - cloud1_trans_ptr->points[i].z, 2);
    }
    rmse_x = sqrt(esum_x/double(point_size));
    rmse_y = sqrt(esum_y/double(point_size));
    rmse_z = sqrt(esum_z/double(point_size));
    rmse_total = sqrt(pow(rmse_x, 2) + pow(rmse_y, 2) + pow(rmse_z, 2));

    vector<double> rmse = {rmse_x, rmse_y, rmse_z, rmse_total};
    return rmse;
}

vector<double> calExtrinsicParam(CloudT& feature_pc_1, CloudT& feature_pc_2, Eigen::Matrix4d& TR_1to2)
{
    TR_1to2 = Eigen::Matrix4d::Identity();
    int size_p1 = feature_pc_1.points.size();
    int size_p2 = feature_pc_2.points.size();
    int size_p = size_p1 <= size_p2 ? size_p1 : size_p2;
    if(DEBUG) cout << "size_p = " << size_p << endl;

    // ICP REGISTRATION
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Quaterniond q(R);
    double ext[7];
    ext[0] = q.x();
    ext[1] = q.y();
    ext[2] = q.z();
    ext[3] = q.w();
    ext[4] = 0;  
    ext[5] = 0;  
    ext[6] = 0;

    Eigen::Map<Eigen::Quaterniond> m_q = Eigen::Map<Eigen::Quaterniond>(ext);
    Eigen::Map<Eigen::Vector3d> m_t = Eigen::Map<Eigen::Vector3d>(ext + 4);

    if(DEBUG){
    cout << "init rot = \n" << m_q.toRotationMatrix() << endl;
    cout << "init m_t = " << m_t << endl;
    }

    ceres::LocalParameterization * q_parameterization = new ceres::EigenQuaternionParameterization();
    ceres::Problem problem;

    problem.AddParameterBlock(ext, 4, q_parameterization);
    problem.AddParameterBlock(ext + 4, 3);
    for(int val = 0; val < size_p; val++)
    {
        ceres::CostFunction *cost_function;
        cost_function = excalib3d_ceres::Create(feature_pc_2.points[val], feature_pc_1.points[val]);
        problem.AddResidualBlock(cost_function, NULL, ext, ext + 4);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if(DEBUG) cout << summary.BriefReport() << endl;

    Eigen::Matrix3d rot = m_q.toRotationMatrix();

    TR_1to2.block(0,0,3,3) = rot;
    TR_1to2.topRightCorner(3,1) = m_t;
    // cout << "Tr_1to2 = " << endl << TR_1to2 << endl;

    // MATCHING ERROR
    vector<double> matching_rmse = calMacthingErr(feature_pc_1, feature_pc_2, TR_1to2);

    return matching_rmse;
}

void callbackLidar1(const sensor_msgs::PointCloud2ConstPtr& lidar1_msg)
{
    fromROSMsg(*lidar1_msg, *lidar1_pc);
    ext_updated = false;
    int lidar1_size = lidar1_pc->points.size();
    if(lidar1_size > lidar1_pre_size)
    {
        cout << "[lidar1] recieved " << lidar1_size << " points" << endl;
        lidar1_pre_size = lidar1_size;
        if(lidar1_size >= 3)
        {
            lidar1_recieved = true;
        }
    }

    if(lidar1_recieved && lidar2_recieved)
    {
        if(DEBUG) cout << "<<<<< lidar1 start ext cal!" << endl;
        align_rmse = calExtrinsicParam(*lidar1_pc, *lidar2_pc, TR_1to2);
        lidar1_recieved = false;
        lidar2_recieved = false;
        if(!ext_got)
            ext_got = true;
        else
        {
            ext_updated = true;
            ROS_WARN("param updated");
        }
        cout << "TR_1to2 = " << TR_1to2 << endl;
        cout << "align rmse = [";
        for(auto p : align_rmse)
        {
            cout << p << " ";
        }
        cout << "]" << endl;
    }
}

void callbackLidar2(const sensor_msgs::PointCloud2ConstPtr& lidar2_msg)
{
    ext_updated = false;
    fromROSMsg(*lidar2_msg, *lidar2_pc);
    int lidar2_size = lidar2_pc->points.size();
    if(lidar2_size > lidar2_pre_size)
    {
        cout << "[lidar2] recieved " << lidar2_size << " points" << endl;
        lidar2_pre_size = lidar2_size;
        if(lidar2_size >= 3)
        {
            lidar2_recieved = true;
        }
    }

    if(lidar1_recieved && lidar2_recieved)
    {
        if(DEBUG) cout << "<<<<< lidar2 start ext cal!" << endl;
        align_rmse = calExtrinsicParam(*lidar1_pc, *lidar2_pc, TR_1to2);
        lidar1_recieved = false;
        lidar2_recieved = false;
        if(!ext_got)
            ext_got = true;
        else
        {
            ext_updated = true;
            ROS_WARN("param updated");
        }
        cout << "TR_1to2 = " << TR_1to2 << endl;
        cout << "align rmse = [";
        for(auto p : align_rmse)
        {
            cout << p << " ";
        }
        cout << "]" << endl;
    }
}



void getParameters() 
{
    // cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("~lidar1_point_tp", lidar1_point_tp)) {
        cout << "Can not get the value of lidar1_point_tp" << endl;
        exit(1);
    }
    // else {
    //     cout << "lidar1_point_tp: " << lidar1_point_tp << endl;
    // }
    if (!ros::param::get("~lidar2_point_tp", lidar2_point_tp)) {
        cout << "Can not get the value of lidar2_point_tp" << endl;
        exit(1);
    }
    // else {
    //     cout << "lidar2_point_tp: " << lidar2_point_tp << endl;
    // }
    if (!ros::param::get("~lidar1_pcd_filepath", lidar1_pcd_filepath)) {
        cout << "Can not get the value of lidar1_pcd_filepath" << endl;
        to_show_fusion_cloud = false;
    }
    if (!ros::param::get("~lidar2_pcd_filepath", lidar2_pcd_filepath)) {
        cout << "Can not get the value of lidar2_pcd_filepath" << endl;
        to_show_fusion_cloud = false;
    }
    if (!ros::param::get("~exParam_filepath", exParam_filepath)) {
        cout << "Can not get the value of exParam_filepath" << endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "l2lCalibration");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    string priv_nh_str = priv_nh.getNamespace();
    getParameters();

    lidar1_pc = CloudT::Ptr(new CloudT);
    lidar2_pc = CloudT::Ptr(new CloudT);

    ros::Subscriber sub1 = priv_nh.subscribe(lidar1_point_tp, 5, callbackLidar1);
    ros::Subscriber sub2 = priv_nh.subscribe(lidar2_point_tp, 5, callbackLidar2);
    
    CloudT::Ptr lidar1_pcd_pc(new CloudT);
    CloudT::Ptr lidar2_pcd_pc(new CloudT);
    CloudT::Ptr lidar1_pcd_pc_trans(new CloudT);

    bool auto_mode = false;
    priv_nh.param("auto_mode", auto_mode, false);
    if(auto_mode)
    {
        string start_flag_lidar1_str, start_flag_lidar2_str;
        priv_nh.param<string>("start_flag_lidar1", start_flag_lidar1_str, "/lidar1/pcdTransfer/pcdTransferFinished");
        priv_nh.param<string>("start_flag_lidar2", start_flag_lidar2_str, "/lidar2/pcdTransfer/pcdTransferFinished");
        bool start_proc_lidar1 = false, start_proc_lidar2 = false;
        do
        {
            ros::param::param(start_flag_lidar1_str, start_proc_lidar1, false);
            ros::param::param(start_flag_lidar2_str, start_proc_lidar2, false);
        } while (!start_proc_lidar1 || !start_proc_lidar2);
        // ros::param::set(start_flag_lidar1_str, false);
        // ros::param::set(start_flag_lidar2_str, false);
        sleep(1);
    }

    cout << "[" << priv_nh_str << "] Initialized......" << endl;

    if(to_show_fusion_cloud)
    {
        if(pcl::io::loadPCDFile(lidar1_pcd_filepath, *lidar1_pcd_pc))
        {
            cerr << "ERROR: Cannot open file " << lidar1_pcd_filepath << "! Aborting..." << endl;
            to_show_fusion_cloud = false;
        }
        else
            cout << "Loaded lidar1 pcd pointcloud!" << endl;
        if(pcl::io::loadPCDFile(lidar2_pcd_filepath, *lidar2_pcd_pc))
        {
            cerr << "ERROR: Cannot open file " << lidar2_pcd_filepath << "! Aborting..." << endl;
            to_show_fusion_cloud = false;
        }
        else
            cout << "Loaded lidar2 pcd pointcloud!" << endl;
    }
    CloudVisual my_cv;

    bool showed_fusion_cloud = false;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        spinOnce();
        loop_rate.sleep();
        if(ext_got)
        {
            if(to_show_fusion_cloud && !showed_fusion_cloud)
            {
                cout << "got ext param, to show fused cloud" << endl;
                pcl::transformPointCloud(*lidar1_pcd_pc, *lidar1_pcd_pc_trans, TR_1to2);
                my_cv.setCloud12(lidar1_pcd_pc_trans, lidar2_pcd_pc);
                thread th(&CloudVisual::showPointCloud, &my_cv);
                th.detach();
                showed_fusion_cloud = true;
            }
        }
        if(ext_updated)
        {
            if(to_show_fusion_cloud)
            {
                pcl::transformPointCloud(*lidar1_pcd_pc, *lidar1_pcd_pc_trans, TR_1to2);
                my_cv.updatePointCloud(lidar1_pcd_pc_trans, lidar2_pcd_pc);
                // ext_updated = false;
            }
        }
    }

    // save result
    if(ext_got)
    {
        ofstream savefile_exparam, savefile_exparam_time;
        string outputName = exParam_filepath + ".csv";
        time_t rawtime;
        struct tm *ptminfo;
        time(&rawtime);
        ptminfo = localtime(&rawtime);
        string outputNameTime = exParam_filepath + "_" + std::to_string(ptminfo->tm_year + 1900) + std::to_string(ptminfo->tm_mon + 1) + std::to_string(ptminfo->tm_mday) + std::to_string(ptminfo->tm_hour) + std::to_string(ptminfo->tm_min) + std::to_string(ptminfo->tm_sec) + ".csv";
        savefile_exparam.open(outputName, ios::out);
        savefile_exparam_time.open(outputNameTime, ios::out);
        savefile_exparam << "TR_L1toL2" << endl;
        savefile_exparam_time << "TR_L1toL2" << endl;
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                if(j==3)
                {
                    savefile_exparam << TR_1to2(i,j) << endl;
                    savefile_exparam_time << TR_1to2(i,j) << endl;
                }
                else
                {
                    savefile_exparam << TR_1to2(i,j) << ",";
                    savefile_exparam_time << TR_1to2(i,j) << ",";
                }
            }
        }
        savefile_exparam << endl;
        savefile_exparam_time << endl;
        savefile_exparam << "align_rmse_x,align_rmse_y,align_rmse_z,align_rmse_total"<< endl;
        savefile_exparam_time << "align_rmse_x,align_rmse_y,align_rmse_z,align_rmse_total"<< endl;
        for(auto p:align_rmse)
        {
            savefile_exparam << p << ",";
            savefile_exparam_time << p << ",";
        }

        savefile_exparam.close();
        savefile_exparam_time.close();
        cout << "[" << priv_nh.getNamespace() << "]" << " calib result saved!" << endl;
    }

    ros::shutdown();
    return 0;
}
