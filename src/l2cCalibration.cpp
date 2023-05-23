#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>	
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <ctime>
#include <thread>

#define DEBUG 0

using namespace std;
using namespace Eigen;
using namespace ros;
using namespace pcl;
using namespace sensor_msgs;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef pcl::PointXYZI PointIT;
typedef pcl::PointCloud<PointIT> CloudIT;
typedef pcl::PointXYZRGB PointRGBT;
typedef pcl::PointCloud<PointRGBT> CloudRGBT;

string lidar_point_tp, camera_point_tp;
string lidar_pcd_filepath, camera_img_filepath, camera_param_filepath, exParam_filepath;
vector<float> init_TR_L2C;

bool lidar_recieved, camera_recieved = false;
int lidar_pre_size, camera_pre_size = 0;
CloudT::Ptr lidar_recieved_pc_ptr, camera_recieved_pc_ptr;
cv::Mat cam_intrinsic, cam_distcoff;
cv::Size img_size;
Eigen::Matrix3d camera_matrix;
Eigen::Matrix4d TR_L2C;
bool ext_got = false, ext_updated = false, to_show_reproj_img = true;
vector<double> reproj_rmse;

int color[21][3] = 
{
    {255, 0, 0}, {255, 69, 0}, {255, 99, 71}, 
    {255, 140, 0}, {255, 165, 0}, {238, 173, 14},
    {255, 193, 37}, {255, 255, 0}, {255, 236, 139},
    {202, 255, 112}, {0, 255, 0}, {84, 255, 159},
    {127, 255, 212}, {0, 229, 238}, {152, 245, 255},
    {178, 223, 238}, {126, 192, 238}, {28, 134, 238},
    {0, 0, 255}, {72, 118, 255}, {122, 103, 238} 
};
float color_distance;   //step length to color the lidar points according to plane distance(z)
float color_intenisty;
bool colored_by_intensity = false;

class CloudVisual
{
private:
    CloudRGBT::Ptr cloud;
    bool to_update = false;
public:
    CloudVisual(){
        cloud = CloudRGBT::Ptr(new CloudRGBT);
    }
    CloudVisual(CloudRGBT::Ptr cloud_):cloud(cloud_){}
    void setCloud(CloudRGBT::Ptr cloud_){
        cloud = cloud_;
    }
    bool getToUpdate(){return to_update;}
    void updatePointCloud(CloudRGBT::Ptr& cloud_new){
        cloud = cloud_new;
        to_update = true;
    }
    void showPointCloud()
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("rebuild_viewer"));
        pcl::visualization::PointCloudColorHandlerRGBField<PointRGBT> rgb(cloud);
        viewer->addPointCloud<PointRGBT>(cloud, rgb, "rebuild_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "rebuild_cloud");

        while(!viewer->wasStopped() && ros::ok())
        {
            if(to_update)
            {
                viewer->removeAllPointClouds();
                pcl::visualization::PointCloudColorHandlerRGBField<PointRGBT> rgb_(cloud);
                viewer->addPointCloud<PointRGBT>(cloud, rgb_, "rebuild_cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "rebuild_cloud");

                cout << "[viewer] cloud updated!" << endl;
                to_update = false; 
            }
            viewer->spinOnce(100);
        }

    }
};

class excalib2d_ceres
{
private:
    PointT lidar_p;
    cv::Point2f img_p;
    Eigen::Matrix3d camera_matrix;

public:
    excalib2d_ceres(PointT lidar_p_, cv::Point2f img_p_, Eigen::Matrix3d camera_matrix_):lidar_p(lidar_p_), img_p(img_p_), camera_matrix(camera_matrix_){};
    excalib2d_ceres(){};
    ~excalib2d_ceres(){};
    Eigen::Matrix3d getCameraMatrix()
    {
        return camera_matrix;
    }

    template <typename T>
    bool operator()(const T *_q, const T *_t, T *residuals) const {
        Eigen::Matrix<T, 3, 3> innerT = camera_matrix.cast<T>();
        Eigen::Quaternion<T> q_incre{_q[ 3 ], _q[ 0 ], _q[ 1 ], _q[ 2 ]};
        Eigen::Matrix<T, 3, 1> t_incre{_t[ 0 ], _t[ 1 ], _t[ 2 ]};

        Eigen::Matrix<T, 3, 1> p_l(T(lidar_p.x), T(lidar_p.y), T(lidar_p.z));
        Eigen::Matrix<T, 3, 1> p_l_in_c = q_incre.toRotationMatrix() * p_l + t_incre;
         
        Eigen::Matrix<T, 3, 1> p_l_2d = innerT * p_l_in_c;

        residuals[0] = p_l_2d[0]/p_l_2d[2] - T(img_p.x);
        residuals[1] = p_l_2d[1]/p_l_2d[2] - T(img_p.y);

        return true;
    }

    static ceres::CostFunction *Create(PointT lidar_p_, cv::Point2f img_p_, Eigen::Matrix3d camera_matrix_) {
        return (new ceres::AutoDiffCostFunction<excalib2d_ceres, 2, 4, 3>(new excalib2d_ceres(lidar_p_, img_p_, camera_matrix_)));
    }
};

vector<double> calReprojErr(CloudT& cloud_l, vector<cv::Point2f>& points_c, cv::Mat& cam_intrinsic_, cv::Mat& cam_distcoff_, Eigen::Matrix4d& TR_L2C_)
{
    CloudT::Ptr trans_cloud_l(new CloudT);
    pcl::transformPointCloud(cloud_l, *trans_cloud_l, TR_L2C_);
    vector<cv::Point3d> trans_points_l;
    vector<cv::Point2d> reproj_points_l;
    for(auto it = trans_cloud_l->points.begin(); it < trans_cloud_l->points.end(); it++)
    {
        trans_points_l.push_back(cv::Point3d(it->x, it->y, it->z));
    }
    //--projection
    cv::projectPoints(trans_points_l, cv::Mat::eye(3, 3, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1), cam_intrinsic_, cam_distcoff_, reproj_points_l);
    //--cal error
    size_t pc1_size = cloud_l.points.size();
    size_t pc2_size = points_c.size();
    size_t point_size = pc1_size < pc2_size ? pc1_size : pc2_size;
    double esum_x = 0, esum_y = 0;
    double rmse_total = 0, rmse_x = 0, rmse_y = 0;
    for(int i = 0; i < point_size; i++)
    {
        esum_x += pow(reproj_points_l[i].x - points_c[i].x, 2);
        esum_y += pow(reproj_points_l[i].y - points_c[i].y, 2);
    }

    rmse_x = sqrt(esum_x/double(point_size));
    rmse_y = sqrt(esum_y/double(point_size));
    rmse_total = sqrt(pow(rmse_x, 2) + pow(rmse_y, 2));

    vector<double> rmse = {rmse_x, rmse_y, rmse_total};
    return rmse;
}

vector<double> calExtrinsicParam(CloudT& feature_pc_l, CloudT& feature_pc_c, cv::Mat& cam_intrinsic_, cv::Mat& cam_distcoff_, Eigen::Matrix4d& TR_L2C_)
{
    Eigen::Matrix3d camera_matrix;
    cv::cv2eigen(cam_intrinsic_, camera_matrix);

    int size_pl = feature_pc_l.points.size();
    int size_pc = feature_pc_c.points.size();
    int size_p = size_pl <= size_pc ? size_pl : size_pc;
    if(DEBUG) cout << "size_p = " << size_p << endl;
    vector<cv::Point2f> feature_pv_c;
    for(int i = 0; i < size_p; i++)
    {
        cv::Point2f P(feature_pc_c.points[i].x, feature_pc_c.points[i].y);
        feature_pv_c.push_back(P);
    }

    // ICP REGISTRATION
    Eigen::Matrix3d R = TR_L2C_.block(0,0,3,3);
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
        cost_function = excalib2d_ceres::Create(feature_pc_l.points[val], feature_pv_c[val], camera_matrix);
        problem.AddResidualBlock(cost_function, NULL, ext, ext + 4);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if(DEBUG) cout << summary.BriefReport() << endl;

    Eigen::Matrix3d rot = m_q.toRotationMatrix();
    TR_L2C_.block(0,0,3,3) = rot;
    TR_L2C_.topRightCorner(3,1) = m_t;
    // cout << "Tr_L2C = " << endl << TR_L2C_ << endl;

    // REPROJECTION ERROR
    vector<double> reproj_rmse = calReprojErr(feature_pc_l, feature_pv_c, cam_intrinsic_, cam_distcoff_, TR_L2C_);
    // if(DEBUG1){
        // cout << "reproj rmse = [";
        // for(auto p : reproj_rmse)
        // {
        //     cout << p << " ";
        // }
        // cout << "]" << endl;
    // }
    return reproj_rmse;    
}


void callbackLidar(const sensor_msgs::PointCloud2ConstPtr& lidar_msg)
{
    fromROSMsg(*lidar_msg, *lidar_recieved_pc_ptr);
    ext_updated = false;
    int lidar_size = lidar_recieved_pc_ptr->points.size();
    if(lidar_size > lidar_pre_size)
    {
        cout << "[Lidar] recieved " << lidar_size << " points" << endl;
        lidar_pre_size = lidar_size;
        if(lidar_size >= 3)
        {
            lidar_recieved = true;
        }
    }

    if(lidar_recieved && camera_recieved)
    {
        if(DEBUG) cout << "<<<<< lidar start ext cal!" << endl;
        reproj_rmse = calExtrinsicParam(*lidar_recieved_pc_ptr, *camera_recieved_pc_ptr, cam_intrinsic, cam_distcoff, TR_L2C);
        lidar_recieved = false;
        camera_recieved = false;
        if(!ext_got)
            ext_got = true;
        else
        {
            ROS_WARN("param updated");
            ext_updated = true;
        }

        cout << "TR_L2C = " << TR_L2C << endl;
        cout << "reproj rmse = [";
        for(auto p : reproj_rmse)
        {
            cout << p << " ";
        }
        cout << "]" << endl;
    }
}

void callbackCamera(const sensor_msgs::PointCloud2ConstPtr& camera_msg)
{
    fromROSMsg(*camera_msg, *camera_recieved_pc_ptr);
    ext_updated = false;
    int camera_size = camera_recieved_pc_ptr->points.size();
    if(camera_size > camera_pre_size)
    {
        cout << "[Camera] recieved " << camera_size << " points" << endl;
        camera_pre_size = camera_size;
        if(camera_size >= 3)
        {
            camera_recieved = true;
        }
    }

    if(lidar_recieved && camera_recieved)
    {
        if(DEBUG) cout << "<<<<< camera start ext cal!" << endl;
        reproj_rmse = calExtrinsicParam(*lidar_recieved_pc_ptr, *camera_recieved_pc_ptr, cam_intrinsic, cam_distcoff, TR_L2C);
        lidar_recieved = false;
        camera_recieved = false;
        if(!ext_got)
            ext_got = true;
        else
        {
            ROS_WARN("param updated");
            ext_updated = true;
        }

        cout << "TR_L2C = " << TR_L2C << endl;
        cout << "reproj rmse = [";
        for(auto p : reproj_rmse)
        {
            cout << p << " ";
        }
        cout << "]" << endl;
    }
}

void getReprojImgPC(CloudIT& cloud, CloudRGBT& colored_cloud, cv::Mat& image, cv::Mat& cam_intrinsic_, cv::Mat& cam_distcoff_, cv::Size& img_size_, Eigen::Matrix4d TR_L2C_)
{
    CloudIT::Ptr trans_cloud(new CloudIT);
    pcl::transformPointCloud(cloud, *trans_cloud, TR_L2C_);
    std::vector<cv::Point3d> lidar_points;
    std::vector<cv::Point2d> imagePoints;
    std::vector<cv::Scalar> dis_color;
    
    for(auto it = trans_cloud->points.begin(); it < trans_cloud->points.end(); it++)
    {
        if(it->z > 0)
        {
            lidar_points.push_back(cv::Point3d(it->x, it->y, it->z));
            int color_order;
            if(colored_by_intensity)
                color_order = int(it->intensity/color_intenisty);
            else
                color_order = int(it->z/color_distance);
            if(color_order > 20)    color_order = 20;
            dis_color.push_back(cv::Scalar(color[color_order][2], color[color_order][1], color[color_order][0]));
        }
    }

    cv::Mat image_copy = image.clone();
    cv::projectPoints(lidar_points, cv::Mat::eye(3, 3, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1), cam_intrinsic_, cam_distcoff_, imagePoints);
    for(auto it = imagePoints.begin(); it < imagePoints.end(); it ++)
    {
        int idx = it - imagePoints.begin();
        if(it->x >= 0 && it->x < img_size_.width && it->y >= 0 && it->y < img_size_.height)
        {
            cv::circle(image, *it, 1, dis_color[idx], -1, 9, 0);

            // for rebuild point cloud
            PointRGBT point;
            point.x = lidar_points[idx].x;
            point.y = lidar_points[idx].y;
            point.z = lidar_points[idx].z;
            point.r = image_copy.at<cv::Vec3b>(*it)[2];
            point.g = image_copy.at<cv::Vec3b>(*it)[1];
            point.b = image_copy.at<cv::Vec3b>(*it)[0];
            colored_cloud.points.push_back(point);
        }
    }
}


void getParameters() 
{
    // cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("~lidar_point_tp", lidar_point_tp)) {
        cout << "Can not get the value of lidar_point_tp" << endl;
        exit(1);
    }
    // else {
    //     cout << "lidar_point_tp: " << lidar_point_tp << endl;
    // }
    if (!ros::param::get("~camera_point_tp", camera_point_tp)) {
        cout << "Can not get the value of camera_point_tp" << endl;
        exit(1);
    }
    // else {
    //     cout << "camera_point_tp: " << camera_point_tp << endl;
    // }
    if (!ros::param::get("~camera_param_filepath", camera_param_filepath)) {
        cout << "Can not get the value of camera_param_filepath" << endl;
        exit(1);
    }
    else{
        cout << "camera_param_filepath: " << camera_param_filepath << endl;
    }
    if (!ros::param::get("~exParam_filepath", exParam_filepath)) {
        cout << "Can not get the value of exParam_filepath" << endl;
    }
    if (!ros::param::get("~lidar_pcd_filepath", lidar_pcd_filepath)) {
        cout << "Can not get the value of lidar_pcd_filepath" << endl;
        to_show_reproj_img = false;
    }
    if (!ros::param::get("~camera_img_filepath", camera_img_filepath)) {
        cout << "Can not get the value of camera_img_filepath" << endl;
        to_show_reproj_img = false;
    }
    init_TR_L2C.resize(12);
    if (!ros::param::get("~init_TR_L2C", init_TR_L2C)) {
        cout << "Can not get the value of init_TR_L2C, use default" << endl;
        init_TR_L2C.resize(12, 0.0);
        init_TR_L2C[1] = -1.0;
        init_TR_L2C[6] = -1.0;
        init_TR_L2C[8] = 1.0;
    }

    if(!ros::param::get("~color_intenisty", color_intenisty)){
        colored_by_intensity = false;
        ros::param::param<float>("~color_distance", color_distance, 0.2);
    }
    else{
        colored_by_intensity = true;
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "l2cCalibration");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    string priv_nh_str = priv_nh.getNamespace();
    getParameters();

    cv::FileStorage fs_reader(camera_param_filepath, cv::FileStorage::READ);
    fs_reader["CameraMat"] >> cam_intrinsic;
    fs_reader["DistCoeff"] >> cam_distcoff;
    fs_reader["ImageSize"] >> img_size;
    fs_reader.release();
    
    TR_L2C << init_TR_L2C[0], init_TR_L2C[1], init_TR_L2C[2], init_TR_L2C[3],
         init_TR_L2C[4], init_TR_L2C[5], init_TR_L2C[6], init_TR_L2C[7],
         init_TR_L2C[8], init_TR_L2C[9], init_TR_L2C[10], init_TR_L2C[11],
         0, 0, 0, 1;

    lidar_recieved_pc_ptr = CloudT::Ptr(new CloudT);
    camera_recieved_pc_ptr = CloudT::Ptr(new CloudT);

    ros::Subscriber sub1 = priv_nh.subscribe(lidar_point_tp, 5, callbackLidar);
    ros::Subscriber sub2 = priv_nh.subscribe(camera_point_tp, 5, callbackCamera);

    bool auto_mode = false;
    priv_nh.param("auto_mode", auto_mode, false);
    if(auto_mode)
    {
        string start_flag_lidar_str, start_flag_cam_str;
        priv_nh.param<string>("start_flag_lidar", start_flag_lidar_str, "/pcdTransferFinished");
        priv_nh.param<string>("start_flag_cam", start_flag_cam_str, "/imgCaptureFinished");
        bool start_proc_lidar = false, start_proc_cam = false;
        do
        {
            ros::param::param(start_flag_lidar_str, start_proc_lidar, false);
            ros::param::param(start_flag_cam_str, start_proc_cam, false);
        } while (!start_proc_lidar || !start_proc_cam);
        // ros::param::set(start_flag_lidar_str, false);
        // ros::param::set(start_flag_cam_str, false);
        sleep(1);
    }

    CloudIT::Ptr lidar_pcd_pc(new CloudIT);
    cv::Mat cam_image;
    if(to_show_reproj_img)
    {
        if(pcl::io::loadPCDFile(lidar_pcd_filepath, *lidar_pcd_pc))
        {
            cerr << "ERROR: Cannot open file " << lidar_pcd_filepath << "! Aborting..." << endl;
            to_show_reproj_img = false;
        }
        else
        {
            pcl::ApproximateVoxelGrid<PointIT> avf;
            avf.setInputCloud(lidar_pcd_pc);
            avf.setLeafSize(0.01, 0.01, 0.01);
            avf.filter(*lidar_pcd_pc);
        }

        cam_image = cv::imread(camera_img_filepath);
        // cout << "Loaded camera image!" << endl;
    }
    CloudRGBT::Ptr colored_cloud(new CloudRGBT);
    // cv::imread(camera_img_filepath);

    cout << "[" << priv_nh_str << "] Initialized......" << endl;
    cout << "cam_intrinsic: " << cam_intrinsic << endl;
    cout << "cam_distcoff: " << cam_distcoff << endl;
    cout << "imagesize: " << img_size << endl;
    cout << "init_TR_L2C: " << TR_L2C << endl;

    CloudVisual my_cv;

    bool showed_reproj_img = false;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        spinOnce();
        loop_rate.sleep();
        if(ext_got)
        {
            if(to_show_reproj_img && !showed_reproj_img)
            {
                cout << "got ext param, to show reproj image and rebuild point cloud" << endl;
                cv::Mat image_copy = cam_image.clone();
                getReprojImgPC(*lidar_pcd_pc, *colored_cloud, image_copy, cam_intrinsic, cam_distcoff, img_size, TR_L2C);
                my_cv.setCloud(colored_cloud);
                thread th(&CloudVisual::showPointCloud, &my_cv);
                th.detach();
                cv::imshow("reproj image", image_copy);
                showed_reproj_img = true;
            }
        }
        if(ext_updated)
        {
            if(to_show_reproj_img)
            {
                cv::Mat image_copy = cam_image.clone();
                getReprojImgPC(*lidar_pcd_pc, *colored_cloud, image_copy, cam_intrinsic, cam_distcoff, img_size, TR_L2C);
                my_cv.updatePointCloud(colored_cloud);
                cv::imshow("reproj image", image_copy);
            }
        }
        if(to_show_reproj_img && showed_reproj_img)
        {
            int key = cv::waitKey(10);
            if(key == 27 || key == 113 || key == 81) // `ESC` `q` `Q`
            {
                cv::destroyAllWindows();
                break;
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
        savefile_exparam << "TR_L2C" << endl;
        savefile_exparam_time << "TR_L2C" << endl;
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                if(j==3)
                {
                    savefile_exparam << TR_L2C(i,j) << endl;
                    savefile_exparam_time << TR_L2C(i,j) << endl;
                }
                else
                {
                    savefile_exparam << TR_L2C(i,j) << ",";
                    savefile_exparam_time << TR_L2C(i,j) << ",";
                }
            }
        }
        savefile_exparam << endl;
        savefile_exparam_time << endl;
        savefile_exparam << "reproj_rmse_x,reproj_rmse_y,reproj_rmse_total"<< endl;
        savefile_exparam_time << "reproj_rmse_x,reproj_rmse_y,reproj_rmse_total"<< endl;
        for(auto p:reproj_rmse)
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