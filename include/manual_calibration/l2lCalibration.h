#define PCL_NO_PRECOMPILE

#include <stdio.h>
#include <string>
#include <ceres/ceres.h>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#define DEBUG 1

using namespace std;
using namespace Eigen;
using namespace pcl;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

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


vector<double> calMacthingErr(CloudT& cloud1, CloudT& cloud2, Eigen::Matrix4d& TR_1to2)
{
    double esum_x = 0, esum_y = 0, esum_z = 0;
    double rmse_total = 0, rmse_x = 0, rmse_y = 0, rmse_z = 0;
    CloudT::Ptr cloud1_trans_ptr(new CloudT);
    pcl::transformPointCloud(cloud1, *cloud1_trans_ptr, TR_1to2);
    size_t point_size = cloud1.points.size();
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
        cost_function = excalib3d_ceres::Create(feature_pc_1.points[val], feature_pc_2.points[val]);
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
    if(DEBUG) cout << "Tr_1to2 = " << endl << TR_1to2 << endl;

    // MATCHING ERROR
    vector<int> valid_indice(size_p);
    for(int i = 0; i < size_p; i++)
    {
        valid_indice.push_back(i);
    }
    CloudT::Ptr cloud1_valid(new CloudT);
    CloudT::Ptr cloud2_valid(new CloudT);
    pcl::copyPointCloud(feature_pc_1, valid_indice, *cloud1_valid);
    pcl::copyPointCloud(feature_pc_2, valid_indice, *cloud2_valid);
    vector<double> matching_rmse = calMacthingErr(*cloud1_valid, *cloud2_valid, TR_1to2);
    return matching_rmse;
}
