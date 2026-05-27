#ifndef LASER_MAPPING_H_
#define LASER_MAPPING_H_

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <sstream>
#include <Python.h>
#include <so3_math.h>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <ikd-Tree/ikd_Tree.h>
#include <nlohmann/json.hpp>
#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "msgs.h"

using json = nlohmann::json;

using custom_messages::ImuConstPtr;
using custom_messages::ImuPtr;
using custom_messages::OdomMsgPtr;

#define CONFIG_FILE_PATH    std::string("../config/horizon.json")
#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)

PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));

double solve_time = 0, match_time = 0;

double res_mean_last = 0.05, total_residual = 0.0;

bool extrinsic_est_en = true;

float res_last[100000] = {0.0};

int effct_feat_num = 0;
int feats_down_size = 0;
bool   point_selected_surf[100000] = {0};

vector<PointVector>  Nearest_Points;
KD_TREE<PointType> ikdtree;

class LaserMapping
{
public:
    LaserMapping(/* args */);
    ~LaserMapping();

    void livox_pcl_cbk(const CstMsgConstPtr &msg);
    void imu_cbk(const ImuConstPtr &msg_in);
    void run(OdomMsgPtr &msg_in);

private:
    void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s);
    void pointBodyToWorld(PointType const * const pi, PointType * const po);
    void RGBpointBodyToWorld(PointType const * const pi, PointType * const po);
    void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po);
    void points_cache_collect();
    void lasermap_fov_segment();
    bool sync_packages(MeasureGroup &meas);
    void map_incremental();
    void update_odometry(OdomMsgPtr &msg_in);
    static void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
    {
        double match_start = omp_get_wtime();
        laserCloudOri->clear(); 
        corr_normvect->clear(); 
        total_residual = 0.0; 

        /** closest surface search and residual computation **/
        #ifdef MP_EN
            omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
        #endif
        for (int i = 0; i < feats_down_size; i++)
        {
            PointType &point_body  = feats_down_body->points[i]; 
            PointType &point_world = feats_down_world->points[i]; 

            /* transform to world frame */
            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_world.intensity = point_body.intensity;

            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

            auto &points_near = Nearest_Points[i];

            if (ekfom_data.converge)
            {
                /** Find the closest surfaces in the map **/
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
            }

            if (!point_selected_surf[i]) continue;

            VF(4) pabcd;
            point_selected_surf[i] = false;
            if (esti_plane(pabcd, points_near, 0.1f))
            {
                float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                if (s > 0.9)
                {
                    point_selected_surf[i] = true;
                    normvec->points[i].x = pabcd(0);
                    normvec->points[i].y = pabcd(1);
                    normvec->points[i].z = pabcd(2);
                    normvec->points[i].intensity = pd2;
                    res_last[i] = abs(pd2);
                }
            }
        }

        effct_feat_num = 0;

        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_selected_surf[i])
            {
                laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];
                total_residual += res_last[i];
                effct_feat_num ++;
            }
        }

        if (effct_feat_num < 1)
        {
            ekfom_data.valid = false;
            //   ROS_WARN("No Effective Points! \n");
            return;
        }

        res_mean_last = total_residual / effct_feat_num;
        match_time  += omp_get_wtime() - match_start;
        double solve_start_  = omp_get_wtime();

        /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
        ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
        ekfom_data.h.resize(effct_feat_num);

        for (int i = 0; i < effct_feat_num; i++)
        {
            const PointType &laser_p  = laserCloudOri->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat<<SKEW_SYM_MATRX(point_this);

            /*** get the normal vector of closest surface/corner ***/
            const PointType &norm_p = corr_normvect->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            /*** calculate the Measuremnt Jacobian matrix H ***/
            V3D C(s.rot.conjugate() *norm_vec);
            V3D A(point_crossmat * C);
            if (extrinsic_est_en)
            {
                V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
                ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            }
            else
            {
                ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }

            /*** Measuremnt: distance to the closest surface/corner ***/
            ekfom_data.h(i) = -norm_p.intensity;
        }
        solve_time += omp_get_wtime() - solve_start_;
    }

    template<typename T>
    void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po);

    template<typename T>
    void set_posestamp(T & out);

private:
    static const int MAXN = 720000;
    double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
    double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
    double solve_const_H_time = 0;
    int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
    bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, path_en = true;

    double msr_freq = 0.0, main_freq = 0.0;
    double timediff_lidar_wrt_imu = 0.0;

    float DET_RANGE = 300.0f;
    const float MOV_THRESHOLD = 1.5f;
    double time_diff_lidar_to_imu = 0.0;

    mutex mtx_buffer;
    condition_variable sig_buffer;

    string root_dir = ROOT_DIR;
    string map_file_path, lid_topic, imu_topic;

    double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
    double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
    int    time_log_counter = 0, scan_count = 0, publish_count = 0;
    int    iterCount = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
    bool   lidar_pushed, flg_first_scan = true, flg_EKF_inited;
    bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

    vector<vector<int>>  pointSearchInd_surf; 
    vector<BoxPointType> cub_needrm;
    vector<double>       extrinT;
    vector<double>       extrinR;
    deque<double>                     time_buffer;
    deque<PointCloudXYZI::Ptr>        lidar_buffer;
    deque<ImuConstPtr> imu_buffer;

    PointCloudXYZI::Ptr featsFromMap;
    PointCloudXYZI::Ptr feats_undistort;
    PointCloudXYZI::Ptr _featsArray;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    V3F XAxisPoint_body;
    V3F XAxisPoint_world;
    V3D euler_cur;
    V3D position_last;
    V3D Lidar_T_wrt_IMU;
    M3D Lidar_R_wrt_IMU;

    /*** EKF inputs and output ***/
    MeasureGroup Measures;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom state_point;
    vect3 pos_lid;

    custom_messages::Odometry odomAftMapped;
    custom_messages::Quaternion geoQuat;
    custom_messages::PoseStamped msg_body_pose;

    shared_ptr<Preprocess> p_pre;
    shared_ptr<ImuProcess> p_imu;

    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
};

LaserMapping::LaserMapping() : extrinT(3, 0.0), extrinR(9, 0.0), featsFromMap(new PointCloudXYZI()), feats_undistort(new PointCloudXYZI()),\
                            XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0), XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0),\
                            position_last(Zero3d), Lidar_T_wrt_IMU(Zero3d), Lidar_R_wrt_IMU(Eye3d),\
                            p_pre(new Preprocess()), p_imu(new ImuProcess())
{
    // res_mean_last = 0.05;
    // total_residual = 0.0;
    // feats_down_size = 0;
    // effct_feat_num = 0;
    // match_time = 0.0;
    // extrinsic_est_en = true;
    // solve_time = 0.0;

    // feats_down_body = boost::make_shared<PointCloudXYZI>();
    // feats_down_world = boost::make_shared<PointCloudXYZI>();
    // normvec = boost::make_shared<PointCloudXYZI>(100000, 1);
    // laserCloudOri = boost::make_shared<PointCloudXYZI>(100000, 1);
    // corr_normvect = boost::make_shared<PointCloudXYZI>(100000, 1);

    // read json file and set config vars
    std::ifstream config_f(CONFIG_FILE_PATH);
    json config = json::parse(config_f);
    p_pre->lidar_type             = config["preprocess"]["lidar_type"].get<int>();
    if (p_pre->lidar_type == 2)
    {
        p_pre->SCAN_RATE              = config["preprocess"]["scan_rate"].get<int>();
        p_pre->time_unit              = config["preprocess"]["timestamp_unit"].get<int>();
    }
    time_sync_en                  = config["common"]["time_sync_en"].get<bool>();
    time_diff_lidar_to_imu        = config["common"]["time_offset_lidar_to_imu"].get<double>();
    msr_freq                      = config["common"]["msr_freq"].get<double>();
    main_freq                     = config["common"]["main_freq"].get<double>();
    p_pre->N_SCANS                = config["preprocess"]["scan_line"].get<int>();
    p_pre->blind                  = config["preprocess"]["blind"].get<int>();
    acc_cov                       = config["mapping"]["acc_cov"].get<double>();
    gyr_cov                       = config["mapping"]["gyr_cov"].get<double>();
    b_acc_cov                     = config["mapping"]["b_acc_cov"].get<double>();
    b_gyr_cov                     = config["mapping"]["b_gyr_cov"].get<double>();
    fov_deg                       = config["mapping"]["fov_degree"].get<int>();
    DET_RANGE                     = config["mapping"]["det_range"].get<double>();
    extrinsic_est_en              = config["mapping"]["extrinsic_est_en"].get<bool>();
    extrinT                       = config["mapping"]["extrinsic_T"].get<std::vector<double>>();
    extrinR                       = config["mapping"]["extrinsic_R"].get<std::vector<double>>();
    NUM_MAX_ITERATIONS            = 4;
    filter_size_corner_min        = 0.5;
    filter_size_surf_min          = 0.5;
    filter_size_map_min           = 0.5;
    cube_len                      = 200;
    p_pre->point_filter_num       = 2;
    p_pre->feature_enabled        = false;

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
}

LaserMapping::~LaserMapping()
{
    
}

void LaserMapping::pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::update_odometry(OdomMsgPtr &msg_in)
{
    msg_in->header.frame_id = "camera_init";
    msg_in->child_frame_id = "body";
    msg_in->header.stamp = custom_messages::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(msg_in->pose);
    // std::cout << odomAftMapped.pose.pose.position.x << " " << odomAftMapped.pose.pose.position.y << " " << odomAftMapped.pose.pose.position.z << "\n";
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        msg_in->pose.covariance[i*6 + 0] = P(k, 3);
        msg_in->pose.covariance[i*6 + 1] = P(k, 4);
        msg_in->pose.covariance[i*6 + 2] = P(k, 5);
        msg_in->pose.covariance[i*6 + 3] = P(k, 0);
        msg_in->pose.covariance[i*6 + 4] = P(k, 1);
        msg_in->pose.covariance[i*6 + 5] = P(k, 2);
    }

    // msg_in = odomAftMapped;
}

template<typename T>
void LaserMapping::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

template<typename T>
void LaserMapping::set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

void LaserMapping::RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void LaserMapping::points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

void LaserMapping::lasermap_fov_segment()
{
    BoxPointType LocalMap_Points;
    bool Localmap_Initialized = false;

    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void LaserMapping::livox_pcl_cbk(const CstMsgConstPtr &msg)
{
    bool   timediff_set_flg = false;

    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    // std::cout << msg->header.stamp.toSec() << std::endl;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        std::cout << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
   //  std::cout << "msg size: " << msg->points.size();
    p_pre->process(msg, ptr);
   //  std::cout << "ptr size: " << ptr->points.size() << std::endl;
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void LaserMapping::imu_cbk(const ImuConstPtr &msg_in)
{
    publish_count ++;
   // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
   ImuPtr msg(new custom_messages::Imu(*msg_in));

   if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
   {
      msg->header.stamp = \
      custom_messages::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
   }

   msg->header.stamp = custom_messages::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);

   double timestamp = msg->header.stamp.toSec();

   mtx_buffer.lock();

   if (timestamp < last_timestamp_imu)
   {
   //   ROS_WARN("imu loop back, clear buffer");
      imu_buffer.clear();
   }

   last_timestamp_imu = timestamp;

   imu_buffer.push_back(msg);
   mtx_buffer.unlock();
   sig_buffer.notify_all();
}

bool LaserMapping::sync_packages(MeasureGroup &meas)
{
    double lidar_mean_scantime = 0.0;
    int    scan_num = 0;

    // std::cout << "Lidar: " << lidar_buffer.size() << ", IMU: " << imu_buffer.size() << std::endl;
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        // std::cout << "Empttttyyyy..." << std::endl;
        return false;
    }
    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cout << "Too few input point cloud!\n";
            // ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void LaserMapping::map_incremental()
{
    int process_increments = 0;

    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

// static void LaserMapping::h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
// {
//     double match_start = omp_get_wtime();
//     laserCloudOri->clear(); 
//     corr_normvect->clear(); 
//     total_residual = 0.0; 

//     /** closest surface search and residual computation **/
//     #ifdef MP_EN
//         omp_set_num_threads(MP_PROC_NUM);
//         #pragma omp parallel for
//     #endif
//     for (int i = 0; i < feats_down_size; i++)
//     {
//         PointType &point_body  = feats_down_body->points[i]; 
//         PointType &point_world = feats_down_world->points[i]; 

//         /* transform to world frame */
//         V3D p_body(point_body.x, point_body.y, point_body.z);
//         V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
//         point_world.x = p_global(0);
//         point_world.y = p_global(1);
//         point_world.z = p_global(2);
//         point_world.intensity = point_body.intensity;

//         vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

//         auto &points_near = Nearest_Points[i];

//         if (ekfom_data.converge)
//         {
//             /** Find the closest surfaces in the map **/
//             ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
//             point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
//         }

//         if (!point_selected_surf[i]) continue;

//         VF(4) pabcd;
//         point_selected_surf[i] = false;
//         if (esti_plane(pabcd, points_near, 0.1f))
//         {
//             float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
//             float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

//             if (s > 0.9)
//             {
//                 point_selected_surf[i] = true;
//                 normvec->points[i].x = pabcd(0);
//                 normvec->points[i].y = pabcd(1);
//                 normvec->points[i].z = pabcd(2);
//                 normvec->points[i].intensity = pd2;
//                 res_last[i] = abs(pd2);
//             }
//         }
//     }

//     effct_feat_num = 0;

//     for (int i = 0; i < feats_down_size; i++)
//     {
//         if (point_selected_surf[i])
//         {
//             laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
//             corr_normvect->points[effct_feat_num] = normvec->points[i];
//             total_residual += res_last[i];
//             effct_feat_num ++;
//         }
//     }

//     if (effct_feat_num < 1)
//     {
//         ekfom_data.valid = false;
//         //   ROS_WARN("No Effective Points! \n");
//         return;
//     }

//     res_mean_last = total_residual / effct_feat_num;
//     match_time  += omp_get_wtime() - match_start;
//     double solve_start_  = omp_get_wtime();

//     /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
//     ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
//     ekfom_data.h.resize(effct_feat_num);

//     for (int i = 0; i < effct_feat_num; i++)
//     {
//         const PointType &laser_p  = laserCloudOri->points[i];
//         V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
//         M3D point_be_crossmat;
//         point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
//         V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
//         M3D point_crossmat;
//         point_crossmat<<SKEW_SYM_MATRX(point_this);

//         /*** get the normal vector of closest surface/corner ***/
//         const PointType &norm_p = corr_normvect->points[i];
//         V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

//         /*** calculate the Measuremnt Jacobian matrix H ***/
//         V3D C(s.rot.conjugate() *norm_vec);
//         V3D A(point_crossmat * C);
//         if (extrinsic_est_en)
//         {
//             V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
//             ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
//         }
//         else
//         {
//             ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//         }

//         /*** Measuremnt: distance to the closest surface/corner ***/
//         ekfom_data.h(i) = -norm_p.intensity;
//     }
//     solve_time += omp_get_wtime() - solve_start_;
// }

void LaserMapping::run(OdomMsgPtr &msg_in)
{
    if(sync_packages(Measures))
    {
        if (flg_first_scan)
        {
            // std::cout << "I am here 1" << std::endl;
            first_lidar_time = Measures.lidar_beg_time;
            p_imu->first_lidar_time = first_lidar_time;
            flg_first_scan = false;
            return;
        }

        double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

        match_time = 0;
        kdtree_search_time = 0.0;
        solve_time = 0;
        solve_const_H_time = 0;
        svd_time   = 0;
        t0 = omp_get_wtime();

        // std::cout << "I am here 2" << std::endl;
        p_imu->Process(Measures, kf, feats_undistort);
        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

        // std::cout << "I am here 3" << std::endl;

        // std::cout << "feats_undistort->empty(): " << feats_undistort->empty() << std::endl;
        // std::cout << "feats_undistort == NULL: " << (feats_undistort == NULL) << std::endl;
        // std::cout << "feats_undistort->size()" << feats_undistort->size() << std::endl;

        if (feats_undistort->empty() || (feats_undistort == NULL))
        {
            //  ROS_WARN("No point, skip this scan!\n");
            // std::cout << "I am here 4" << std::endl;
            return;
        }

        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                        false : true;
        // std::cout << "I am here 5" << std::endl;
        /*** Segment the map in lidar FOV ***/
        lasermap_fov_segment();

        // std::cout << "I am here 6" << std::endl;

        /*** downsample the feature points in a scan ***/
        downSizeFilterSurf.setInputCloud(feats_undistort);
        // std::cout << "I am here 7" << std::endl;
        downSizeFilterSurf.filter(*feats_down_body);
        // std::cout << "I am here 8" << std::endl;
        t1 = omp_get_wtime();
        // std::cout << "I am here 9" << std::endl;
        feats_down_size = feats_down_body->points.size();
        // std::cout << "feats_down_size: " << feats_down_size << std::endl;
        /*** initialize the map kdtree ***/
        if(ikdtree.Root_Node == nullptr)
        {
            // std::cout << "I am here 10" << std::endl;
            if(feats_down_size > 5)
            {
                // std::cout << "I am here 11" << std::endl;
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);
                for(int i = 0; i < feats_down_size; i++)
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                ikdtree.Build(feats_down_world->points);
            }
            return;
        }
        int featsFromMapNum = ikdtree.validnum();
        kdtree_size_st = ikdtree.size();
        // std::cout << "I am here 12" << std::endl;
        // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

        /*** ICP and iterated Kalman filter update ***/
        if (feats_down_size < 5)
        {
            // std::cout << "I am here 13" << std::endl;
            //  ROS_WARN("No point, skip this scan!\n");
            return;
        }
        
        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);

        V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
        // std::cout << "I am here 14" << std::endl;
        if(0) // If you need to see map point, change to "if(1)"
        {
            PointVector ().swap(ikdtree.PCL_Storage);
            ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
            featsFromMap->clear();
            featsFromMap->points = ikdtree.PCL_Storage;
        }

        pointSearchInd_surf.resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);
        int  rematch_num = 0;
        bool nearest_search_en = true; //

        t2 = omp_get_wtime();

        // std::cout << "I am here 15" << std::endl;
        
        /*** iterated state estimation ***/
        double t_update_start = omp_get_wtime();
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        state_point = kf.get_x();
        euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        geoQuat.x = state_point.rot.coeffs()[0];
        geoQuat.y = state_point.rot.coeffs()[1];
        geoQuat.z = state_point.rot.coeffs()[2];
        geoQuat.w = state_point.rot.coeffs()[3];

        double t_update_end = omp_get_wtime();

        /******* Update odometry *******/
        update_odometry(msg_in);

        /*** add the feature points to map kdtree ***/
        t3 = omp_get_wtime();
        map_incremental();
        t5 = omp_get_wtime();
    }
}

#endif