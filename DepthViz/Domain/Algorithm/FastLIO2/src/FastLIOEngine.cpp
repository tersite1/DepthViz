//
//  FastLIOEngine.cpp
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//  Abstract: Implementation of FastLIOEngine with real math logic.
//

#include "FastLIOEngine.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

// Include implementation of ikd-Tree (workaround for missing file in project)
#include "../include/ikd-Tree/ikd_Tree.cpp"

// Helper functions for IKFOM
double omp_get_wtime() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

void omp_set_num_threads(int n) {}
int omp_get_thread_num() { return 0; }

static FastLIOEngine* g_fastLIOEngineInstance = nullptr;

static void h_share_model_wrapper(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    if (g_fastLIOEngineInstance) {
        g_fastLIOEngineInstance->h_share_model(s, ekfom_data);
    }
}

FastLIOEngine::FastLIOEngine() : _isRunning(false), _firstLidar(true) {
    g_fastLIOEngineInstance = this;
    _statePoint = state_ikfom(); // Reset state
    
    _featsFromMap.reset(new PointCloudXYZI());
    _featsUndistort.reset(new PointCloudXYZI());
    _featsDownBody.reset(new PointCloudXYZI());
    _featsDownWorld.reset(new PointCloudXYZI());
    _normVec.reset(new PointCloudXYZI(100000, 1));
    _laserCloudOri.reset(new PointCloudXYZI(100000, 1));
    _corrNormVect.reset(new PointCloudXYZI(100000, 1));
    
    _preProcessor = std::shared_ptr<Preprocess>(new Preprocess());
    _imuProcessor = std::shared_ptr<ImuProcess>(new ImuProcess());
    
    _preProcessor->feature_enabled = false;
    _preProcessor->point_filter_num = 2;
    _preProcessor->blind = 0.01;
    
    _currentPose = Eigen::Matrix4d::Identity();
    
    // Resize vectors
    _pointSearchIndSurf.resize(100000); // Max points
    _nearestPoints.resize(100000);
}

FastLIOEngine::~FastLIOEngine() {
    stop();
}

void FastLIOEngine::init() {
    double epsi[23] = {0.001};
    std::fill(epsi, epsi+23, 0.001);
    
    _kf.init_dyn_share(get_f, df_dx, df_dw, 
                       h_share_model_wrapper, 
                       kMaxIterations, epsi);
                       
    _ikdTree.set_downsample_param(0.5); // filter_size_map_min
}

void FastLIOEngine::start() {
    if (_isRunning) return;
    _isRunning = true;
    _processThread.reset(new std::thread(&FastLIOEngine::run, this));
}

void FastLIOEngine::stop() {
    _isRunning = false;
    if (_processThread && _processThread->joinable()) {
        _processThread->join();
    }
}

void FastLIOEngine::pushIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
    std::lock_guard<std::mutex> lock(_mtxBuffer);
    
    custom_messages::ImuPtr msg(new custom_messages::Imu());
    msg->header.stamp.fromSec(timestamp);
    
    msg->linear_acceleration.x = acc.x();
    msg->linear_acceleration.y = acc.y();
    msg->linear_acceleration.z = acc.z();
    
    msg->angular_velocity.x = gyr.x();
    msg->angular_velocity.y = gyr.y();
    msg->angular_velocity.z = gyr.z();
    
    _imuBuffer.push_back(msg);
}

void FastLIOEngine::pushPointCloud(double timestamp, const std::vector<AppPoint3D>& points) {
    std::lock_guard<std::mutex> lock(_mtxBuffer);
    
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    ptr->reserve(points.size());
    
    for (const auto& p : points) {
        PointType pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = p.intensity;
        ptr->push_back(pt);
    }
    
    _lidarBuffer.push_back(ptr);
    _timeBuffer.push_back(timestamp);
}

bool FastLIOEngine::syncPackages(MeasureGroup &meas) {
    if (_lidarBuffer.empty() || _imuBuffer.empty()) {
        return false;
    }

    if (!_measures.lidar_beg_time) {
        meas.lidar = _lidarBuffer.front();
        meas.lidar_beg_time = _timeBuffer.front();
        
        double lidar_mean_scantime = 0.1; 
        meas.lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime; 
        
        _measures.lidar_beg_time = meas.lidar_beg_time;
    }

    if (_imuBuffer.back()->header.stamp.toSec() < meas.lidar_end_time) {
        return false;
    }

    double imu_time = _imuBuffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while (!_imuBuffer.empty() && imu_time < meas.lidar_end_time) {
        imu_time = _imuBuffer.front()->header.stamp.toSec();
        if (imu_time > meas.lidar_end_time) break;
        meas.imu.push_back(_imuBuffer.front());
        _imuBuffer.pop_front();
    }

    _lidarBuffer.pop_front();
    _timeBuffer.pop_front();
    _measures.lidar_beg_time = 0;
    
    return true;
}

void FastLIOEngine::run() {
    while (_isRunning) {
        bool hasData = false;
        {
            std::lock_guard<std::mutex> lock(_mtxBuffer);
            hasData = syncPackages(_measures);
        }
        
        if (!hasData) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        
        if (_firstLidar) {
            _firstLidar = false;
            _firstLidarTime = _measures.lidar_beg_time;
            _imuProcessor->first_lidar_time = _firstLidarTime;
            continue;
        }
        
        _imuProcessor->Process(_measures, _kf, _featsUndistort);
        _statePoint = _kf.get_x();
        
        {
             std::lock_guard<std::mutex> lock(_mtxPose);
             Eigen::Matrix3d rot = _statePoint.rot.toRotationMatrix();
             Eigen::Vector3d pos = _statePoint.pos;
             
             _currentPose.setIdentity();
             _currentPose.block<3,3>(0,0) = rot;
             _currentPose.block<3,1>(0,3) = pos;
        }

        if (_featsUndistort->empty()) continue;
        
        _downSizeFilterSurf.setInputCloud(_featsUndistort);
        _downSizeFilterSurf.filter(*_featsDownBody);
        
        int feats_down_size = _featsDownBody->points.size();
        
        if (_ikdTree.Root_Node == nullptr) {
            if (feats_down_size > 5) {
                _featsDownWorld->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++) {
                    pointBodyToWorld(&(_featsDownBody->points[i]), &(_featsDownWorld->points[i]));
                }
                _ikdTree.Build(_featsDownWorld->points);
            }
            continue;
        }

        double solve_H_time = 0;
        _kf.update_iterated_dyn_share_modified(0.001, solve_H_time);
        _statePoint = _kf.get_x();

        mapIncremental();
        
        {
            std::lock_guard<std::mutex> lock(_mtxCloud);
            _displayCloud.clear();
            _displayCloud.reserve(_featsUndistort->points.size());
            for (const auto& pt : _featsUndistort->points) {
                 AppPoint3D p;
                 p.x = pt.x; p.y = pt.y; p.z = pt.z; p.intensity = pt.intensity;
                 _displayCloud.push_back(p);
            }
        }
    }
}

void FastLIOEngine::mapIncremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    
    int feats_down_size = _featsDownBody->points.size();
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++) {
        pointBodyToWorld(&(_featsDownBody->points[i]), &(_featsDownWorld->points[i]));
        
        if (!_nearestPoints[i].empty()) {
            const PointVector &points_near = _nearestPoints[i];
            bool need_add = true;
            
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            
            float filter_size_map_min = 0.5; // Fixed value for map resolution
            
            mid_point.x = floor(_featsDownWorld->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(_featsDownWorld->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(_featsDownWorld->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(_featsDownWorld->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(_featsDownWorld->points[i]);
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
            if (need_add) PointToAdd.push_back(_featsDownWorld->points[i]);
        } else {
            PointToAdd.push_back(_featsDownWorld->points[i]);
        }
    }
    
    _ikdTree.Add_Points(PointToAdd, true);
    _ikdTree.Add_Points(PointNoNeedDownsample, false);
}

void FastLIOEngine::h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    
    int feats_down_size = _featsDownBody->points.size();
    _laserCloudOri->clear();
    _corrNormVect->clear();
    
    if (_nearestPoints.size() < feats_down_size) _nearestPoints.resize(feats_down_size);
    if (_pointSearchIndSurf.size() < feats_down_size) _pointSearchIndSurf.resize(feats_down_size);
    
    int effct_feat_num = 0;
    
    for (int i = 0; i < feats_down_size; i++) {
        PointType &point_body  = _featsDownBody->points[i];
        PointType &point_world = _featsDownWorld->points[i];
        
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;
        
        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &points_near = _nearestPoints[i];
        
        if (ekfom_data.converge) {
            _ikdTree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
        }
        
        VF(4) pabcd;
        bool point_selected = false;
        if (esti_plane(pabcd, points_near, 0.1f)) // 0.1 threshold
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s_val = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s_val > 0.9)
            {
                point_selected = true;
                _normVec->points[i].x = pabcd(0);
                _normVec->points[i].y = pabcd(1);
                _normVec->points[i].z = pabcd(2);
                _normVec->points[i].intensity = pd2;
            }
        }
        
        if (point_selected) {
            _laserCloudOri->points[effct_feat_num] = _featsDownBody->points[i];
            _corrNormVect->points[effct_feat_num] = _normVec->points[i];
            effct_feat_num ++;
        }
    }
    
    if (effct_feat_num < 1) {
        ekfom_data.valid = false;
        return;
    }
    
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12); 
    ekfom_data.h.resize(effct_feat_num);
    
    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = _laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const PointType &norm_p = _corrNormVect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        
        V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
        ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);

        ekfom_data.h(i) = -norm_p.intensity;
    }
}

Eigen::Matrix4d FastLIOEngine::getPose() {
    std::lock_guard<std::mutex> lock(_mtxPose);
    return _currentPose;
}

std::vector<AppPoint3D> FastLIOEngine::getDisplayCloud() {
    std::lock_guard<std::mutex> lock(_mtxCloud);
    return _displayCloud;
}

void FastLIOEngine::pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(_statePoint.rot * (_statePoint.offset_R_L_I*p_body + _statePoint.offset_T_L_I) + _statePoint.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
