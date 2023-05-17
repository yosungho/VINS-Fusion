/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/line_geometry.h"

class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
        is_stereo = false;
    }
    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;
    }
    double cur_td;
    Vector3d point, pointRight;
    Vector2d uv, uvRight;
    Vector2d velocity, velocityRight;
    bool is_stereo;
};

class FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;
    int used_num;   // feature size in a sliding window
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

class LineFeaturePerFrame
{
  public:
    LineFeaturePerFrame(const Eigen::Matrix<double, 10, 1> &_line, double td)
    {
        unline = _line.head(6);
        line = _line.tail<4>();

        cur_td = td;
        is_stereo = false;
    }
    void rightObservation(const Eigen::Matrix<double, 10, 1> &_line)
    {
        unlineRight = _line.head(6);
        lineRight = _line.tail<4>();
        is_stereo = true;
    }
    double cur_td;
    Vector6d unline, unlineRight;
    Vector4d line, lineRight;
    bool is_stereo;
};

class LineFeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<LineFeaturePerFrame> linefeature_per_frame;
    int used_num;
    Vector2d estimated_depth;
    bool is_triangulation;
    Vector6d line_plucker;
    Vector3d start;
    Vector3d end;
    int removed_cnt;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    LineFeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0, -1.0), solve_flag(0), removed_cnt(0)
    {
    }

    int endFrame();
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);
    void clearState();
    int getFeatureCount();
    // 두 프레임 사이의 시차를 감지하여 키프레임으로 사용할지 여부를 결정하고, 이전에 감지된 피처 포인트를 피처(list< FeaturePerId >) 컨테이너에 추가하면 각 포인트가 추적된 횟수와 시차가 계산됩니다.
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const map<int, vector<pair<int, Eigen::Matrix<double, 10, 1>>>> &line_image, double td);  
    // 두 프레임 사이의 특징점 관계 가져오기
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth();
    VectorXd getDepthVector();
    void triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                            Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    void triangulateLines(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    double triangulateLine(const Eigen::Matrix<double, 3, 4> &Pose0, const Eigen::Matrix<double, 3, 4> &Pose1,
                            const Vector6d &line0, const Vector6d &line1, Vector6d &line_plucker, Vector6d &line_endpoints);
    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, 
                            vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier(set<int> &outlierIndex);
    list<FeaturePerId> feature;
    list<LineFeaturePerId> linefeature;
    int last_track_num;
    double last_average_parallax;
    int new_feature_num;
    int long_track_num;

    int last_linetrack_num;
    int new_linefeature_num;
    int long_linetrack_num;
    

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);  // 유사한 피처 포인트에 대한 시차 계산

    const Matrix3d *Rs;
    Matrix3d ric[2];
};

#endif