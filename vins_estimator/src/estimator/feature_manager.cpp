/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"

int LineFeaturePerId::endFrame()
{
    return start_frame + linefeature_per_frame.size() - 1;
}

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 4)
        {
            cnt++;
        }
    }
    return cnt;
}

                                                                 // featureId       cameraId, point
bool FeatureManager::addFeatureCheckParallax(const int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, 
                                                                const map<int, vector<pair<int, Eigen::Matrix<double, 10, 1>>>> &line_image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("input line feature: %d", (int)line_image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount()); // 기존 feature 수
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_linetrack_num = 0;
    last_average_parallax = 0;
    new_feature_num = 0;
    new_linefeature_num = 0;
    long_track_num = 0;

    // Point
    for (auto &id_pts : image)  // image frame에 있는 각 feature를 하나씩 살펴보자.
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td); // FeaturePerFrame에는 각 feature의 데이터를 저장함. 데이터: (xyz_uv_vel)와 td(0)
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)   // stereo 케이스 (camera_id가 2개인 경우)
        {
            f_per_fra.rightObservation(id_pts.second[1].second);
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        // feature 멤버변수에서 feature_id와 동일한 것이 있는지 확인. (feature 멤버변수는 list<FeaturePerId>로 정의되어 있음.)
        // ((feature 멤버변수는 서로 다른 id로 나뉜 feature(FeaturePerId)들의 모음. list > id > vector<FeaturePerFrame>))
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end()) // feature에 등록되어 있지 않은 이전에 없던 feature_id가 들어왔다면,
        {
            // feature.push_back(FeaturePerId(feature_id, frame_count));   // 새로운 FeaturePerId 생성; FeaturePerId는 고유 id를 가진 feature들의 모집합. start_frame을 전달해준다.
            feature.emplace_back(feature_id, frame_count);   // 새로운 FeaturePerId 생성; FeaturePerId는 고유 id를 가진 feature들의 모집합. start_frame을 전달해준다.
            feature.back().feature_per_frame.push_back(f_per_fra);      // 새로 만들어 넣은 객체 (.back())에 있는 vector에 신규 feature를 추가함. (신규 feature: FeaturePerFrame형태의 feature 정보가 있는 객체)
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)  // feature이 이미 등록된 feature_id였다면, 
        {
            it->feature_per_frame.push_back(f_per_fra); // 해당 feature_id를 가리키는 feature_vector에 추가한다.
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)     // feature track이 4개 이상 연속되면 long_track_num을 증가시킴.
                long_track_num++;
        }
    }

    // Line
    for (auto &id_line: line_image) {
        int feature_id = id_line.first;
        int cameraId = id_line.second[0].first;
        assert(cameraId == 0);
        auto line_data = id_line.second[0].second;

        LineFeaturePerFrame feat_per_frame(line_data, td); // FeaturePerFrame에는 각 feature의 데이터를 저장함. 데이터: (xyz_uv_vel)와 td(time offset=0(default))
        if(id_line.second.size() == 2)   // stereo 케이스 (camera_id가 2개인 경우)
        {
            int right_cameraId = id_line.second[1].first;
            auto right_line_data = id_line.second[1].second;

            feat_per_frame.rightObservation(right_line_data);
            assert(right_cameraId == 1);
        }
        
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const LineFeaturePerId &it) {return it.feature_id == feature_id;});

        if (it == linefeature.end()) // feature에 등록되어 있지 않은 이전에 없던 feature_id가 들어왔다면,
        {
            // linefeature.push_back(LineFeaturePerId(feature_id, frame_count));   // 새로운 FeaturePerId 생성; FeaturePerId는 고유 id를 가진 feature들의 모집합. start_frame을 전달해준다.
            linefeature.emplace_back(feature_id, frame_count);   // 새로운 FeaturePerId 생성; FeaturePerId는 고유 id를 가진 feature들의 모집합. start_frame을 전달해준다.
            linefeature.back().linefeature_per_frame.push_back(feat_per_frame);      // 새로 만들어 넣은 객체 (.back())에 있는 vector에 신규 feature를 추가함. (신규 feature: FeaturePerFrame형태의 feature 정보가 있는 객체)
            new_linefeature_num++;
        }
        else if (it->feature_id == feature_id)  // feature이 이미 등록된 feature_id였다면, 
        {
            it->linefeature_per_frame.push_back(feat_per_frame); // 해당 feature_id를 가리키는 feature_vector에 추가한다.
            last_linetrack_num++;
            if( it-> linefeature_per_frame.size() >= 4)     // feature track이 4개 이상 연속되면 long_track_num을 증가시킴.
                long_linetrack_num++;
        }
    }
    
    // keyFrame인지 확인하는 기준:
    //if (frame_count < 2 || last_track_num < 20)
    //if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 * last_track_num)
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num) 
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);   // 유사한 피처 포인트에 대한 시차 계산
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;   // solve fail
        }
        else
            it_per_id.solve_flag = 1;   // solve success
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}


void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);     // Pose0: c_T_w = projection matrix
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);     
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  

    // reference: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
    // If true (1), the function uses the provided rvec and tvec values as initial approximations of the rotation and translation vectors, respectively, and further optimizes them.
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);    
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);


    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}

void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{

    if(frameCnt > 0)
    {
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D;
        for (auto &it_per_id : feature)
        {   
            // feature에 추정된 depth값이 있으면,
            if (it_per_id.estimated_depth > 0)  
            {
                int index = frameCnt - it_per_id.start_frame;
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];   // body_t_feat
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];     // world_R_body @ body_t_feat + world_t_body

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
                    pts3D.push_back(point3d);   // world_t_feat
                    pts2D.push_back(point2d);   // uv on an image
                }
            }
        }
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam
        RCam = Rs[frameCnt - 1] * ric[0];   // RCam = world_R_camera = world_R_body @ body_R_camera
        PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];    // PCam = world_T_camera = (world_R_body @ body_t_camera) + world_t_body

        if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
        {
            // trans to w_T_imu
            Rs[frameCnt] = RCam * ric[0].transpose(); // w_R_cam @ (body_R_cam)^(-1) = w_R_body
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;  // (-w_R_body @ body_t_camExtrinsic) + w_t_cam

            Eigen::Quaterniond Q(Rs[frameCnt]);
            //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
    }
}

void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    /**
     * output: it_per_id.estimated_depth에 추가..
    */
    for (auto &it_per_id : feature)
    {
        if (it_per_id.estimated_depth > 0)
            continue;

        if(STEREO && it_per_id.feature_per_frame[0].is_stereo)  // STREO 세팅이고 좌우측 카메라에서 모두 보이는 feat이면,
        {
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];    // twc = twi + Rwi * tic
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];                // Rwc = Rwi * Ric
            leftPose.leftCols<3>() = R0.transpose();            // w_R_c -> c_R_w
            leftPose.rightCols<1>() = -R0.transpose() * t0;     // w_t_c -> c_t_w
            //cout << "left pose " << leftPose << endl;

            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;
            //cout << "right pose " << rightPose << endl;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);      // normalized xyz where z=1 && feature_id 하나당 1개의 point가 있더라
            point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
            //cout << "point0 " << point0.transpose() << endl;
            //cout << "point1 " << point1.transpose() << endl;

            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();    // c_R_w @ point3d_w + c_t_w
            double depth = localPoint.z();  // startFrame에서의 depth
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH; // 5 meter
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        else if(it_per_id.feature_per_frame.size() > 1)
        {
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];    // twc = twi + Rwi * tic
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];                // Rwc = Rwi * Ric
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;         // leftPose: tcw, Rcw

            imu_i++;    // 다음 frame...
            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[1].point.head(2);
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            
        }
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        // 동일 id의 feature가 4개 이상인 경우, (왜 이 케이스는 안나오는걸까? KITTI(Stereo+Non-IMU)라는 특성 때문일까? 그건 아닌듯.)
        // (it_per_id.feature_per_frame.size()>1의 경우 수에 다 들어가기 때문에 앞에 continue에 다 포함되어 >=4 케이스가 나올 수가 없음.)
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }
    }
}

/**
 * point는 2개 이상일 때 Ax=0 꼴로 다 합친담에 3D좌표를 linear least-square로 풀고 있다.
 * 한번 linear least-square로 풀면, 이후에는 non-linear least-square로만 handling한다.
 * 
 * line은 5개 이상일 때 각각의 angle-bw planes를 측정해서 제일 parallax를 큰 좌표를 triangulation한다.
 * 이후에는 non-linear optimization만 돌리도록 되어 있다. PL-VINS에서..
*/
void FeatureManager::triangulateLines(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    const int LINE_MIN_OBS = 5;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2))   // 현재 특징이 2개 이상의 프레임에서 관찰되었고, 최근 두 번째 프레임 이전에 처음 관찰되었다면 삼각화를 시도합니다. 그렇지 않으면 다음 특징으로 넘어갑니다.
            continue;

        if (it_per_id.is_triangulation)
            continue;

        int imu_i = it_per_id.start_frame;
        int imu_j = imu_i - 1;
        Eigen::Matrix<double, 3, 4> pose0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];    // twc = twi + Rwi * tic    // Ps는 기본적으로 twi인데, 이를 imu-camera extrinsic(tic, Ric; 0:leftcam, 1:rightcam)을 반영하여 twc로 바꾸는 과정..
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];                // Rwc = Rwi * Ric
        pose0.leftCols<3>() = R0.transpose();
        pose0.rightCols<1>() = -R0.transpose() * t0;         // pose0: tcw, Rcw
        // pose0.leftCols<3>() = R0;
        // pose0.rightCols<1>() = t0;         // pose0: twc, Rwc
        Vector6d line0, line1;       // two endpoints

        double min_cos_theta = 1.0;
        Vector6d bestline_plucker; 
        Vector6d bestline_endpoints;

        // 같은 id를 가진 모든 관찰을 순회합니다. start_frame도 순회됩니다.
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)   
        {
            imu_j++;    // 다음 frame...
            if (imu_j>WINDOW_SIZE) {
                continue;
            }
            if (imu_j == imu_i)   // 첫 번째 관찰은 시작 프레임에 있습니다.
            {
                line0 = it_per_frame.unline;
                continue;
            }

            Eigen::Matrix<double, 3, 4> pose1;
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];    // twc_query
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];                // Rwc_query
            pose1.leftCols<3>() = R1.transpose();                   // tcw
            pose1.rightCols<1>() = -R1.transpose() * t1;            // Rcw -> T_cw

            // std::cout << "feat_id: " << it_per_id.feature_id << std::endl;
            // std::cout << "t0: " << t0.transpose() << std::endl;
            // std::cout << "t1: " << t1.transpose() << std::endl<< std::endl;
            // pose1.leftCols<3>() = R1;                   // twc
            // pose1.rightCols<1>() = t1;            // Rwc -> T_wc
            line1 = it_per_frame.unline;    // normalized xyz where z=1 && feature_id 하나당 1개의 point가 있더라

            Vector6d line_plucker0; 
            Vector6d line_endpoints0;
            double cos_theta = triangulateLine(pose0, pose1, line0, line1, line_plucker0, line_endpoints0);

            if (cos_theta < min_cos_theta) {
                min_cos_theta = cos_theta;
                bestline_plucker = line_plucker0;
                bestline_endpoints = line_endpoints0;
            }
        }

        if(min_cos_theta > 0.998)   // 3.62 deg 이상의 parallax만 유효
            continue;

        it_per_id.line_plucker = bestline_plucker;
        it_per_id.start = bestline_endpoints.head<3>();
        it_per_id.end = bestline_endpoints.tail<3>();
        double length = (it_per_id.end - it_per_id.start).norm();
        // std::cout << "length: " << length << std::endl;
        // if (length > 100) {
        //     std::cout << "length > 100: " << length << std::endl;
        //     it_per_id.is_triangulation = false;
        // }
        if (isnan(it_per_id.start(0))) {
            it_per_id.is_triangulation = false;

            std::cout <<"------------"<<std::endl;
            std::cout << bestline_plucker << "\n\n";
            std::cout << length <<"\n\n";
            std::cout << it_per_id.start <<"\n\n";
            std::cout << it_per_id.end <<std::endl;
        }
        else {
            it_per_id.is_triangulation = true;
        }
        

        // Vector3d point3d_st = bestline_endpoints.head<3>();
        // Vector3d point3d_ed = bestline_endpoints.tail<3>();
        // Eigen::Vector3d localPoint_st, localPoint_ed;
        // localPoint_st = pose0.leftCols<3>() * point3d_st + pose0.rightCols<1>();    // c_R_w @ point3d_w + c_t_w
        // localPoint_ed = pose0.leftCols<3>() * point3d_ed + pose0.rightCols<1>();    // c_R_w @ point3d_w + c_t_w
        // double depth_st = localPoint_st.z();  // startFrame에서의 depth
        // double depth_ed = localPoint_ed.z();  

        // if (depth_st > 0 && depth_ed > 0) {
        //     it_per_id.estimated_depth(0) = depth_st;
        //     it_per_id.estimated_depth(1) = depth_ed;
        //     it_per_id.is_triangulation = true;
        // }
        // else {
        //     it_per_id.estimated_depth(0) = -1; //INIT_DEPTH; // 5 meter
        //     it_per_id.estimated_depth(1) = -1; //INIT_DEPTH;
        //     it_per_id.is_triangulation = false;
        // }
    }
}

double FeatureManager::triangulateLine(const Eigen::Matrix<double, 3, 4> &Pose0, const Eigen::Matrix<double, 3, 4> &Pose1,
                            const Vector6d &line0, const Vector6d &line1, Vector6d &line_plucker, Vector6d &line_endpoints) {
    /**
     * Pose0: Reference frame
     * Pose1: Query frame
     * return: parallex in degree
    */
    Eigen::Vector3d t0 = Pose0.rightCols<1>();  // t_wc0
    Eigen::Matrix3d R0 = Pose0.leftCols<3>();
    Eigen::Vector3d t1 = Pose1.rightCols<1>();  // t_wc1
    Eigen::Matrix3d R1 = Pose1.leftCols<3>();
    Eigen::Vector3d t01 = R0.transpose() * (t1 - t0);   // relative pose: 0_t_1
    Eigen::Matrix3d R01 = R0.transpose() * R1;          // relative pose: 0_R_1

    Eigen::Vector3d pt0( line0(0), line0(1), line0(2) );
    Eigen::Vector3d pt1( line0(3), line0(4), line0(5) );
    Vector4d pi_ref = pi_from_ppp(pt0, pt1,Vector3d( 0, 0, 0 ));
    Eigen::Vector3d n_ref = pi_ref.head(3); n_ref.normalize();

    Vector3d pt2( line1(0), line1(1), line1(2) );
    Vector3d pt3( line1(3), line1(4), line1(5) );
    pt2 = R01 * pt2 + t01;  // line1 wrt coordinate 0
    pt3 = R01 * pt3 + t01;
    Vector4d pi_query = pi_from_ppp(pt2, pt3, t01);
    Eigen::Vector3d n_query = pi_query.head(3); n_query.normalize(); 

    double ang_cosine = n_ref.dot(n_query);
    line_plucker = pipi_plk( pi_ref, pi_query ); // line_3d: plucker coordinate

    // endpoint trimming
    Vector3d mc = line_plucker.head(3);
    Vector3d dc = line_plucker.tail(3);
    
    Matrix4d plucker_mat;    // plucker matrix
    plucker_mat << skew_symmetric(mc), dc, -dc.transpose(), 0;

    Vector2d ln = ( pt0.cross(pt1) ).head(2);     // 선의 세로 방향
    ln = ln / ln.norm();

    Vector3d pt0e = Vector3d(pt0(0) + ln(0), pt0(1) + ln(1), 1.0);  // 선의 세로 방향으로 한 단위 이동
    Vector3d pt1e = Vector3d(pt1(0) + ln(0), pt1(1) + ln(1), 1.0);
    Vector3d cam = Vector3d( 0, 0, 0 );

    Vector4d pi0 = pi_from_ppp(cam, pt0, pt0e);
    Vector4d pi1 = pi_from_ppp(cam, pt1, pt1e);

    Vector4d P0 = plucker_mat * pi0;
    Vector4d P1 = plucker_mat * pi1;
    P0 = P0/P0(3);
    P1 = P1/P1(3);

    line_endpoints.head(3) = Vector3d{P0(0),P0(1),P0(2)};
    line_endpoints.tail(3) = Vector3d{P1(0),P1(1),P1(2)};

    return ang_cosine;
}

void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            feature.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == 0)
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        else {
            it->start_frame--;
        }
        
        
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }

    for (auto it = linefeature.begin(), it_next = linefeature.begin(); it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == 0)    // 기능이 이 프레임에서 초기화되지 않은 경우 걱정하지 말고 ID만 신경쓰면 됩니다.
        {
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());  // 관찰 제거
            if (it->linefeature_per_frame.size() < 2)                     // 이 프레임의 이미지가 두 프레임 미만으로 관찰되면 이 기능이 필요하지 않습니다.
            {
                linefeature.erase(it);
                continue;
            }
            else  // 피처가 더 많은 프레임에 표시되고 피처의 초기화 프레임에 마진이 있는 경우 피처를 다음 프레임으로 전송해야 하며, 여기서 marg_R, new_R은 해당 순간에 카메라 좌표계에서 월드 좌표계로의 변환입니다.
            {
                it->removed_cnt++;
                // transpose this line to the new pose
                Matrix3d Rji = new_R.transpose() * marg_R;     // Rcjw * Rwci where i: old, j: new
                Vector3d tji = new_R.transpose() * (marg_P - new_P);
                Vector6d plk_j = plk_to_pose(it->line_plucker, Rji, tji);
                it->line_plucker = plk_j;
            }
        }
        else
        {
            it->start_frame--;
        }
    }
}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == 0)
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
            {
                feature.erase(it);
            }
        }
        else 
        {
            it->start_frame--;
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }

    for (auto it = linefeature.begin(), it_next = linefeature.begin(); it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)  // 가장 최근 프레임에서 뽑힌 linefeature를 가장 앞 slide(frame_count-1)에 배치한다.
        {
            it->start_frame = frame_count-1;
        }
        else
        {
            int j = (WINDOW_SIZE - 1) - it->start_frame;      // 이 it의 frame이 최신 대비 얼마나 앞서서 뽑혔는지?
            if (it->endFrame() < frame_count - 1)           // it의 feature가 현재까지 track으로 이어져오고 있지 않는다면 continue; 
                continue;
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin() + j);   // track 으로 이어져오고 있다면
            if (it->linefeature_per_frame.size() == 0)                            // 다른 이미지 프레임에서 이 기능을 볼 수 없으면 삭제하세요.
                linefeature.erase(it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}