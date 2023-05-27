/*******************************************************
 * Copyright (C) 2023
 * 
 * This file is part of PL-VINS-Fusion.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Sungho Yoon (rp.sungho@gmail.com)
 *******************************************************/

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <opencv2/line_descriptor.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include "../base/linebase.h"
#include "linedet/lsd.h"
#include "linedet/include/LineBandDescriptor.h"
#include "linedet/include/PairwiseLineMatching.h"
#include "linedet/include/EDLineDetector.h"
using namespace std;
using namespace camodocal;
using namespace Eigen;
typedef cv::line_descriptor::KeyLine KeyLine;

// struct CVKeyLine : public cv::line_descriptor::KeyLine {
//     // KeyLine 객체를 입력 받아 CVKeyLine으로 변환하는 생성자
//     CVKeyLine(const cv::line_descriptor::KeyLine& keyLine) {
//         // KeyLine의 멤버들을 CVKeyLine에 복사
//         this->angle = keyLine.angle;
//         this->class_id = keyLine.class_id;
//         this->octave = keyLine.octave;
//         this->pt = keyLine.pt;
//         this->response = keyLine.response;
//         this->size = keyLine.size;
//         this->startPointInImage = keyLine.startPointInImage;
//         this->endPointInImage = keyLine.endPointInImage;
//         this->startPointInOctave = keyLine.startPointInOctave;
//         this->endPointInOctave = keyLine.endPointInOctave;
//         this->lineLength = keyLine.lineLength;
//     }

//     // CVKeyLine에서 KeyLine 형태의 데이터만 추출하는 메서드
//     cv::line_descriptor::KeyLine toKeyLine() const {
//         cv::line_descriptor::KeyLine keyLine;
//         keyLine.angle = this->angle;
//         keyLine.class_id = this->class_id;
//         keyLine.octave = this->octave;
//         keyLine.pt = this->pt;
//         keyLine.response = this->response;
//         keyLine.size = this->size;
//         keyLine.startPointInImage = this->startPointInImage;
//         keyLine.endPointInImage = this->endPointInImage;
//         keyLine.startPointInOctave = this->startPointInOctave;
//         keyLine.endPointInOctave = this->endPointInOctave;
//         keyLine.lineLength = this->lineLength;
//         return keyLine;
//     }
// };

class FrameLines
{
public:
    int camera_id;
    float time;         // frame time
    cv::Mat img;        // for debug
    cv::Mat img1;
    
    std::vector<int> line_ids;
    std::vector<std::vector<KeyLine>> klines;
    std::vector<std::vector<cv::Mat>> descs;
};
typedef shared_ptr< FrameLines > FrameLinesPtr;

class LineFeatureTracker
{
public:
    LineFeatureTracker();
    map<int, vector<pair<int, Eigen::Matrix<double, 10, 1>>>> trackImage(double _cur_time, const cv::Mat &_img);
    map<int, vector<pair<int, Eigen::Matrix<double, 10, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1);
    void detectLines(const cv::Mat &image, vector<vector<KeyLine>> &detected_lines);
    void describeLines(const cv::Mat &image, vector<vector<KeyLine>> &lines, std::vector<std::vector<cv::Mat>> &descs);
    // std::vector<std::vector<KeyLine>> to_cv(const vector<Line2d> &lines);
    void readIntrinsicParameter(const vector<string> &calib_file);
    void filter_matches_by_angularVelocity(float dt, const std::vector<std::vector<KeyLine>> &prev_lines, const std::vector<std::vector<KeyLine>> &cur_lines, std::vector<std::pair<uint32_t, uint32_t>> &goodMatches);
    void visualizeLines(cv::Mat &image, vector<KeyLine> &lines);
    void visualizeLineMatches(const cv::Mat &cvLeftImage, const cv::Mat &cvRightImage,
                  const std::vector<std::vector<KeyLine>> &linesInLeft, const std::vector<std::vector<KeyLine>> &linesInRight,
                  const std::vector<std::pair<uint32_t, uint32_t>> &goodMatches);
    void drawTrack(const cv::Mat &cvLeftImage, const cv::Mat &cvRightImage,
                  const std::vector<std::vector<KeyLine>> &linesInLeft, const std::vector<std::vector<KeyLine>> &linesInRight,
                  const std::vector<std::pair<uint32_t, uint32_t>> &goodMatches);
    cv::Mat getTrackImage();
    int row, col;
    std::map<uint64_t, int> num_tracked;    // id, tracked_num
    bool is_first_frame = true;
    uint64_t allfeature_id = 0; // -1 : not assigned
    cv::Mat imTrack;
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat image_track_;
    FrameLinesPtr prev_frame_, cur_frame_, cur_right_frame_;


    // cv::Mat prev_img, cur_img;
    // vector<vector<KeyLine>> prev_lines, cur_lines, cur_right_lines;
    // std::vector<std::vector<cv::Mat>> prev_descs, cur_descs, cur_right_descs;
    // vector<int> ids, ids_right;

    vector<camodocal::CameraPtr> camera_;
    vector<pair<cv::Point2f, cv::Point2f>> undistortedLineEndPoints(const vector<vector<KeyLine>> &klines, const camodocal::CameraPtr cam);
    vector<cv::Mat> K_, undist_map1_, undist_map2_;
    double cur_time;
    double prev_time;
    bool stereo_cam;
    int n_id;
    bool hasPrediction; // what kind of prediction??

private:
    eth::LineBandDescriptor line_desc;
    eth::PairwiseLineMatching line_match;
};
