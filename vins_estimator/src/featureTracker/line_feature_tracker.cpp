/*******************************************************
 * Copyright (C) 2019, Sungho Yoon
 * 
 * This file is part of PL-VINS-Fusion.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Sungho Yoon (rp.sungho@gmail.com)
 *******************************************************/

#include "line_feature_tracker.h"
#include <ctime>

cv::Scalar idToColor(int id) {
    int b = (id * 23) % 256; // Blue: 0-255
    int g = (id * 123) % 256; // Green: 0-255
    int r = (id * 231) % 256; // Red: 0-255
    return cv::Scalar(b, g, r);
}

void reduceVector(vector<KeyLine> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

LineFeatureTracker::LineFeatureTracker()
{
    stereo_cam = 0;
    n_id = 0;
    hasPrediction = false;
}

void LineFeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading parameter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        camera_.push_back(camera);

        cv::Mat undist_map1,undist_map2;
        cv::Mat K = camera->initUndistortRectifyMap(undist_map1,undist_map2);
        K_.push_back(K);
        undist_map1_.push_back(undist_map1);
        undist_map2_.push_back(undist_map2);        
    }
    if (calib_file.size() == 2)
        stereo_cam = 1;
}

vector<pair<cv::Point2f, cv::Point2f>> LineFeatureTracker::undistortedLineEndPoints(const vector<vector<KeyLine>> &klines, const camodocal::CameraPtr cam)
{
    vector<pair<cv::Point2f, cv::Point2f>> un_lines;
    for (size_t i = 0; i <klines.size(); i++)
    {
        Eigen::Vector2d pt_st(klines[i][0].startPointX, klines[i][0].startPointY);
        Eigen::Vector3d unpt_st;
        cam->liftProjective(pt_st, unpt_st);

        Eigen::Vector2d pt_ed(klines[i][0].endPointX, klines[i][0].endPointY);
        Eigen::Vector3d unpt_ed;
        cam->liftProjective(pt_ed, unpt_ed);
        un_lines.emplace_back(cv::Point2f(unpt_st.x() / unpt_st.z(), unpt_st.y() / unpt_st.z()), cv::Point2f(unpt_ed.x() / unpt_ed.z(), unpt_ed.y() / unpt_ed.z()));
    }
    return un_lines;
}

map<int, vector<pair<int, Eigen::Matrix<double, 10, 1>>>> LineFeatureTracker::trackImage(double _cur_time, const cv::Mat &_img) {
    return trackImage(_cur_time, _img, _img);
}

map<int, vector<pair<int, Eigen::Matrix<double, 10, 1>>>> LineFeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1) {
    /**
     * 
     * output: feature_id, vector<camera_id, xyz_uv_velocity> where xyz_uv_velocity has two endpoints(10) and orthogonal velocity(1)
    */

    if (prev_frame_ == nullptr) // 시스템 초기화의 첫 번째 프레임 이미지
    {
        prev_frame_.reset(new FrameLines);
        prev_frame_->time = _cur_time;
        cur_frame_.reset(new FrameLines);
        cur_frame_->time = _cur_time;

        prev_frame_->img = _img;
        prev_frame_->img = _img;
        cur_frame_->img1 = _img1;
        cur_frame_->img1 = _img1;
    }
    else
    {
        cur_frame_.reset(new FrameLines);  // 새 프레임 초기화
        cur_frame_->time = _cur_time;
        cur_frame_->img = _img;
        cur_frame_->img1 = _img1;
    }

    TicToc t_r;
    const cv::Mat &cur_img = _img;
    const cv::Mat &rightImg = _img1;
    row = cur_img.rows;
    col = cur_img.cols;
    vector<vector<KeyLine>> &prev_lines = prev_frame_->klines;
    vector<vector<KeyLine>> &cur_lines = cur_frame_->klines;
    vector<vector<KeyLine>> &cur_right_lines = cur_right_frame_->klines;
    vector<vector<cv::Mat>> &prev_descs = prev_frame_->descs;
    vector<vector<cv::Mat>> &cur_descs = cur_frame_->descs;
    vector<vector<cv::Mat>> &cur_right_descs = cur_right_frame_->descs;

    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */

    // TODO: 현재는 이전 frame과 지금 frame과의 비교만 하는데, 이전이전 frame에서 매칭 안된 선분의 id와 descriptor는 살려서 이전이전과 지금도 매칭 후보에 들어갈 수 있게 하면 track이 덜 끊길 듯하다.
    // 그것이 n개 전까지 고려되도록 하면 더 track이 안끊길 듯.. 이 때 n은 descriptor의 성능에 의존되게 됨.
    detectLines(cur_img, cur_lines);
    describeLines(cur_img, cur_lines, cur_descs);

    std::vector<std::pair<uint32_t, uint32_t>> match_results;
    if (prev_lines.size() > 0) {
        line_match.matchLines(prev_lines, cur_lines, prev_descs, cur_descs, match_results);

        // hand-crafted outlier rejection
        float dt = cur_frame_->time - prev_frame_->time;
        filter_matches_by_angularVelocity(dt, prev_lines, cur_lines, match_results);    

        vector<uchar> status;
        for (auto match:match_results) {
            int prev_id = prev_lines[match.first][0].class_id;
            cur_lines[match.second][0].class_id = prev_id;
            num_tracked[prev_id]++;
        }
    }

    for (size_t i = 0; i < cur_lines.size(); i++) {
        if (cur_lines[i][0].class_id == 0) {
            cur_lines[i][0].class_id = allfeature_id++;
        }
    }

    if(SHOW_TRACK){
        drawTrack(prev_frame_->img, cur_img, prev_lines, cur_lines, match_results);
        // visualizeLineMatches(prev_frame_->img, cur_img, prev_lines, cur_lines, match_results);
    }
    
    // {
        /**
         * rejectWithF();
         * setMask();
         * if (n_max_cnt > 0) 
         * cv::goodFeaturesToTrack
         * undistortedPts()
         * ptsVelocity()
        */
    // }

    // making return values
    // key: feature_id ; value: camera_id, xyz_uv_velocity
    map<int, vector<pair<int, Eigen::Matrix<double, 10, 1>>>> featureFrame;
    vector<pair<cv::Point2f, cv::Point2f>> un_cur_lines = undistortedLineEndPoints(cur_lines, camera_[0]);
    for (size_t i = 0; i < cur_lines.size(); i++) {
        int feature_id = cur_lines[i][0].class_id;
        int camera_id = 0;
        Eigen::Matrix<double, 10, 1> xyz_uv;
        double x_st = un_cur_lines[i].first.x; double y_st = un_cur_lines[i].first.y; double z_st = 1;
        double x_ed = un_cur_lines[i].second.x; double y_ed = un_cur_lines[i].second.y; double z_ed = 1;
        double u_st = cur_lines[i][0].startPointX; double v_st = cur_lines[i][0].startPointY;
        double u_ed = cur_lines[i][0].endPointX; double v_ed = cur_lines[i][0].endPointY;

        xyz_uv << x_st, y_st, z_st, x_ed, y_ed, z_ed, u_st, v_st, u_ed, v_ed;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv);
    }

    prev_frame_->time = _cur_time;
    prev_frame_->img = cur_img;
    prev_frame_->klines = cur_lines;
    prev_frame_->descs = cur_descs;


    return featureFrame;
}

// void LineFeatureTracker::detectLines(const cv::Mat &image, vector<vector<KeyLine>> &detected_lines) {
//   eth::MultiOctaveSegmentDetector detector(std::make_shared<eth::EDLineDetector>());
//   detected_lines = detector.octaveKeyLines(image);
// }

void LineFeatureTracker::detectLines(const cv::Mat &image, vector<vector<KeyLine>> &detected_lines) {
    int num_lines;
    cv::Mat img_flt;
    image.convertTo(img_flt, CV_64FC1);
    double *imagePtr = reinterpret_cast<double *>(img_flt.data);
    double *out = lsd(&num_lines, imagePtr, img_flt.cols, img_flt.rows);
    for (int i = 0; i < num_lines; i++) {
        KeyLine kline;
        kline.class_id = 0;
        kline.octave = 0;
        kline.sPointInOctaveX = kline.startPointX = static_cast<float>(out[7 * i + 0]);
        kline.sPointInOctaveY = kline.startPointY = static_cast<float>(out[7 * i + 1]);
        kline.ePointInOctaveX = kline.endPointX = static_cast<float>(out[7 * i + 2]);
        kline.ePointInOctaveY = kline.endPointY = static_cast<float>(out[7 * i + 3]);
        cv::Point2f d_point = kline.getEndPoint() - kline.getStartPoint();
        kline.angle = std::atan2(d_point.y, d_point.x); // -pi ~ pi
        kline.lineLength = std::sqrt(d_point.dot(d_point));
        if (kline.lineLength < 16) {
            continue;
        }
        kline.pt = (kline.getStartPoint() + kline.getEndPoint()) / 2.;
        kline.response = kline.lineLength / std::max(image.cols, image.rows);
        kline.numOfPixels = std::max(std::abs(d_point.x), std::abs(d_point.y));
        detected_lines.push_back(vector<KeyLine>{kline});
    }

    // sort(detected_lines.begin(), detected_lines.end(), [](const vector<KeyLine>& kline1, const vector<KeyLine>& kline2){return kline1[0].lineLength > kline2[0].lineLength;});

    // if (detected_lines.size() > 100) {
    //     detected_lines.erase(detected_lines.begin() + 200, detected_lines.end());
    // }
    free((void *) out);
}

void LineFeatureTracker::describeLines(const cv::Mat &image, vector<vector<KeyLine>> &lines, std::vector<std::vector<cv::Mat>> &descs) {
    line_desc.compute(image, lines, descs, nullptr);
}

void LineFeatureTracker::filter_matches_by_angularVelocity(float dt, const std::vector<std::vector<KeyLine>> &prev_lines, const std::vector<std::vector<KeyLine>> &cur_lines, std::vector<std::pair<uint32_t, uint32_t>> &goodMatches) {
    std::vector<std::pair<uint32_t, uint32_t>> valid_matches;
    int count = 0;
    for (int i=0 ; i < goodMatches.size(); i++) {
        uint32_t prev_idx = goodMatches[i].first;
        uint32_t cur_idx = goodMatches[i].second;

        float diff = std::abs(cur_lines[cur_idx][0].angle - prev_lines[prev_idx][0].angle)*180./CV_PI; // 0 ~ 2pi
        float angular_velocity = std::min(diff, float(180.)-diff) / dt;   // deg/sec
        
        // std::cout << "angular_velocity: " << angular_velocity << " " << cur_lines[cur_idx][0].angle*180./CV_PI 
        // <<" " << prev_lines[prev_idx][0].angle*180./CV_PI << " " << diff << " " << float(180.)-diff  << std::endl;
        // std::cout << " ";
        if (angular_velocity < 200) {
            valid_matches.emplace_back(prev_idx, cur_idx);
        }
        else{
            // std::cout << "reject matches: " << count++ << " " << angular_velocity << std::endl;
        }
    }
    goodMatches = valid_matches;
}

void LineFeatureTracker::visualizeLines(cv::Mat &image, vector<KeyLine> &lines) {
    cv::Mat color;
    cv::cvtColor(image,color, cv::COLOR_GRAY2BGR);

    for (KeyLine line : lines) {
        cv::line(color,
                 cv::Point(line.startPointX, line.startPointY),
                 cv::Point(line.endPointX, line.endPointY), CV_RGB(0, 255, 0));
    }
    cv::imwrite("/home/sungho/plvinsfusion_ws/src/VINS-Fusion/test.jpg", color);
}

void LineFeatureTracker::visualizeLineMatches(const cv::Mat &cvLeftImage, const cv::Mat &cvRightImage,
                  const std::vector<std::vector<KeyLine>> &linesInLeft, const std::vector<std::vector<KeyLine>> &linesInRight,
                  const std::vector<std::pair<uint32_t, uint32_t>> &goodMatches) {
  cv::Point startPoint;
  cv::Point endPoint;

  cv::Mat cvLeftColorImage, cvRightColorImage;
  cv::cvtColor(cvLeftImage, cvLeftColorImage, cv::COLOR_GRAY2BGR);
  cv::cvtColor(cvRightImage, cvRightColorImage, cv::COLOR_GRAY2BGR);

  int w = cvLeftImage.cols, h = cvLeftImage.rows;
  int lowest = 100, highest = 255;
  int range = (highest - lowest) + 1;
  unsigned int r, g, b; //the color of lines
  for (auto &lines_vec : linesInLeft) {
    r = lowest + int(rand() % range);
    g = lowest + int(rand() % range);
    b = lowest + int(rand() % range);
    startPoint = cv::Point(int(lines_vec[0].startPointX), int(lines_vec[0].startPointY));
    endPoint = cv::Point(int(lines_vec[0].endPointX), int(lines_vec[0].endPointY));
    cv::line(cvLeftColorImage, startPoint, endPoint, CV_RGB(r, g, b));

    std::string lineId = std::to_string(lines_vec[0].class_id);
    cv::putText(cvLeftColorImage, lineId, (startPoint+endPoint)/2, cv::FONT_HERSHEY_DUPLEX,0.5,cv::Scalar(0,255,0),1,false);
  }
  cv::imshow("Left", cvLeftColorImage);

  for (auto &lines_vec : linesInRight) {
    r = lowest + int(rand() % range);
    g = lowest + int(rand() % range);
    b = lowest + int(rand() % range);
    startPoint = cv::Point(int(lines_vec[0].startPointX), int(lines_vec[0].startPointY));
    endPoint = cv::Point(int(lines_vec[0].endPointX), int(lines_vec[0].endPointY));
    cv::line(cvRightColorImage, startPoint, endPoint, CV_RGB(r, g, b));
    
    std::string lineId = std::to_string(lines_vec[0].class_id);
    cv::putText(cvRightColorImage, lineId, (startPoint+endPoint)/2, cv::FONT_HERSHEY_DUPLEX,0.5,cv::Scalar(0,255,0),1,false);
  }
  cv::imshow("Right", cvRightColorImage);

  ///////////####################################################################

  //store the matching results of the first and second images into a single image
  int lineIDLeft, lineIDRight;
  cv::cvtColor(cvLeftImage, cvLeftColorImage, cv::COLOR_GRAY2RGB);
  cv::cvtColor(cvRightImage, cvRightColorImage, cv::COLOR_GRAY2RGB);
  int lowest1 = 0, highest1 = 255;
  int range1 = (highest1 - lowest1) + 1;
  std::vector<unsigned int> r1(goodMatches.size()), g1(goodMatches.size()),
      b1(goodMatches.size()); //the color of lines
  for (unsigned int pair = 0; pair < goodMatches.size(); pair++) {
    r1[pair] = lowest1 + int(rand() % range1);
    g1[pair] = lowest1 + int(rand() % range1);
    b1[pair] = 255 - r1[pair];
    lineIDLeft = goodMatches[pair].first;
    lineIDRight = goodMatches[pair].second;
    startPoint.x = linesInLeft[lineIDLeft][0].startPointX;
    startPoint.y = linesInLeft[lineIDLeft][0].startPointY;
    endPoint.x = linesInLeft[lineIDLeft][0].endPointX;
    endPoint.y = linesInLeft[lineIDLeft][0].endPointY;
    cv::line(cvLeftColorImage,
             startPoint,
             endPoint,
             CV_RGB(r1[pair], g1[pair], b1[pair]),
             4,
             cv::LINE_AA);
    std::string lineIdLeft = std::to_string(linesInLeft[lineIDLeft][0].class_id);
    cv::putText(cvLeftColorImage, lineIdLeft, (startPoint+endPoint)/2, cv::FONT_HERSHEY_DUPLEX,0.5,cv::Scalar(0,255,0),1,false);

    startPoint.x = linesInRight[lineIDRight][0].startPointX;
    startPoint.y = linesInRight[lineIDRight][0].startPointY;
    endPoint.x = linesInRight[lineIDRight][0].endPointX;
    endPoint.y = linesInRight[lineIDRight][0].endPointY;
    cv::line(cvRightColorImage,
             startPoint,
             endPoint,
             CV_RGB(r1[pair], g1[pair], b1[pair]),
             4,
             cv::LINE_AA);
    std::string lineIdRight = std::to_string(linesInRight[lineIDRight][0].class_id);
    cv::putText(cvRightColorImage, lineIdRight, (startPoint+endPoint)/2, cv::FONT_HERSHEY_DUPLEX,0.5,cv::Scalar(0,255,0),1,false);
  }

  cv::Mat cvResultColorImage1(h, w * 2, CV_8UC3);
  cv::Mat cvResultColorImage2, cvResultColorImage;

  cv::Mat out1 = cvResultColorImage1(cv::Rect(0, 0, w, h));
  cvLeftColorImage.copyTo(out1);
  cv::Mat out2 = cvResultColorImage1(cv::Rect(w, 0, w, h));
  cvRightColorImage.copyTo(out2);

  cvResultColorImage2 = cvResultColorImage1.clone();
  for (unsigned int pair = 0; pair < goodMatches.size(); pair++) {
    lineIDLeft = goodMatches[pair].first;
    lineIDRight = goodMatches[pair].second;
    startPoint.x = linesInLeft[lineIDLeft][0].startPointX;
    startPoint.y = linesInLeft[lineIDLeft][0].startPointY;
    endPoint.x = linesInRight[lineIDRight][0].startPointX + w;
    endPoint.y = linesInRight[lineIDRight][0].startPointY;
    cv::line(cvResultColorImage2,
             startPoint,
             endPoint,
             CV_RGB(r1[pair], g1[pair], b1[pair]),
             2,
             cv::LINE_AA);
  }
  cv::addWeighted(cvResultColorImage1, 0.5, cvResultColorImage2, 0.5, 0.0, cvResultColorImage);

  std::cout << "number of total matches = " << goodMatches.size() << std::endl;
  cv::imshow("LBDSG", cvResultColorImage);
  cv::waitKey();
}

void LineFeatureTracker::drawTrack(const cv::Mat &cvLeftImage, const cv::Mat &cvRightImage,
                  const std::vector<std::vector<KeyLine>> &linesInLeft, const std::vector<std::vector<KeyLine>> &linesInRight,
                  const std::vector<std::pair<uint32_t, uint32_t>> &goodMatches) {
    cv::Point startPoint;
    cv::Point endPoint;

    cv::Mat cvLeftColorImage, cvRightColorImage;
    cv::cvtColor(cvLeftImage, cvLeftColorImage, cv::COLOR_GRAY2BGR);
    cv::cvtColor(cvRightImage, cvRightColorImage, cv::COLOR_GRAY2BGR);

    int w = cvLeftImage.cols, h = cvLeftImage.rows;
    int lowest = 100, highest = 255;
    int range = (highest - lowest) + 1;
    unsigned int r, g, b; //the color of lines
    // for (auto &lines_vec : linesInLeft) {
    //     startPoint = cv::Point(int(lines_vec[0].startPointX), int(lines_vec[0].startPointY));
    //     endPoint = cv::Point(int(lines_vec[0].endPointX), int(lines_vec[0].endPointY));
    //     // r = lowest + int(rand() % range);
    //     // g = lowest + int(rand() % range);
    //     // b = lowest + int(rand() % range);
    //     // cv::line(cvLeftColorImage, startPoint, endPoint, CV_RGB(r, g, b));
    //     cv::line(cvLeftColorImage, startPoint, endPoint, idToColor(lines_vec[0].class_id));

    //     std::string lineId = std::to_string(lines_vec[0].class_id);
    //     cv::putText(cvLeftColorImage, lineId, (startPoint+endPoint)/2, cv::FONT_HERSHEY_DUPLEX,0.5,cv::Scalar(0,255,0),1,false);
    // }
    // // cv::imshow("Left", cvLeftColorImage);

    // for (auto &lines_vec : linesInRight) {
    //     startPoint = cv::Point(int(lines_vec[0].startPointX), int(lines_vec[0].startPointY));
    //     endPoint = cv::Point(int(lines_vec[0].endPointX), int(lines_vec[0].endPointY));
    //     // r = lowest + int(rand() % range);
    //     // g = lowest + int(rand() % range);
    //     // b = lowest + int(rand() % range);        
    //     // cv::line(cvRightColorImage, startPoint, endPoint, CV_RGB(r, g, b));
    //     cv::line(cvRightColorImage, startPoint, endPoint, idToColor(lines_vec[0].class_id));
        
    //     std::string lineId = std::to_string(lines_vec[0].class_id);
    //     cv::putText(cvRightColorImage, lineId, (startPoint+endPoint)/2, cv::FONT_HERSHEY_DUPLEX,0.5,cv::Scalar(0,255,0),1,false);
    // }
    // // cv::imshow("Right", cvRightColorImage);

    ///////////####################################################################

    //store the matching results of the first and second images into a single image
    int lineIDLeft, lineIDRight;
    cv::cvtColor(cvLeftImage, cvLeftColorImage, cv::COLOR_GRAY2RGB);
    cv::cvtColor(cvRightImage, cvRightColorImage, cv::COLOR_GRAY2RGB);
    int lowest1 = 0, highest1 = 255;
    int range1 = (highest1 - lowest1) + 1;
    std::vector<unsigned int> r1(goodMatches.size()), g1(goodMatches.size()),
        b1(goodMatches.size()); //the color of lines
    for (unsigned int pair = 0; pair < goodMatches.size(); pair++) {
        r1[pair] = lowest1 + int(rand() % range1);
        g1[pair] = lowest1 + int(rand() % range1);
        b1[pair] = 255 - r1[pair];
        lineIDLeft = goodMatches[pair].first;
        lineIDRight = goodMatches[pair].second;
        startPoint.x = linesInLeft[lineIDLeft][0].startPointX;
        startPoint.y = linesInLeft[lineIDLeft][0].startPointY;
        endPoint.x = linesInLeft[lineIDLeft][0].endPointX;
        endPoint.y = linesInLeft[lineIDLeft][0].endPointY;
        // cv::line(cvLeftColorImage,
        //         startPoint,
        //         endPoint,
        //         CV_RGB(r1[pair], g1[pair], b1[pair]),
        //         4,
        //         cv::LINE_AA);
        cv::line(cvLeftColorImage,
            startPoint,
            endPoint,
            idToColor(linesInLeft[lineIDLeft][0].class_id),
            4,
            cv::LINE_AA);
        // std::string lineIdLeft = std::to_string(linesInLeft[lineIDLeft][0].class_id);
        // cv::putText(cvLeftColorImage, lineIdLeft, (startPoint+endPoint)/2, cv::FONT_HERSHEY_DUPLEX,0.5,cv::Scalar(0,255,0),1,false);

        startPoint.x = linesInRight[lineIDRight][0].startPointX;
        startPoint.y = linesInRight[lineIDRight][0].startPointY;
        endPoint.x = linesInRight[lineIDRight][0].endPointX;
        endPoint.y = linesInRight[lineIDRight][0].endPointY;
        // cv::line(cvRightColorImage,
        //         startPoint,
        //         endPoint,
        //         CV_RGB(r1[pair], g1[pair], b1[pair]),
        //         4,
        //         cv::LINE_AA);
        cv::line(cvRightColorImage,
            startPoint,
            endPoint,
            idToColor(linesInRight[lineIDRight][0].class_id),
            4,
            cv::LINE_AA);
        // std::string lineIdRight = std::to_string(linesInRight[lineIDRight][0].class_id);
        // cv::putText(cvRightColorImage, lineIdRight, (startPoint+endPoint)/2, cv::FONT_HERSHEY_DUPLEX,0.5,cv::Scalar(0,255,0),1,false);
    }

    cv::Mat cvResultColorImage1(h, w * 2, CV_8UC3);
    // cv::Mat cvResultColorImage2, cvResultColorImage;

    cv::Mat out1 = cvResultColorImage1(cv::Rect(0, 0, w, h));
    cvLeftColorImage.copyTo(out1);
    cv::Mat out2 = cvResultColorImage1(cv::Rect(w, 0, w, h));
    cvRightColorImage.copyTo(out2);

    // cvResultColorImage2 = cvResultColorImage1.clone();
    // for (unsigned int pair = 0; pair < goodMatches.size(); pair++) {
    //     lineIDLeft = goodMatches[pair].first;
    //     lineIDRight = goodMatches[pair].second;
    //     startPoint.x = linesInLeft[lineIDLeft][0].startPointX;
    //     startPoint.y = linesInLeft[lineIDLeft][0].startPointY;
    //     endPoint.x = linesInRight[lineIDRight][0].startPointX + w;
    //     endPoint.y = linesInRight[lineIDRight][0].startPointY;
    //     // cv::line(cvResultColorImage2,
    //     //         startPoint,
    //     //         endPoint,
    //     //         CV_RGB(r1[pair], g1[pair], b1[pair]),
    //     //         2,
    //     //         cv::LINE_AA);
    //     cv::line(cvResultColorImage2,
    //         startPoint,
    //         endPoint,
    //         idToColor(linesInLeft[lineIDLeft][0].class_id),
    //         2,
    //         cv::LINE_AA);
    // }
    // cv::addWeighted(cvResultColorImage1, 0.5, cvResultColorImage2, 0.5, 0.0, cvResultColorImage);

    image_track_ = cvResultColorImage1;
    // std::cout << "number of total matches = " << goodMatches.size() << std::endl;
    // cv::imshow("LBDSG", cvResultColorImage);
    // cv::waitKey();
}
cv::Mat LineFeatureTracker::getTrackImage() {
    return image_track_;
}