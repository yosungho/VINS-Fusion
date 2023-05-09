#include "linebase.h"
#include <cmath>

// namespace limap {

Line2d::Line2d(Eigen::Vector2d start_, Eigen::Vector2d end_, double score_) {
    start = start_; 
    end = end_;
    score = score_;
}

Line2d::Line2d(const Eigen::MatrixXd& seg) {
    // THROW_CHECK_EQ(seg.rows(), 2);
    // THROW_CHECK_EQ(seg.cols(), 2);
    start = Eigen::Vector2d(seg(0, 0), seg(0, 1));
    end = Eigen::Vector2d(seg(1, 0), seg(1, 1));
}

Eigen::Vector2d Line2d::point_projection(const Eigen::Vector2d& p) const {
    double projection = (p - start).dot(direction());
    if (projection < 0)
        return start;
    if (projection > length())
        return end;
    return start + projection * direction();
}

double Line2d::point_distance(const Eigen::Vector2d& p) const {
    Eigen::Vector2d p_proj = point_projection(p);
    double dist = (p - p_proj).norm();
    return dist;
}

// Eigen::Vector3d Line2d::coords() const {
//     Eigen::Vector3d start_homo = homogeneous(start);
//     Eigen::Vector3d end_homo = homogeneous(end);
//     return start_homo.cross(end_homo).normalized();
// }

Eigen::MatrixXd Line2d::as_array() const {
    Eigen::MatrixXd arr(2, 2);
    arr(0, 0) = start[0]; arr(0, 1) = start[1];
    arr(1, 0) = end[0]; arr(1, 1) = end[1];
    return arr;
}

Line3d::Line3d(Eigen::Vector3d start_, Eigen::Vector3d end_, double score_, double depth_start, double depth_end, double uncertainty_) {
    start = start_; 
    end = end_;
    score = score_;
    uncertainty = uncertainty_;
    depths[0] = depth_start; depths[1] = depth_end;
}

Line3d::Line3d(const Eigen::MatrixXd& seg) {
    // THROW_CHECK_EQ(seg.rows(), 2);
    // THROW_CHECK_EQ(seg.cols(), 3);
    start = Eigen::Vector3d(seg(0, 0), seg(0, 1), seg(0, 2));
    end = Eigen::Vector3d(seg(1, 0), seg(1, 1), seg(1, 2));
}

Eigen::Vector3d Line3d::point_projection(const Eigen::Vector3d& p) const {
    double projection = (p - start).dot(direction());
    if (projection < 0)
        return start;
    if (projection > length())
        return end;
    return start + projection * direction();
}

double Line3d::point_distance(const Eigen::Vector3d& p) const {
    Eigen::Vector3d p_proj = point_projection(p);
    double dist = (p - p_proj).norm();
    return dist;
}

Eigen::MatrixXd Line3d::as_array() const {
    Eigen::MatrixXd arr(2, 3);
    arr(0, 0) = start[0]; arr(0, 1) = start[1]; arr(0, 2) = start[2];
    arr(1, 0) = end[0]; arr(1, 1) = end[1]; arr(1, 2) = end[2];
    return arr;
}

// Line2d Line3d::projection(const CameraView& view) const {
//     Line2d line2d;
//     line2d.start = view.projection(start);
//     line2d.end = view.projection(end);
//     return line2d;
// }

// double Line3d::sensitivity(const CameraView& view) const {
//     Line2d line2d = projection(view);
//     Eigen::Vector3d dir3d = view.ray_direction(line2d.midpoint());
//     double cos_val = std::abs(direction().dot(dir3d));
//     double angle = acos(cos_val) * 180.0 / M_PI;
//     double sensitivity = 90 - angle;
//     return sensitivity;
// }

// double Line3d::computeUncertainty(const CameraView& view, const double var2d) const {
//     double d1 = view.pose.projdepth(start);
//     double d2 = view.pose.projdepth(end);
//     double d = (d1 + d2) / 2.0;
//     double uncertainty = view.cam.uncertainty(d, var2d);
//     return uncertainty;
// }

// Line2d projection_line3d(const Line3d& line3d, const CameraView& view) {
//     return line3d.projection(view);
// }

// Line3d unprojection_line2d(const Line2d& line2d, const CameraView& view, const std::pair<double, double>& depths) {
//     Line3d line3d;
//     Eigen::Vector3d start_homo = homogeneous(line2d.start);
//     line3d.start = view.R().transpose() * (view.K_inv() * start_homo * depths.first - view.T());
//     Eigen::Vector3d end_homo = homogeneous(line2d.end);
//     line3d.end = view.R().transpose() * (view.K_inv() * end_homo * depths.second - view.T());
//     return line3d;
// }

std::vector<Line2d> GetLine2dVectorFromArray(const Eigen::MatrixXd& segs2d) {
    // if (segs2d.rows() != 0)
    //     THROW_CHECK_GE(segs2d.cols(), 4);
    std::vector<Line2d> lines;
    for (int i = 0; i < segs2d.rows(); ++i)
        lines.push_back(Line2d(Eigen::Vector2d(segs2d(i, 0), segs2d(i, 1)), Eigen::Vector2d(segs2d(i, 2), segs2d(i, 3))));
    return lines;
}

std::vector<Line3d> GetLine3dVectorFromArray(const std::vector<Eigen::MatrixXd>& segs3d) {
    std::vector<Line3d> lines;
    for (size_t seg_id = 0; seg_id < segs3d.size(); ++seg_id) {
        const Eigen::MatrixXd& seg = segs3d[seg_id];
        lines.push_back(Line3d(seg));
    }
    return lines;
}

// } // namespace limap

