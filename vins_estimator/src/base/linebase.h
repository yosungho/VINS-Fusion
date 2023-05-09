#ifndef LINEBASE_H_
#define LINEBASE_H_

// #include <pybind11/pybind11.h>
// #include <pybind11/numpy.h>
// #include <pybind11/eigen.h>
#include <cmath>
#include <vector>
#include <set>
#include <eigen3/Eigen/Dense>

// namespace py = pybind11;
// #include "base/camera_view.h"

// namespace limap {

class Line2d {
public:
    Line2d() {}
    Line2d(const Eigen::MatrixXd& seg2d);
    Line2d(Eigen::Vector2d start, Eigen::Vector2d end, double score=-1);
    Eigen::Vector2d start, end;
    double score = -1;

    double length() const {return (start - end).norm();}
    Eigen::Vector2d midpoint() const {return 0.5 * (start + end);}
    Eigen::Vector2d direction() const {return (end - start).normalized();}
    Eigen::Vector2d perp_direction() const {Eigen::Vector2d dir = direction(); return Eigen::Vector2d(dir[1], -dir[0]); }
    // Eigen::Vector3d coords() const; // get homogeneous coordinate
    Eigen::Vector2d point_projection(const Eigen::Vector2d& p) const;
    double point_distance(const Eigen::Vector2d& p) const;
    Eigen::MatrixXd as_array() const;
};

class Line3d {
public:
    Line3d() {}
    Line3d(const Eigen::MatrixXd& seg3d);
    Line3d(Eigen::Vector3d start, Eigen::Vector3d end, double score=-1, double depth_start=-1, double depth_end=-1, double uncertainty=-1);
    Eigen::Vector3d start, end;
    double score = -1;
    double uncertainty = -1.0;
    Eigen::Vector2d depths; // [depth_start, depth_end] for the source perspective image

    void set_uncertainty(const double val) { uncertainty = val; }
    double length() const {return (start - end).norm();}
    Eigen::Vector3d midpoint() const {return 0.5 * (start + end);}
    Eigen::Vector3d direction() const {return (end - start).normalized();}
    Eigen::Vector3d point_projection(const Eigen::Vector3d& p) const;
    double point_distance(const Eigen::Vector3d& p) const;
    Eigen::MatrixXd as_array() const;
    // Line2d projection(const CameraView& view) const;
    // double sensitivity(const CameraView& view) const; // in angle, 0 for perfect view, 90 for collapsing
    // double computeUncertainty(const CameraView& view, const double var2d=5.0) const;
};

std::vector<Line2d> GetLine2dVectorFromArray(const Eigen::MatrixXd& segs2d);
std::vector<Line3d> GetLine3dVectorFromArray(const std::vector<Eigen::MatrixXd>& segs3d);

// Line2d projection_line3d(const Line3d& line3d, const CameraView& view);
// Line3d unprojection_line2d(const Line2d& line2d, const CameraView& view, const std::pair<double, double>& depths);

// } // namespace limap

#endif

