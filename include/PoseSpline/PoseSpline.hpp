#ifndef POSE_SPLINE_H_
#define POSE_SPLINE_H_

// Local
#include "PoseSpline/BSplineBase.hpp"
#include "PoseSpline/Pose.hpp"

// PoseSpline class
// ElementType = Pose<double>, spline order = 4
class PoseSpline: public BSplineBase<Pose<double>, 4> {
public:
    // constructor function
    PoseSpline();
    PoseSpline(double interval);
    virtual ~PoseSpline() {}
    // initialization
    void initialPoseSpline(std::vector<std::pair<double, Pose<double>>> Meas);
    // evaluation function
    Pose<double> evalPoseSpline(real_t t);
    Eigen::Vector3d evalLinearVelocity(real_t t);
    Eigen::Vector3d evalLinearAccelerator(real_t t, const Eigen::Vector3d& gravity);
    Eigen::Vector3d evalOmega(real_t t);
};

#endif // POSE_SPLINE_H_
