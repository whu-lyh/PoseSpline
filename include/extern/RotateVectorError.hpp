/*
 * @Author: whu-lyh yhaoli@whu.edu.cn
 * @Date: 2023-02-15 14:55:35
 * @LastEditors: whu-lyh yhaoli@whu.edu.cn
 * @LastEditTime: 2023-03-04 18:12:36
 * @FilePath: \PoseSpline\include\extern\RotateVectorError.hpp
 * @Description: RoatateVectorError
 */
#ifndef RORATEVECTORERROR_H
#define RORATEVECTORERROR_H

// STL
#include <iostream>
// ceres
#include <ceres/ceres.h>
#include "PoseSpline/QuaternionSpline.hpp"
#include "PoseSpline/QuaternionLocalParameter.hpp"
#include "PoseSpline/ErrorInterface.hpp"

class RoatateVectorError
    : public ceres::SizedCostFunction<3, 4, 4, 4, 4>
{
public:
    typedef Eigen::Matrix<double, 3, 3> covariance_t;
    typedef covariance_t information_t;

    RoatateVectorError(double t_meas, Eigen::Vector3d originalVector,
                       Eigen::Vector3d rotatedVector);
    virtual ~RoatateVectorError();

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const;
                          
    bool EvaluateWithMinimalJacobians(double const *const *parameters,
                                      double *residuals,
                                      double **jacobians,
                                      double **jacobiansMinimal) const;

private:
    double t_meas_;
    Eigen::Vector3d rotatedVector_Meas_;
    Eigen::Vector3d originalVector_;
    mutable information_t information_;           ///< The information matrix for this error term.
    mutable information_t squareRootInformation_; ///< The square root information matrix for this error term.
};

#endif