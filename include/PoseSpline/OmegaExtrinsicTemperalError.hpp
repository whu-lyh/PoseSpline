/*
 * @Author: whu-lyh yhaoli@whu.edu.cn
 * @Date: 2023-02-15 14:55:35
 * @LastEditors: whu-lyh yhaoli@whu.edu.cn
 * @LastEditTime: 2023-03-04 22:37:04
 * @FilePath: \PoseSpline\include\PoseSpline\OmegaExtrinsicTemperalError.hpp
 * @Description: OmegaExtrinsicTemperalError
 */
#ifndef OMEGAEXTRINSICTEMPERALERROR_H
#define OMEGAEXTRINSICTEMPERALERROR_H

// STL
#include <iostream>
// ceres
#include <ceres/ceres.h>
// Local
#include "PoseSpline/QuaternionSpline.hpp"
#include "PoseSpline/QuaternionLocalParameter.hpp"
#include "PoseSpline/ErrorInterface.hpp"

class OmegaExtrinsicTemperalError : public ceres::SizedCostFunction<3, 4, 1>
{
    OmegaExtrinsicTemperalError();
    OmegaExtrinsicTemperalError(const Eigen::Vector3d &omega_meas,
                                const Quaternion &Q_cw,
                                const Quaternion &dotQ_cw,
                                const Quaternion &dotdotQ_cw);

    virtual ~OmegaExtrinsicTemperalError();

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const;
    bool EvaluateWithMinimalJacobians(double const *const *parameters,
                                      double *residuals,
                                      double **jacobians,
                                      double **jacobiansMinimal) const;

private:
    Eigen::Vector3d omega_meas_;
    Quaternion Q_cw_;
    Quaternion dotQ_cw_;
    Quaternion dotdotQ_cw_;
};

#endif