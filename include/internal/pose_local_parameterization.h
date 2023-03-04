/*
 * @Author: whu-lyh yhaoli@whu.edu.cn
 * @Date: 2023-02-15 14:55:35
 * @LastEditors: whu-lyh yhaoli@whu.edu.cn
 * @LastEditTime: 2023-03-04 15:36:25
 * @FilePath: \PoseSpline\include\internal\pose_local_parameterization.h
 * @Description: PoseLocalParameterization for hamilton formulation
 */
/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

// Eigen
#include <eigen3/Eigen/Dense>
// ceres
#include <ceres/ceres.h>
// Local
#include "utility.h"

namespace hamilton {
    class PoseLocalParameterization : public ceres::LocalParameterization {
    public:
        virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;

        virtual bool ComputeJacobian(const double *x, double *jacobian) const;

        virtual int GlobalSize() const { return 7; };

        virtual int LocalSize() const { return 6; };
    };
}
