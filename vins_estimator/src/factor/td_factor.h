/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../estimator/parameters.h"

class TdFactor : public ceres::SizedCostFunction<1, 1>
{
  public:
    TdFactor(const double &_td, const double &_td_lstm);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    double td, td_lstm;
    // Eigen::Matrix<double, 2, 3> tangent_base;
    static double sqrt_info;
    static double sum_t;
};
