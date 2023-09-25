/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "td_factor.h"

double TdFactor::sqrt_info;
double TdFactor::sum_t;

TdFactor::TdFactor(const double &_td, const double &_td_lstm) : td(_td) , td_lstm(_td_lstm)
{
// #ifdef UNIT_SPHERE_ERROR
//     Eigen::Vector3d b1, b2;
//     Eigen::Vector3d a = pts_j.normalized();
//     Eigen::Vector3d tmp(0, 0, 1);
//     if(a == tmp)
//         tmp << 1, 0, 0;
//     b1 = (tmp - a * (a.transpose() * tmp)).normalized();
//     b2 = a.cross(b1);
//     tangent_base.block<1, 3>(0, 0) = b1.transpose();
//     tangent_base.block<1, 3>(1, 0) = b2.transpose();
// #endif
    sqrt_info = 5.0e5; //important
};

bool TdFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;

    double* residual(residuals);
    double cur_td = parameters[0][0];

    *residual = cur_td - td_lstm;
    *residual = sqrt_info * (*residual);

    if (jacobians)
    {
        if (jacobians[0])
        {
            *jacobians[0] = 1;
        }
    }
    sum_t += tic_toc.toc();

    return true;
}

// void TdFactor::check(double **parameters)
// {
//     double *res = new double[15];
//     double **jaco = new double *[4];
//     jaco[0] = new double[2 * 7];
//     jaco[1] = new double[2 * 7];
//     jaco[2] = new double[2 * 7];
//     jaco[3] = new double[2 * 1];
//     Evaluate(parameters, res, jaco);
//     puts("check begins");

//     puts("my");

//     std::cout << Eigen::Map<Eigen::Matrix<double, 2, 1>>(res).transpose() << std::endl
//               << std::endl;
//     std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[0]) << std::endl
//               << std::endl;
//     std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[1]) << std::endl
//               << std::endl;
//     std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[2]) << std::endl
//               << std::endl;
//     std::cout << Eigen::Map<Eigen::Vector2d>(jaco[3]) << std::endl
//               << std::endl;

//     Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
//     Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

//     Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
//     Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

//     Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
//     Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
//     double inv_dep_i = parameters[3][0];

//     Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
//     Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
//     Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
//     Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
//     Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);


//     Eigen::Vector2d residual;
// #ifdef UNIT_SPHERE_ERROR 
//     residual =  tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
// #else
//     double dep_j = pts_camera_j.z();
//     residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
// #endif
//     residual = sqrt_info * residual;

//     puts("num");
//     std::cout << residual.transpose() << std::endl;

//     const double eps = 1e-6;
//     Eigen::Matrix<double, 2, 19> num_jacobian;
//     for (int k = 0; k < 19; k++)
//     {
//         Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
//         Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

//         Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
//         Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

//         Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
//         Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
//         double inv_dep_i = parameters[3][0];

//         int a = k / 3, b = k % 3;
//         Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

//         if (a == 0)
//             Pi += delta;
//         else if (a == 1)
//             Qi = Qi * Utility::deltaQ(delta);
//         else if (a == 2)
//             Pj += delta;
//         else if (a == 3)
//             Qj = Qj * Utility::deltaQ(delta);
//         else if (a == 4)
//             tic += delta;
//         else if (a == 5)
//             qic = qic * Utility::deltaQ(delta);
//         else if (a == 6)
//             inv_dep_i += delta.x();

//         Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
//         Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
//         Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
//         Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
//         Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

//         Eigen::Vector2d tmp_residual;
// #ifdef UNIT_SPHERE_ERROR 
//         tmp_residual =  tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
// #else
//         double dep_j = pts_camera_j.z();
//         tmp_residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
// #endif
//         tmp_residual = sqrt_info * tmp_residual;
//         num_jacobian.col(k) = (tmp_residual - residual) / eps;
//     }
//     std::cout << num_jacobian << std::endl;
// }
