//
// Created by Hai Zhu on 4/17/22.
//

#ifndef STATE_ESTIMATOR_KALMAN_FILTER_H
#define STATE_ESTIMATOR_KALMAN_FILTER_H

#include <Eigen/Dense>

class Kalman_Filter
{
public:
    Kalman_Filter();
    ~Kalman_Filter();
    void initialize_kf(const Eigen::Matrix<double, 6, 1> xk, const Eigen::Matrix<double, 6, 6> Pk);
    void kf_update(const double dt, const Eigen::Matrix<double, 3, 1> uk, const Eigen::Matrix<double, 3, 1> zk,
                   Eigen::Matrix<double, 6, 6> Qk, Eigen::Matrix<double, 3, 3> Rk);

    Eigen::Matrix<double, 6, 1> xk_est_;
    Eigen::Matrix<double, 6, 6> Pk_est_;
};

#endif