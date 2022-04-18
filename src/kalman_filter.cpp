//
// Created by Hai Zhu on 4/17/22.
//

#include "state_estimator/kalman_filter.h"

Kalman_Filter::Kalman_Filter()
{
}

Kalman_Filter::~Kalman_Filter()
{
}

//! Initialize estimation
void Kalman_Filter::initialize_kf(Eigen::Matrix<double, 6, 1> xk, Eigen::Matrix<double, 6, 6> Pk)
{
    this->xk_est_ = xk;
    this->Pk_est_ = Pk;
}

//! KF update
void Kalman_Filter::kf_update(const double dt, const Eigen::Matrix<double, 3, 1> uk, const Eigen::Matrix<double, 3, 1> zk,
                              Eigen::Matrix<double, 6, 6> Qk, Eigen::Matrix<double, 3, 3> Rk)
{
    // system model
    Eigen::Matrix<double, 6, 6> Ak;
    Ak << 1, 0, 0, dt, 0, 0,
        0, 1, 0, 0, dt, 0,
        0, 0, 1, 0, 0, dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    Eigen::Matrix<double, 6, 3> Bk;
    double dt_2 = 0.5 * dt * dt;
    Bk << dt_2, 0, 0,
        0, dt_2, 0,
        0, 0, dt_2,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0;
    Eigen::Matrix<double, 3, 6> Hk;
    Hk << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0;

    // prediction 
    this->xk_est_ = Ak*this->xk_est_ + Bk*uk; 
    this->Pk_est_ = Ak*this->Pk_est_*Ak.transpose() + Qk; 

    // update
    Eigen::Matrix<double, 3, 1> zk_res = zk - Hk*this->xk_est_;
    Eigen::Matrix<double, 3, 3> Sk = Hk*this->Pk_est_*Hk.transpose() + Rk; 
    Eigen::Matrix<double, 6, 3> Kk = (this->Pk_est_*Hk.transpose()) * Sk.inverse();
    this->xk_est_ =  this->xk_est_ + Kk*zk_res;
    Eigen::Matrix<double, 6, 6> I;
    I.setIdentity();
    this->Pk_est_ = (I - Kk*Hk) * this->Pk_est_;
}
