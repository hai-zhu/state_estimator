//
// Created by Hai Zhu on 4/17/22.
//

#ifndef STATE_ESTIMATOR_MOCAP_STATE_KF_H
#define STATE_ESTIMATOR_MOCAP_STATE_KF_H

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "state_estimator/kalman_filter.h"

struct KF_Param
{
    double pos_factor_to_meter;
    double Q_acc;
    double Q_rot_acc;
    double R_pos;
    double R_rpy;
};

class Mocap_State_KF
{
public:
    //! Constructor 
    Mocap_State_KF(ros::NodeHandle nh, KF_Param kf_param);
    ~Mocap_State_KF();

private:
    //! Ros node handle
    ros::NodeHandle     nh_;        // we will need this to pass between main and constructor

    //! Subscriber and publisher
    ros::Subscriber     mocap_pose_sub_;
    ros::Publisher      state_est_pub_;

    //! Measurement
    Eigen::Vector3d     pos_measured_;
    Eigen::Vector3d     rpy_measured_;      // rotation, ZYX

    //! Time info
    ros::Time           time_stamp_now_;
    ros::Time           time_stamp_previous_;
    double              dt_;
    ros::Time           time_stamp_print_;

    //! KF parameters
    KF_Param            kf_param_;
    

    //! State estimation
    Kalman_Filter       pos_vel_kf_;
    Kalman_Filter       rpy_rate_kf_;
    Eigen::Matrix<double, 6, 1>     pos_vel_est_; 
    Eigen::Matrix<double, 6, 6>     pos_vel_cov_est_;
    Eigen::Matrix<double, 6, 1>     rpy_rate_est_; 
    Eigen::Matrix<double, 6, 6>     rpy_rate_cov_est_;

    bool flag_first_pose_received_;
    bool flag_later_pose_received_;

    //! Initializations
    void initialize_subscribers();
    void initialize_publishers();

    //! Callback functions
    void callback_mocap_state_kf(const geometry_msgs::PoseStamped &msg);
};



#endif
