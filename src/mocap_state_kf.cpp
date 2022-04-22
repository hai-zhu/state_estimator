//
// Created by Hai Zhu on 4/17/22.
//

#include "state_estimator/mocap_state_kf.h"

// Constructor
Mocap_State_KF::Mocap_State_KF(ros::NodeHandle nh, KF_Param kf_param) : nh_(nh), kf_param_(kf_param)
{
    ROS_INFO("In class constructor of Mocap_State_KF");

    // Initialization subscriber and publisher
    this->initialize_publishers();
    this->initialize_subscribers();

    // Other initialization
    this->pos_measured_.setZero();
    this->rpy_measured_.setZero();
    this->pos_vel_est_.setZero();
    this->pos_vel_cov_est_.setZero();
    this->rpy_rate_est_.setZero();
    this->rpy_rate_cov_est_.setZero();
    this->time_stamp_print_ = ros::Time::now();
    this->time_stamp_previous_ = ros::Time::now();
    this->time_stamp_now_ = ros::Time::now();
    this->dt_ = 1.0 / 60.0;

    this->flag_first_pose_received_ = false;
    this->flag_later_pose_received_ = false;

    // Other info by default
    this->kf_param_.pos_factor_to_meter = 0.001;
    this->kf_param_.Q_acc = 10.0;
    this->kf_param_.Q_rot_acc = 10.0;
    this->kf_param_.R_pos = 0.001;
    this->kf_param_.R_rpy = 0.001;
}

Mocap_State_KF::~Mocap_State_KF()
{
}

// Initialize publishers
void Mocap_State_KF::initialize_publishers()
{
    ROS_INFO("Initializing publishers");
    this->state_est_pub_ = this->nh_.advertise<nav_msgs::Odometry>("/mocap_object/state_odom_estimation", 1, true);
}

// Initialize subscribers
void Mocap_State_KF::initialize_subscribers()
{
    ROS_INFO("Initializing subscribers");
    this->mocap_pose_sub_ = this->nh_.subscribe("/mocap_object/pose", 1, &Mocap_State_KF::callback_mocap_state_kf, this);
    // this->mocap_pose_sub_ = this->nh_.subscribe("/vrpn_client_node/Tracker0/pose", 1, &Mocap_State_KF::callback_mocap_state_kf, this);
}

// Callback
void Mocap_State_KF::callback_mocap_state_kf(const geometry_msgs::PoseStamped &msg)
{
    // set flag
    if (this->flag_first_pose_received_ == false)
    {
        this->flag_first_pose_received_ = true;
        ROS_INFO("First measured pose received!");
    }
    else if (this->flag_later_pose_received_ == false)
    {
        this->flag_later_pose_received_ = true;
        ROS_INFO("Second measured pose received! Will start KF!");
    }

    // dt
    this->time_stamp_now_ = msg.header.stamp;
    this->dt_ = (this->time_stamp_now_ - this->time_stamp_previous_).toSec();

    // measurements
    this->pos_measured_[0] = this->kf_param_.pos_factor_to_meter * msg.pose.position.x;
    this->pos_measured_[1] = this->kf_param_.pos_factor_to_meter * msg.pose.position.y;
    this->pos_measured_[2] = this->kf_param_.pos_factor_to_meter * msg.pose.position.z;
    tf::Quaternion quat_measured;
    tf::quaternionMsgToTF(msg.pose.orientation, quat_measured);
    tf::Matrix3x3(quat_measured).getRPY(this->rpy_measured_[0], this->rpy_measured_[1], this->rpy_measured_[2]); // XYZ
    if (this->flag_first_pose_received_ == true && this->flag_later_pose_received_ == false)
    {
        // initialize pos vel kf
        Eigen::Matrix<double, 6, 1> xk;
        xk << pos_measured_[0], pos_measured_[1], pos_measured_[2], 0.0, 0.0, 0.0;
        double P_pos = 0.01;
        double P_vel = 1.0;
        Eigen::Matrix<double, 6, 1> P_vec;
        P_vec << P_pos, P_pos, P_pos, P_vel, P_vel, P_vel;
        Eigen::Matrix<double, 6, 6> Pk = P_vec.asDiagonal();
        this->pos_vel_kf_.initialize_kf(xk, Pk);
        // initialize rpy rate kf
        xk << rpy_measured_[0], rpy_measured_[1], rpy_measured_[2], 0.0, 0.0, 0.0;
        double P_rpy = 0.01;
        double P_rate = 1.0;
        P_vec << P_rpy, P_rpy, P_rpy, P_rate, P_rate, P_rate;
        Pk = P_vec.asDiagonal();
        this->rpy_rate_kf_.initialize_kf(xk, Pk);
    }

    // filtering
    if (this->flag_later_pose_received_ == true)
    {
        Eigen::Matrix<double, 3, 1> uk;
        Eigen::Matrix<double, 3, 1> zk;
        Eigen::Matrix<double, 6, 1> Q_vec;
        Eigen::Matrix<double, 6, 6> Qk;
        Eigen::Matrix<double, 3, 1> R_vec;
        Eigen::Matrix<double, 3, 3> Rk;
        // pos vel kf
        uk << 0.0, 0.0, 0.0;
        zk = this->pos_measured_;
        double Q_acc = this->kf_param_.Q_acc;
        double Q_pos = 0.5 * Q_acc * this->dt_ * this->dt_;
        double Q_vel = Q_acc * this->dt_;
        Q_vec << Q_pos * Q_pos, Q_pos * Q_pos, Q_pos * Q_pos, Q_vel * Q_vel, Q_vel * Q_vel, Q_vel * Q_vel;
        Qk = Q_vec.asDiagonal();
        double R_pos = this->kf_param_.R_pos;
        R_vec << R_pos * R_pos, R_pos * R_pos, R_pos * R_pos;
        Rk = R_vec.asDiagonal();
        this->pos_vel_kf_.kf_update(this->dt_, uk, zk, Qk, Rk);
        this->pos_vel_est_ = pos_vel_kf_.xk_est_;
        this->pos_vel_cov_est_ = pos_vel_kf_.Pk_est_;
        // rpy rate kf
        uk << 0.0, 0.0, 0.0;
        zk = this->rpy_measured_;
        double Q_rot_acc = this->kf_param_.Q_rot_acc;
        double Q_rpy = 0.5 * Q_rot_acc * this->dt_ * this->dt_;
        double Q_rate = Q_rot_acc * this->dt_;
        Q_vec << Q_rpy * Q_rpy, Q_rpy * Q_rpy, Q_rpy * Q_rpy, Q_rate * Q_rate, Q_rate * Q_rate, Q_rate * Q_rate;
        Qk = Q_vec.asDiagonal();
        double R_rpy = this->kf_param_.R_rpy;
        R_vec << R_rpy * R_rpy, R_rpy * R_rpy, R_rpy * R_rpy;
        Rk = R_vec.asDiagonal();
        this->rpy_rate_kf_.kf_update(this->dt_, uk, zk, Qk, Rk);
        this->rpy_rate_est_ = rpy_rate_kf_.xk_est_;
        this->rpy_rate_cov_est_ = rpy_rate_kf_.Pk_est_;

        // publish estimation results
        nav_msgs::Odometry msg_pub;
        msg_pub.header = msg.header;
        // pose
        msg_pub.pose.pose.position.x = this->pos_vel_est_[0];
        msg_pub.pose.pose.position.y = this->pos_vel_est_[1];
        msg_pub.pose.pose.position.z = this->pos_vel_est_[2];
        msg_pub.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
            this->rpy_rate_est_[0], this->rpy_rate_est_[1], this->rpy_rate_est_[2]);
        // twist
        msg_pub.twist.twist.linear.x = this->pos_vel_est_[3];
        msg_pub.twist.twist.linear.y = this->pos_vel_est_[4];
        msg_pub.twist.twist.linear.z = this->pos_vel_est_[5];
        msg_pub.twist.twist.angular.x = this->rpy_rate_est_[3];
        msg_pub.twist.twist.angular.y = this->rpy_rate_est_[4];
        msg_pub.twist.twist.angular.z = this->rpy_rate_est_[5];
        // covariance
        msg_pub.pose.covariance[0] = this->pos_vel_cov_est_(0, 0);
        msg_pub.pose.covariance[1] = this->pos_vel_cov_est_(1, 1);
        msg_pub.pose.covariance[2] = this->pos_vel_cov_est_(2, 2);
        msg_pub.pose.covariance[3] = this->rpy_rate_cov_est_(0, 0);
        msg_pub.pose.covariance[4] = this->rpy_rate_cov_est_(1, 1);
        msg_pub.pose.covariance[5] = this->rpy_rate_cov_est_(2, 2);
        msg_pub.twist.covariance[0] = this->pos_vel_cov_est_(3, 3);
        msg_pub.twist.covariance[1] = this->pos_vel_cov_est_(4, 4);
        msg_pub.twist.covariance[2] = this->pos_vel_cov_est_(5, 5);
        msg_pub.twist.covariance[3] = this->rpy_rate_cov_est_(3, 3);
        msg_pub.twist.covariance[4] = this->rpy_rate_cov_est_(4, 4);
        msg_pub.twist.covariance[5] = this->rpy_rate_cov_est_(5, 5);

        this->state_est_pub_.publish(msg_pub);
        this->time_stamp_previous_ = this->time_stamp_now_;

        // print
        if ((this->time_stamp_now_ - this->time_stamp_print_).toSec() >= 1.0)
        {
            ROS_INFO("Mocap KF is running...");
            this->time_stamp_print_ = this->time_stamp_now_;
        }
    }
}