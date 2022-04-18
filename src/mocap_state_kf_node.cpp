//
// Created by hai on 4/17/22.
//

#include "state_estimator/mocap_state_kf.h"

int main(int argc, char **argv)
{
    // Set up ROS
    ros::init(argc, argv, "mocap_state_kalman_filter_node");     // node name
    ros::NodeHandle nh;                                         // create a node handle

    // Set parameters
    KF_Param kf_param;
    if (!nh.getParam(ros::this_node::getName()+"/pos_factor_to_meter", kf_param.pos_factor_to_meter))
    {
        ROS_ERROR_STREAM("mocap_state_kalman_filter_node Parameter " << ros::this_node::getName()+"/pos_factor_to_meter not set");
        return 0;
    }
    if (!nh.getParam(ros::this_node::getName()+"/Q_acc", kf_param.Q_acc))
    {
        ROS_ERROR_STREAM("mocap_state_kalman_filter_node Parameter " << ros::this_node::getName()+"/Q_acc not set");
        return 0;
    }
    if (!nh.getParam(ros::this_node::getName()+"/Q_rot_acc", kf_param.Q_rot_acc))
    {
        ROS_ERROR_STREAM("mocap_state_kalman_filter_node Parameter " << ros::this_node::getName()+"/Q_rot_acc not set");
        return 0;
    }
    if (!nh.getParam(ros::this_node::getName()+"/R_pos", kf_param.R_pos))
    {
        ROS_ERROR_STREAM("mocap_state_kalman_filter_node Parameter " << ros::this_node::getName()+"/R_pos not set");
        return 0;
    }
    if (!nh.getParam(ros::this_node::getName()+"/R_rpy", kf_param.R_rpy))
    {
        ROS_ERROR_STREAM("mocap_state_kalman_filter_node Parameter " << ros::this_node::getName()+"/R_rpy not set");
        return 0;
    }

    ROS_INFO("Parameter loaded successfully.");

    // Initialize a class object and pass node handle for constructor
    Mocap_State_KF mocap_state_kalman_filter(nh, kf_param);

    // not specify the publishing rate
    ros::spin();

    return 0;
}