#ifndef HELPERS_H
#define HELPERS_H

#include "ros/ros.h"
#include "spot/spot.h"

/* non clearpath */
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/JointState.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/static_transform_broadcaster.h"

/* clearpath */ 
#include "spot_msgs/Metrics.h"
#include "spot_msgs/LeaseArray.h"
#include "spot_msgs/LeaseResource.h"
#include "spot_msgs/FootState.h"
#include "spot_msgs/FootStateArray.h"
#include "spot_msgs/EStopState.h"
#include "spot_msgs/EStopStateArray.h"
#include "spot_msgs/WiFiState.h"
#include "spot_msgs/PowerState.h"
#include "spot_msgs/BehaviorFault.h"
#include "spot_msgs/BehaviorFaultState.h"
#include "spot_msgs/SystemFault.h"
#include "spot_msgs/SystemFaultState.h"
#include "spot_msgs/BatteryState.h"
#include "spot_msgs/BatteryStateArray.h"
#include "spot_msgs/Feedback.h"
#include "spot_msgs/MobilityParams.h"
#include "spot_msgs/ListGraph.h"
#include "spot_msgs/ClearBehaviorFault.h"

/* interface for storing function pointers (used in callbacks for translation layer) */

tf2_msgs::TFMessage get_tf_from_state(bosdyn::api::RobotState state, const std::string inverse_target_frame);
nav_msgs::Odometry get_odom_from_state(bosdyn::api::RobotState state, bool use_vision = true);
geometry_msgs::TwistWithCovarianceStamped get_odom_twist_from_state(bosdyn::api::RobotState state);
geometry_msgs::TransformStamped populate_transform_stamped(const std::string child_frame, const std::string parent_frame, Math::SE3Pose transform);
spot_msgs::FootStateArray get_foot_states_from_robotstate(bosdyn::api::RobotState state);
spot_msgs::EStopStateArray get_estop_states_from_robotstate(bosdyn::api::RobotState state);
spot_msgs::PowerState get_power_state_from_state(bosdyn::api::RobotState state);
spot_msgs::BatteryStateArray get_battery_states_from_state(bosdyn::api::RobotState state);
sensor_msgs::JointState get_joint_state_from_robotstate(bosdyn::api::RobotState state);
sensor_msgs::Image get_image_message(bosdyn::api::ImageResponse data);
sensor_msgs::CameraInfo get_camerainfo_message(bosdyn::api::ImageResponse data);

#endif