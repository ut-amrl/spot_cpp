#include "helpers.h"

// todo: timestamps and time stuff (no easy way to get robot time from sdk yet)

tf2_msgs::TFMessage get_tf_from_state(bosdyn::api::RobotState state, const std::string inverse_target_frame) {
    tf2_msgs::TFMessage tf_msg;
    bosdyn::api::FrameTreeSnapshot snapshot = state.kinematic_state().transforms_snapshot();

    for (auto it = snapshot.child_to_parent_edge_map().begin(); it != snapshot.child_to_parent_edge_map().end(); it++) {
        if (!it->second.parent_frame_name().empty()) {
            bosdyn::api::FrameTreeSnapshot_ParentEdge transform = it->second;
            if (inverse_target_frame == it->first) {
                Math::SE3Pose geo_tform_inversed = Math::SE3Pose::from_proto(transform.parent_tform_child()).inverse();
                geometry_msgs::TransformStamped new_tf = populate_transform_stamped(it->first, transform.parent_frame_name(), geo_tform_inversed);
                tf_msg.transforms.push_back(new_tf);
            } else {
                geometry_msgs::TransformStamped new_tf = populate_transform_stamped(it->first, transform.parent_frame_name(), Math::SE3Pose::from_proto(transform.parent_tform_child()));
                tf_msg.transforms.push_back(new_tf);
            }
        }
    }

    return tf_msg;
}

nav_msgs::Odometry get_odom_from_state(bosdyn::api::RobotState state, bool use_vision) {
    // create utspot frametree object
    FrameTree tree(state.kinematic_state().transforms_snapshot());
    nav_msgs::Odometry ret;
    Math::SE3Pose transform = Math::SE3Pose::from_identity();

    if (use_vision) {
        ret.header.frame_id = "vision";
        transform = tree.a_tf_b(VISION_FRAME_NAME, BODY_FRAME_NAME);
    } else {
        ret.header.frame_id = "odom";
        transform = tree.a_tf_b(ODOM_FRAME_NAME, BODY_FRAME_NAME);
    }

    // fill out message
    // ret.header = kinematic_state.header;
	ret.child_frame_id = "base_link";
	ret.pose.pose.position.x = transform.x();
	ret.pose.pose.position.y = transform.y();
	ret.pose.pose.position.z = transform.z();

	ret.pose.pose.orientation.x = transform.quat().x();
	ret.pose.pose.orientation.y = transform.quat().y();
	ret.pose.pose.orientation.z = transform.quat().z();
	ret.pose.pose.orientation.w = transform.quat().w();
    return ret;
}

geometry_msgs::TwistWithCovarianceStamped get_odom_twist_from_state(bosdyn::api::RobotState state) {
    geometry_msgs::TwistWithCovarianceStamped ret;

    ret.twist.twist.linear.x = state.kinematic_state().velocity_of_body_in_odom().linear().x();
    ret.twist.twist.linear.y = state.kinematic_state().velocity_of_body_in_odom().linear().y();
    ret.twist.twist.linear.z = state.kinematic_state().velocity_of_body_in_odom().linear().z();
    ret.twist.twist.angular.x = state.kinematic_state().velocity_of_body_in_odom().angular().x();
    ret.twist.twist.angular.y = state.kinematic_state().velocity_of_body_in_odom().angular().y();
    ret.twist.twist.angular.z = state.kinematic_state().velocity_of_body_in_odom().angular().z();

    return ret;
}

geometry_msgs::TransformStamped populate_transform_stamped(const std::string child_frame, const std::string parent_frame, Math::SE3Pose transform) {
    geometry_msgs::TransformStamped new_tf;

    new_tf.header.frame_id = parent_frame;
    new_tf.child_frame_id = child_frame;
    new_tf.transform.translation.x = transform.x();
    new_tf.transform.translation.y = transform.y();
    new_tf.transform.translation.z = transform.z();
    new_tf.transform.rotation.x = transform.quat().x();
    new_tf.transform.rotation.y = transform.quat().y();
    new_tf.transform.rotation.z = transform.quat().z();
    new_tf.transform.rotation.w = transform.quat().w();

    return new_tf;
}

spot_msgs::FootStateArray get_foot_states_from_robotstate(bosdyn::api::RobotState state) {
    spot_msgs::FootStateArray ret;

    for (int i = 0; i < state.foot_state().size(); i++) {
        spot_msgs::FootState foot_msg;
        foot_msg.foot_position_rt_body.x = state.foot_state(i).foot_position_rt_body().x();
        foot_msg.foot_position_rt_body.y = state.foot_state(i).foot_position_rt_body().y();
        foot_msg.foot_position_rt_body.z = state.foot_state(i).foot_position_rt_body().z();
        foot_msg.contact = state.foot_state(i).contact();
        ret.states.push_back(foot_msg);
    }

    return ret;
}

spot_msgs::EStopStateArray get_estop_states_from_robotstate(bosdyn::api::RobotState state) {
    spot_msgs::EStopStateArray ret;

    for (int i = 0; i < state.estop_states().size(); i++) {
        spot_msgs::EStopState estop_msg;
        estop_msg.name = state.estop_states(i).name();
        estop_msg.type = state.estop_states(i).type();
        estop_msg.state = state.estop_states(i).state();
        ret.estop_states.push_back(estop_msg);
    }

    return ret;
}

spot_msgs::PowerState get_power_state_from_state(bosdyn::api::RobotState state) {
    spot_msgs::PowerState power_state_msg;

    power_state_msg.motor_power_state = state.power_state().motor_power_state();
    power_state_msg.shore_power_state = state.power_state().shore_power_state();
    power_state_msg.locomotion_charge_percentage = state.power_state().locomotion_charge_percentage().value();

    return power_state_msg;
}

spot_msgs::BatteryStateArray get_battery_states_from_state(bosdyn::api::RobotState state) {
    spot_msgs::BatteryStateArray ret;

    for (int i = 0; i < state.battery_states().size(); i++) {
        spot_msgs::BatteryState battery_msg;
        // local_time = spot_wrapper.robotToLocalTime(battery.timestamp)
        // battery_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)

        battery_msg.identifier = state.battery_states(i).identifier();
        battery_msg.charge_percentage = state.battery_states(i).charge_percentage().value();
        battery_msg.current = state.battery_states(i).current().value();
        battery_msg.voltage = state.battery_states(i).voltage().value();
        for (int i = 0; i < state.battery_states(i).temperatures().size(); i++) {
            battery_msg.temperatures.push_back(state.battery_states(i).temperatures(i));
        }
        battery_msg.status = state.battery_states(i).status();
        ret.battery_states.push_back(battery_msg);
    }

    return ret;
}

sensor_msgs::JointState get_joint_state_from_robotstate(bosdyn::api::RobotState state) {
    sensor_msgs::JointState joint_state;

    for (int i = 0; i < state.kinematic_state().joint_states().size(); i++) {
        // joint_state.name.push_back(friendly_joint_names.get(joint.name, "ERROR"))
        joint_state.position.push_back(state.kinematic_state().joint_states(i).position().value());
        joint_state.velocity.push_back(state.kinematic_state().joint_states(i).velocity().value());
        joint_state.effort.push_back(state.kinematic_state().joint_states(i).load().value());
    }

    return joint_state;
}

sensor_msgs::Image get_image_message(bosdyn::api::ImageResponse data) {
    sensor_msgs::Image image_msg;

    image_msg.height = data.shot().image().rows();
    image_msg.width = data.shot().image().cols();

    // jpeg format
    if (data.shot().image().format() == bosdyn::api::Image_Format_FORMAT_JPEG) {
        image_msg.encoding = "rgb8";
        image_msg.is_bigendian = true;
        image_msg.step = 3 * data.shot().image().cols();
    }

    // uncompressed format
    if (data.shot().image().format() == bosdyn::api::Image_Format_FORMAT_RAW) {
        // one byte per pixel
        if (data.shot().image().pixel_format() == bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_GREYSCALE_U8) {
            image_msg.encoding = "mono8";
            image_msg.is_bigendian = true;
            image_msg.step = data.shot().image().cols();
        }

        // three bytes per pixel
        if (data.shot().image().pixel_format() == bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_RGB_U8) {
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = true;
            image_msg.step = 3 * data.shot().image().cols();

        }

        // four bytes per pixel
        if (data.shot().image().pixel_format() == bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_RGBA_U8) {
            image_msg.encoding = "rgba8";
            image_msg.is_bigendian = true;
            image_msg.step = 4 * data.shot().image().cols();
        }

        // lendian zdistance
        if (data.shot().image().pixel_format() == bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_DEPTH_U16) {
            image_msg.encoding = "16UC1";
            image_msg.is_bigendian = false;
            image_msg.step = 2 * data.shot().image().cols();
        }
    }   

    // image_msg.data.push_back(data.shot().image().data());
    std::copy(data.shot().image().data().begin(), data.shot().image().data().end(), std::back_inserter(image_msg.data));

    return image_msg;
}

sensor_msgs::CameraInfo get_camerainfo_message(bosdyn::api::ImageResponse data) {
    sensor_msgs::CameraInfo camera_info_msg;
    
    camera_info_msg.header.frame_id = data.shot().frame_name_image_sensor();
    camera_info_msg.height = data.shot().image().rows();
    camera_info_msg.width = data.shot().image().cols();
    
    // fill out default camera info
    camera_info_msg.D.push_back(0);
    camera_info_msg.D.push_back(0);
    camera_info_msg.D.push_back(0);
    camera_info_msg.D.push_back(0);
    camera_info_msg.D.push_back(0);

    camera_info_msg.K[1] = 0;
    camera_info_msg.K[3] = 0;
    camera_info_msg.K[6] = 0;
    camera_info_msg.K[7] = 0;
    camera_info_msg.K[8] = 1;

    camera_info_msg.R[0] = 1;
    camera_info_msg.R[1] = 0;
    camera_info_msg.R[2] = 0;
    camera_info_msg.R[3] = 0;
    camera_info_msg.R[4] = 1;
    camera_info_msg.R[5] = 0;
    camera_info_msg.R[6] = 0;
    camera_info_msg.R[7] = 0;
    camera_info_msg.R[8] = 1;

    camera_info_msg.P[1] = 0;
    camera_info_msg.P[3] = 0;
    camera_info_msg.P[4] = 0;
    camera_info_msg.P[7] = 0;
    camera_info_msg.P[8] = 0;
    camera_info_msg.P[9] = 0;
    camera_info_msg.P[10] = 1;
    camera_info_msg.P[11] = 0;

    // fill data-specific info
    camera_info_msg.K[0] = data.source().pinhole().intrinsics().focal_length().x();
    camera_info_msg.K[2] = data.source().pinhole().intrinsics().principal_point().x();
    camera_info_msg.K[4] = data.source().pinhole().intrinsics().focal_length().y();
    camera_info_msg.K[5] = data.source().pinhole().intrinsics().principal_point().y();

    camera_info_msg.P[0] = data.source().pinhole().intrinsics().focal_length().x();
    camera_info_msg.P[2] = data.source().pinhole().intrinsics().principal_point().x();
    camera_info_msg.P[5] = data.source().pinhole().intrinsics().focal_length().y();
    camera_info_msg.P[6] = data.source().pinhole().intrinsics().principal_point().y();
    
    return camera_info_msg;
}