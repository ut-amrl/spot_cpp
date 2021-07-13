#include "spot_ros_cpp.h"

static const int MS_MULTIPLIER = 1000;

/* FREQUENCIES: how many times a second we should be updating the member */
static std::map<std::string, double> FREQUENCIES {
    {"robot_state", 20.0},
    {"metrics", 0.04},
    //{"lease", 1.0},
    {"front_image", 10.0},
    {"side_image", 10.0},
    {"rear_image", 10.0}
};

static const std::chrono::milliseconds robot_state_rate((int) (MS_MULTIPLIER / FREQUENCIES["robot_state"]));
static const std::chrono::milliseconds metrics_rate((int) (MS_MULTIPLIER / FREQUENCIES["metrics"]));
static const std::chrono::milliseconds front_image_rate((int) (MS_MULTIPLIER / FREQUENCIES["front_image"]));
static const std::chrono::milliseconds side_image_rate((int) (MS_MULTIPLIER / FREQUENCIES["side_image"]));
static const std::chrono::milliseconds rear_image_rate((int) (MS_MULTIPLIER / FREQUENCIES["rear_image"]));

static std::map<std::string, std::chrono::milliseconds> RATES {
    {"robot_state", robot_state_rate},
    {"metrics", metrics_rate},
    {"front_image", front_image_rate},
    {"side_image", side_image_rate},
    {"rear_image", rear_image_rate}
};

SpotROSNode::SpotROSNode() : _spot() {}

bool SpotROSNode::update_ready(const std::string &member) {
    return _last_updated.find(member) == _last_updated.end() || std::chrono::steady_clock::now() - _last_updated[member] > RATES[member];
}

void SpotROSNode::cmd_vel_callback(const geometry_msgs::Twist &msg) {
    // 600 ms
	int vel_cmd_duration = 600;
	std::chrono::seconds max_marching_time(5);
	bool all_zero = (msg.linear.x == 0) && (msg.linear.y == 0) && (msg.linear.z == 0);
	if (all_zero) {
		// vel command still executing
		if (_t_last_non_zero_vel_cmd < std::chrono::steady_clock::now() - max_marching_time) {
			return;
		}
	} else {
		// set time of last nonzero command
		_t_last_non_zero_vel_cmd = std::chrono::steady_clock::now();
		_spot.velocityMove(msg.linear.x, msg.linear.y, msg.linear.z, vel_cmd_duration, FLAT_BODY);
	}
}

/* todo: messages and case handling for services */
bool SpotROSNode::sit_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    _spot.sit();
    return true;
}

bool SpotROSNode::stand_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    _spot.stand();
    return true;
}

bool SpotROSNode::power_on_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    _spot.powerOnBlocking();
    return true;
}

bool SpotROSNode::power_off_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    _spot.powerOffBlocking();
    return true;
}

void SpotROSNode::run(int argc, char **argv, const std::string &username, const std::string &password) {
    ros::init(argc, argv, "spot_cpp_node");
    ros::NodeHandle nh;
    ros::Rate rate(50);

    /* Publishers */
    // image (handled by image)
    ros::Publisher image_back_pub = nh.advertise<sensor_msgs::Image>("camera/back/image", 10);
    ros::Publisher image_frontleft_pub = nh.advertise<sensor_msgs::Image>("camera/frontleft/image", 10);
    ros::Publisher image_frontright_pub = nh.advertise<sensor_msgs::Image>("camera/frontright/image", 10);
    ros::Publisher image_left_pub = nh.advertise<sensor_msgs::Image>("camera/left/image", 10);
    ros::Publisher image_right_pub = nh.advertise<sensor_msgs::Image>("camera/right/image", 10);

    // depth
    ros::Publisher depth_back_pub = nh.advertise<sensor_msgs::Image>("depth/back/image", 10);
    ros::Publisher depth_frontleft_pub = nh.advertise<sensor_msgs::Image>("depth/frontleft/image", 10);
    ros::Publisher depth_frontright_pub = nh.advertise<sensor_msgs::Image>("depth/frontright/image", 10);
    ros::Publisher depth_left_pub = nh.advertise<sensor_msgs::Image>("depth/left/image", 10);
    ros::Publisher depth_right_pub = nh.advertise<sensor_msgs::Image>("depth/right/image", 10);

    // image camera info
    ros::Publisher image_back_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera/back/camera_info", 10);
    ros::Publisher image_frontleft_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera/frontleft/camera_info", 10);
    ros::Publisher image_frontright_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera/frontright/camera_info", 10);
    ros::Publisher image_left_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera/left/camera_info", 10);
    ros::Publisher image_right_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera/right/camera_info", 10);

    // depth camera info
    ros::Publisher depth_back_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("depth/back/camera_info", 10);
    ros::Publisher depth_frontleft_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("depth/frontleft/camera_info", 10);
    ros::Publisher depth_frontright_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("depth/frontright/camera_info", 10);
    ros::Publisher depth_left_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("depth/left/camera_info", 10);
    ros::Publisher depth_right_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("depth/right/camera_info", 10);

    // status (handled by robotstate)
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Publisher metrics_pub = nh.advertise<spot_msgs::Metrics>("status/metrics", 10);
    ros::Publisher leases_pub = nh.advertise<spot_msgs::LeaseArray>("status/leases", 10);
    ros::Publisher odometry_twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("odometry/twist", 10);
    ros::Publisher odometry_pub = nh.advertise<nav_msgs::Odometry>("odometry", 10);
    ros::Publisher feet_pub = nh.advertise<spot_msgs::FootStateArray>("status/feet", 10);
    ros::Publisher estop_pub = nh.advertise<spot_msgs::EStopStateArray>("status/estop", 10);
    ros::Publisher power_pub = nh.advertise<spot_msgs::PowerState>("status/power_state", 10);
    ros::Publisher battery_pub = nh.advertise<spot_msgs::BatteryStateArray>("status/battery_states", 10);

    // feedback (handled by robot id, sdk)
    ros::Publisher feedback_pub = nh.advertise<spot_msgs::Feedback>("status/feedback", 10);

    // mobility params (handled by sdk)
    ros::Publisher mobility_params_pub = nh.advertise<spot_msgs::MobilityParams>("status/mobility_params", 10);

    /* Subscribers TODO: define callbacks*/
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, &SpotROSNode::cmd_vel_callback, this);
    // ros::Subscriber body_pose_sub = nh.subscribe("body_pose");

    /* Services TODO: callbacks */
    ros::ServiceServer sit_srv = nh.advertiseService("sit", &SpotROSNode::sit_srv, this);
    ros::ServiceServer stand_srv = nh.advertiseService("stand", &SpotROSNode::stand_srv, this);
    ros::ServiceServer power_on_srv = nh.advertiseService("power_on", &SpotROSNode::power_on_srv, this);
    ros::ServiceServer power_off_srv = nh.advertiseService("power_off", &SpotROSNode::power_off_srv, this);
    // ros::ServiceServer stair_mode_srv = nh.advertiseService("stair_mode");
    // ros::ServiceServer locomotion_mode_srv = nh.advertiseService("locomotion_mode");

    _spot.basicInit(username, password);
    Trajectory3D trajPose;
	trajPose.addPointRPY(0, 0, 0, 0, 0, 0, 1);
    _spot.setBodyPose(trajPose, true);

    while (ros::ok()) {        
        // update topics and send messages (wrapper will handle timing)
        if (update_ready("front_image")) {
            bosdyn::api::ImageResponse image_frontright = _spot.getSpotState()->image("frontright_fisheye_image", 100, bosdyn::api::Image_Format_FORMAT_RAW);
            bosdyn::api::ImageResponse depth_frontright = _spot.getSpotState()->image("frontright_depth", 100, bosdyn::api::Image_Format_FORMAT_RAW);
            bosdyn::api::ImageResponse image_frontleft = _spot.getSpotState()->image("frontleft_fisheye_image", 100, bosdyn::api::Image_Format_FORMAT_RAW);
            bosdyn::api::ImageResponse depth_frontleft = _spot.getSpotState()->image("frontleft_depth", 100, bosdyn::api::Image_Format_FORMAT_RAW);

            image_frontleft_pub.publish(get_image_message(image_frontleft));
            image_frontright_pub.publish(get_image_message(image_frontright));
            depth_frontleft_pub.publish(get_image_message(depth_frontleft));
            depth_frontright_pub.publish(get_image_message(depth_frontright));

            image_frontleft_camera_info_pub.publish(get_camerainfo_message(image_frontleft));
            image_frontright_camera_info_pub.publish(get_camerainfo_message(image_frontright));
            depth_frontleft_camera_info_pub.publish(get_camerainfo_message(depth_frontleft));
            depth_frontright_camera_info_pub.publish(get_camerainfo_message(depth_frontright));
            _last_updated["front_image"] = std::chrono::steady_clock::now();
        }

        if (update_ready("side_image")) {
            bosdyn::api::ImageResponse image_right = _spot.getSpotState()->image("right_fisheye_image", 100, bosdyn::api::Image_Format_FORMAT_RAW);
            bosdyn::api::ImageResponse depth_right = _spot.getSpotState()->image("right_depth", 100, bosdyn::api::Image_Format_FORMAT_RAW);
            bosdyn::api::ImageResponse image_left = _spot.getSpotState()->image("left_fisheye_image", 100, bosdyn::api::Image_Format_FORMAT_RAW);
            bosdyn::api::ImageResponse depth_left = _spot.getSpotState()->image("left_depth", 100, bosdyn::api::Image_Format_FORMAT_RAW);

            image_left_pub.publish(get_image_message(image_left));
            image_right_pub.publish(get_image_message(image_right));
            depth_left_pub.publish(get_image_message(depth_left));
            depth_right_pub.publish(get_image_message(depth_right));

            image_left_camera_info_pub.publish(get_camerainfo_message(image_left));
            image_right_camera_info_pub.publish(get_camerainfo_message(image_right));
            depth_left_camera_info_pub.publish(get_camerainfo_message(depth_left));
            depth_right_camera_info_pub.publish(get_camerainfo_message(depth_right));
            _last_updated["side_image"] = std::chrono::steady_clock::now();
        }

        if (update_ready("rear_image")) {
            bosdyn::api::ImageResponse image_rear = _spot.getSpotState()->image("back_fisheye_image", 100, bosdyn::api::Image_Format_FORMAT_RAW);
            bosdyn::api::ImageResponse depth_rear = _spot.getSpotState()->image("back_depth", 100, bosdyn::api::Image_Format_FORMAT_RAW);

            image_back_pub.publish(get_image_message(image_rear));
            depth_back_pub.publish(get_image_message(depth_rear));
            image_back_camera_info_pub.publish(get_camerainfo_message(image_rear));
            depth_back_camera_info_pub.publish(get_camerainfo_message(depth_rear));
            _last_updated["rear_image"] = std::chrono::steady_clock::now();
        }

        if (update_ready("metrics")) {
            bosdyn::api::RobotMetrics metrics = _spot.getSpotState()->robotMetrics();
            spot_msgs::Metrics metrics_msg;

            // fill out metrics message (distance, gait cycles, time moving, electric power)
            for (int i = 0; i < metrics.metrics().size(); i++) {
                bosdyn::api::Parameter param = metrics.metrics(i);
                if (param.label() == "distance") {
                    metrics_msg.distance = param.float_value();
                }

                if (param.label() == "gait_cycles") {
                    metrics_msg.gait_cycles = param.int_value();
                }

                if (param.label() == "time_moving") {
                    ros::Duration dur(google::protobuf::util::TimeUtil::DurationToSeconds(param.duration()));
                    metrics_msg.time_moving = dur;
                }

                if (param.label() == "electric_power") {
                    ros::Duration dur(google::protobuf::util::TimeUtil::DurationToSeconds(param.duration()));
                    metrics_msg.electric_power = dur;
                }
            }
        
            metrics_pub.publish(metrics_msg);
            _last_updated["metrics"] = std::chrono::steady_clock::now();
        }

        if (update_ready("state")) {
            bosdyn::api::RobotState state = _spot.getSpotState()->robotState();

            // joint states
            joint_state_pub.publish(get_joint_state_from_robotstate(state));
            
            // leases

            // odometry (twist and regular)
            odometry_twist_pub.publish(get_odom_twist_from_state(state));
            odometry_pub.publish(get_odom_from_state(state));

            // feet
            feet_pub.publish(get_foot_states_from_robotstate(state));

            // estop
            estop_pub.publish(get_estop_states_from_robotstate(state));

            // power state
            power_pub.publish(get_power_state_from_state(state));

            // battery state
            battery_pub.publish(get_battery_states_from_state(state));
            
            _last_updated["robot_state"] = std::chrono::steady_clock::now();
        }

        // feedback (todo: moving)
        spot_msgs::Feedback feedback_msg;
        CoreLayer::SpotFeedback fb = _spot.getSpotBase()->getFeedback();
        feedback_msg.species = fb.species;
        feedback_msg.computer_serial_number = fb.computer_serial_number;
        feedback_msg.nickname = fb.nickname;
        feedback_msg.serial_number = fb.serial_number;
        feedback_msg.standing = _spot.getSpotControl()->isStanding();
        feedback_msg.sitting = !_spot.getSpotControl()->isStanding();
        feedback_pub.publish(feedback_msg);

        // mobility params
        spot_msgs::MobilityParams mobility_params_msg;
        bosdyn::api::spot::MobilityParams mobility_params = _spot.getSpotControl()->getMobilityParams();
        mobility_params_msg.body_control.position.x = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().position().x();
        mobility_params_msg.body_control.position.y = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().position().y();
        mobility_params_msg.body_control.position.z = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().position().z();
        mobility_params_msg.body_control.orientation.x = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().rotation().x();
        mobility_params_msg.body_control.orientation.y = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().rotation().y();
        mobility_params_msg.body_control.orientation.z = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().rotation().z();
        mobility_params_msg.body_control.orientation.w = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().rotation().w();
        mobility_params_msg.locomotion_hint = mobility_params.locomotion_hint();
        mobility_params_msg.stair_hint = mobility_params.stair_hint();
        mobility_params_pub.publish(mobility_params_msg);

        ros::spinOnce();
        rate.sleep();
    }

    // sit and power off
    _spot.sit();
    _spot.powerOffBlocking();
}

int main(int argc, char **argv) {
    const std::string username = argv[1];
    const std::string password = argv[2];

    SpotROSNode node;
    node.run(argc, argv, username, password);
}