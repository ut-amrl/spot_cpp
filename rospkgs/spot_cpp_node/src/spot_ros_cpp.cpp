#include "spot_ros_cpp.h"

/* todo: implement callbacks, find way to pass callbacks */
SpotPoller::SpotPoller(Spot spot, std::map<std::string, double> frequencies) : 
        _spot(spot), 
        _thread(&SpotPoller::update, this), 
        _keepRunning(true) {
    // fill out rates
    for (auto it = frequencies.begin(); it != frequencies.end(); it++) {
        std::chrono::milliseconds ms((int) ((1 / it->second) * 1000));
        _rates.insert(std::pair<std::string, std::chrono::milliseconds>(it->first, ms));
    }

    // fill out last updated w/ default vals
    for (auto it = frequencies.begin(); it != frequencies.end(); it++) {
        std::chrono::steady_clock::time_point tp;
        _last_updated.insert(std::pair<std::string, std::chrono::steady_clock::time_point>(it->first, tp));
    }
}

SpotPoller::~SpotPoller() {
    _keepRunning = false;
    if (_thread.joinable()) {
        _thread.join();
    }
}

bool SpotPoller::update_ready(const std::string &member) {
    // if greater, time to poll
    return std::chrono::steady_clock::now() - _last_updated.find(member)->second > _rates.find(member)->second;
}

void SpotPoller::update() {
    while (_keepRunning) {
        if (update_ready("robot_state") {

        }

        if (update_ready("lease")) {

        }

        if (update_ready("metrics")) {

        }

        if (update_ready("frontleft_image")) {

        }

        if (update_ready("frontright_image")) {

        }

        if (update_ready("left_image")) {

        }

        if (update_ready("right_image")) {

        }

        if (update_ready("back_image")) {

        }
    }
}

SpotROSNode::SpotROSNode(int argc, char **argv, const std::string &username, const std::string &password) :
        _rate(50) {
    ros::init(argc, argv, "spot_cpp_node");

    /* todo: tf */

    /* Publishers */
    // image
    _publishers.insert(std::pair<std::string, ros::Publisher>("camera/back/image", _nh.advertise<sensor_msgs::Image>("camera/back/image", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("camera/frontleft/image", _nh.advertise<sensor_msgs::Image>("camera/frontleft/image", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("camera/frontright/image", _nh.advertise<sensor_msgs::Image>("camera/frontright/image", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("camera/left/image", _nh.advertise<sensor_msgs::Image>("camera/left/image", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("camera/right/image", _nh.advertise<sensor_msgs::Image>("camera/right/image", 10)));

    // depth
    _publishers.insert(std::pair<std::string, ros::Publisher>("depth/back/image", _nh.advertise<sensor_msgs::Image>("depth/back/image", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("depth/frontleft/image", _nh.advertise<sensor_msgs::Image>("depth/frontleft/image", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("depth/frontright/image", _nh.advertise<sensor_msgs::Image>("depth/frontright/image", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("depth/left/image", _nh.advertise<sensor_msgs::Image>("depth/left/image", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("depth/right/image", _nh.advertise<sensor_msgs::Image>("depth/right/image", 10)));

    // image camera info
    _publishers.insert(std::pair<std::string, ros::Publisher>("camera/back/camera_info", _nh.advertise<sensor_msgs::Image>("camera/back/camera_info", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("camera/frontleft/camera_info", _nh.advertise<sensor_msgs::Image>("camera/frontleft/camera_info", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("camera/frontright/camera_info", _nh.advertise<sensor_msgs::Image>("camera/frontright/camera_info", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("camera/left/camera_info", _nh.advertise<sensor_msgs::Image>("camera/left/camera_info", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("camera/right/camera_info", _nh.advertise<sensor_msgs::Image>("camera/right/camera_info", 10)));

    // depth camera info
    _publishers.insert(std::pair<std::string, ros::Publisher>("depth/back/camera_info", _nh.advertise<sensor_msgs::Image>("depth/back/camera_info", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("depth/frontleft/camera_info", _nh.advertise<sensor_msgs::Image>("depth/frontleft/camera_info", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("depth/frontright/camera_info", _nh.advertise<sensor_msgs::Image>("depth/frontright/camera_info", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("depth/left/camera_info", _nh.advertise<sensor_msgs::Image>("depth/left/camera_info", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("depth/right/camera_info", _nh.advertise<sensor_msgs::Image>("depth/right/camera_info", 10)));

    // status
    _publishers.insert(std::pair<std::string, ros::Publisher>("joint_states", _nh.advertise<sensor_msgs::JointState>("joint_states", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/metrics", _nh.advertise<spot_msgs::Metrics>("status/metrics", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/leases", _nh.advertise<spot_msgs::LeaseArray>("status/leases", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("odometry/twist", _nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("odometry/twist", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("odometry", _nh.advertise<nav_msgs::Odometry>("odometry", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/feet", _nh.advertise<spot_msgs::FootStateArray>("status/feet", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/estop", _nh.advertise<spot_msgs::EStopStateArray>("status/estop", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/power_state", _nh.advertise<spot_msgs::PowerState>("status/power_state", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/battery_states", _nh.advertise<spot_msgs::BatteryStateArray>("status/battery_states", 10)));

    // feedback
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/feedback", _nh.advertise<spot_msgs::Feedback>("status/feedback", 10)));

    // mobility params
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/mobility_params", _nh.advertise<spot_msgs::MobilityParams>("status/mobility_params", 10)));

    /* Subscribers TODO: define callbacks*/
    // ros::Subscriber cmd_vel_sub = _nh.subscribe("cmd_vel", );
    // ros::Subscriber body_pose_sub = _nh.subscribe("body_pose");

    /* Services TODO: callbacks */
    // ros::ServiceServer sit_srv = _nh.advertiseService("sit");
    // ros::ServiceServer stand_srv = _nh.advertiseService("stand");
    // ros::ServiceServer power_on_srv = _nh.advertiseService("power_on");
    // ros::ServiceServer power_off_srv = _nh.advertiseService("power_off");
    // ros::ServiceServer stair_mode_srv = _nh.advertiseService("stair_mode");
    // ros::ServiceServer locomotion_mode_srv = _nh.advertiseService("locomotion_mode");
}

void SpotROSNode::run() {
    // create translation layer

    while (ros::ok()) {
        // act on subscribed topics


        // poll spot and publish data

        // feedback
        spot_msgs::Feedback feedback_msg;
        _publishers.find("status/feedback")->second.publish(feedback_msg);

        // mobility params
        spot_msgs::MobilityParams mobilityparams_msg;
        _publishers.find("status/mobility_params")->second.publish(mobilityparams_msg);

        ros::spinOnce();
        _rate.sleep();
    }
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

int main(int argc, char **argv) {
    const std::string username = argv[1];
    const std::string password = argv[2];

    // SpotROSNode node(username, password);
    // node.run(argc, argv);
}