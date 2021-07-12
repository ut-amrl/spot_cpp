#include "spot_ros_cpp.h"

/* todo: possible use function pointers in map instead of switch case for polling */

/* MEMBERS: list of members that get updated by SpotROSWrapper for consumption by SpotROSNode*/
static const std::vector<std::string> MEMBERS {
    "robot_state",
    //"lease",
    "metrics",
    "front_image",
    "side_image",
    "rear_image"    
};

/* FREQUENCIES: how many times a second we should be updating the member */
static const std::map<std::string, double> FREQUENCIES {
    {"robot_state", 20.0},
    {"metrics", 0.04},
    //{"lease", 1.0},
    {"front_image", 10.0},
    {"side_image", 10.0},
    {"rear_image", 10.0}
};

static const int MS_MULTIPLIER = 1000;

SpotROSWrapper::SpotROSWrapper(std::shared_ptr<Spot> spot) : _spot(spot) {
    // fill out rates
    for (auto it = FREQUENCIES.begin(); it != FREQUENCIES.end(); it++) {
        std::chrono::milliseconds ms((int) ((1 / it->second) * MS_MULTIPLIER));
        _rates[it->first] = ms;
    }

    // fill out last updated w/ default vals
    for (auto it = FREQUENCIES.begin(); it != FREQUENCIES.end(); it++) {
        std::chrono::steady_clock::time_point tp;
        _last_updated[it->first] = tp;
    }
}

void SpotROSWrapper::init(std::map<std::string, ros::Publisher> publishers) {
    _publishers = publishers;
}

bool SpotROSWrapper::update_ready(const std::string &member) {
    // if greater, time to poll
    return std::chrono::steady_clock::now() - _last_updated[member] > _rates[member];
}

void SpotROSWrapper::update_images(const std::string &side) {
    if (update_ready(side)) {

        // implement imageresponse in sdk

        // set last updated to now
        _last_updated[side] = std::chrono::steady_clock::now();
    }
}

void SpotROSWrapper::update_metrics() {
    if (update_ready("metrics")) {
        bosdyn::api::RobotMetrics metrics = _spot->getSpotState()->robotMetrics();
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
        
        _publishers["status/metrics"].publish(metrics_msg);
        _last_updated["metrics"] = std::chrono::steady_clock::now();
    }
}

void SpotROSWrapper::update_state() {
    if (update_ready("robot_state")) {
        bosdyn::api::RobotState state = _spot->getSpotState()->robotState();

        // joint states
        _publishers["joint_states"].publish(get_joint_state_from_robotstate(state));
        
        // leases

        // odometry (twist and regular)
        _publishers["odometry/twist"].publish(get_odom_twist_from_state(state));
        _publishers["odometry"].publish(get_odom_from_state(state));

        // feet
        _publishers["status/feet"].publish(get_foot_states_from_robotstate(state));

        // estop
        _publishers["status/estop"].publish(get_estop_states_from_robotstate(state));

        // power state
        _publishers["status/power_state"].publish(get_power_state_from_state(state));

        // battery state
        _publishers["status/battery_state"].publish(get_battery_states_from_state(state));
        
        _last_updated["robot_state"] = std::chrono::steady_clock::now();
    }
}

SpotROSNode::SpotROSNode(int argc, char **argv, const std::string &username, const std::string &password) :
        _rate(50),
        _spot(new Spot),
        _wrapper(_spot) {
    ros::init(argc, argv, "spot_cpp_node");

    /* todo: tf */
    _spot->basicInit(username, password);
    
    /* Publishers */
    // image (handled by image)
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

    // status (handled by robotstate)
    _publishers.insert(std::pair<std::string, ros::Publisher>("joint_states", _nh.advertise<sensor_msgs::JointState>("joint_states", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/metrics", _nh.advertise<spot_msgs::Metrics>("status/metrics", 10))); // handled by metrics
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/leases", _nh.advertise<spot_msgs::LeaseArray>("status/leases", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("odometry/twist", _nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("odometry/twist", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("odometry", _nh.advertise<nav_msgs::Odometry>("odometry", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/feet", _nh.advertise<spot_msgs::FootStateArray>("status/feet", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/estop", _nh.advertise<spot_msgs::EStopStateArray>("status/estop", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/power_state", _nh.advertise<spot_msgs::PowerState>("status/power_state", 10)));
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/battery_states", _nh.advertise<spot_msgs::BatteryStateArray>("status/battery_states", 10)));

    // feedback (handled by robot id, sdk)
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/feedback", _nh.advertise<spot_msgs::Feedback>("status/feedback", 10)));

    // mobility params (handled by sdk)
    _publishers.insert(std::pair<std::string, ros::Publisher>("status/mobility_params", _nh.advertise<spot_msgs::MobilityParams>("status/mobility_params", 10)));

    /* Subscribers TODO: define callbacks*/
    ros::Subscriber cmd_vel_sub = _nh.subscribe("cmd_vel", 10, &SpotROSNode::cmd_vel_callback, this);
    // ros::Subscriber body_pose_sub = _nh.subscribe("body_pose");

    /* Services TODO: callbacks */
    ros::ServiceServer sit_srv = _nh.advertiseService("sit", &SpotROSNode::sit_srv, this);
    ros::ServiceServer stand_srv = _nh.advertiseService("stand", &SpotROSNode::stand_srv, this);
    ros::ServiceServer power_on_srv = _nh.advertiseService("power_on", &SpotROSNode::power_on_srv, this);
    ros::ServiceServer power_off_srv = _nh.advertiseService("power_off", &SpotROSNode::power_off_srv, this);
    // ros::ServiceServer stair_mode_srv = _nh.advertiseService("stair_mode");
    // ros::ServiceServer locomotion_mode_srv = _nh.advertiseService("locomotion_mode");

    _wrapper.init(_publishers);
}

void SpotROSNode::run() {
    while (ros::ok()) {        
        // update topics and send messages (wrapper will handle timing)
        _wrapper.update_images("front_image");
        _wrapper.update_images("side_image");
        _wrapper.update_images("rear_image");

        _wrapper.update_metrics();
        _wrapper.update_state();

        // feedback (todo: implement in sdk)
        spot_msgs::Feedback feedback_msg;
        _publishers["status/feedback"].publish(feedback_msg);

        // mobility params
        spot_msgs::MobilityParams mobility_params_msg;
        bosdyn::api::spot::MobilityParams mobility_params = _spot->getSpotControl()->getMobilityParams();
        mobility_params_msg.body_control.position.x = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().position().x();
        mobility_params_msg.body_control.position.y = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().position().y();
        mobility_params_msg.body_control.position.z = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().position().z();
        mobility_params_msg.body_control.orientation.x = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().rotation().x();
        mobility_params_msg.body_control.orientation.y = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().rotation().y();
        mobility_params_msg.body_control.orientation.z = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().rotation().z();
        mobility_params_msg.body_control.orientation.w = mobility_params.body_control().base_offset_rt_footprint().points()[0].pose().rotation().w();
        mobility_params_msg.locomotion_hint = mobility_params.locomotion_hint();
        mobility_params_msg.stair_hint = mobility_params.stair_hint();
        _publishers["status/mobility_params"].publish(mobility_params_msg);

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
		_spot->velocityMove(msg.linear.x, msg.linear.y, msg.linear.z, vel_cmd_duration, FLAT_BODY);
	}
}

/* todo: messages and case handling for services */
bool SpotROSNode::sit_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    _spot->sit();
    return true;
}

bool SpotROSNode::stand_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    _spot->stand();
    return true;
}

bool SpotROSNode::power_on_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    _spot->powerOnBlocking();
    return true;
}

bool SpotROSNode::power_off_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    _spot->powerOffBlocking();
    return true;
}

int main(int argc, char **argv) {
    const std::string username = argv[1];
    const std::string password = argv[2];

    // SpotROSNode node(username, password);
    // node.run(argc, argv);
}