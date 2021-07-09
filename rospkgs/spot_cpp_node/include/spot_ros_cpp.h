#ifndef SPOT_ROS_CPP_H
#define SPOT_ROS_CPP_H

#include "helpers.h"

/* SpotPoller: polls Spot for state data and updates public variables w/ new information to be published by ROS node */
class SpotPoller {
public:
    SpotPoller(Spot spot, std::map<std::string, double> frequencies);
    ~SpotPoller();
private:
    /* function which runs on thread */
    void update();

    /* checks whether the member should be updated based on the last updated time and now*/
    bool update_ready(const std::string &member);
private:
    /* protects data members while they're being accessed */
    std::mutex _mu;

    /* time at which to poll certain things (dependent on frequencies, so per second) */
    /*
    default:
        robot_state -> 1 / 20
        metrics -> 1 / 0.04
        lease -> 1 / 1.0
        front_image -> 1 / 10.0
        side_image -> 1 / 10.0
        rear_image -> 1 / 10.0
    */
    std::map<std::string, std::chrono::milliseconds> _rates;

    /* data members */
    bosdyn::api::RobotMetrics _metrics;

    // robot state
    bosdyn::api::RobotState _state;
    
    // [source, imageresponse (covers grayscale / depth)]
    std::map<std::string, bosdyn::api::ImageResponse> _images;

    // timepoints at when variables were last updated
    std::map<std::string, std::chrono::steady_clock::time_point> _last_updated;

    Spot _spot;
    std::thread _thread;
    bool _keepRunning;

    /* todo: leasing */
};

class SpotROSNode {
public:
    SpotROSNode(int argc, char **argv, const std::string &username, const std::string &password);
    
    void run();
private:
    void cmd_vel_callback(const geometry_msgs::Twist &msg);
private:
    std::map<std::string, ros::Publisher> _publishers;
    std::map<std::string, ros::Subscriber> _subscribers;
    std::map<std::string, ros::ServiceServer> _services;
private:
    Spot _spot;
    ros::NodeHandle _nh;
    ros::Rate _rate;
    std::chrono::steady_clock::time_point _t_last_non_zero_vel_cmd;
};

#endif