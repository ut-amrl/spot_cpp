#ifndef SPOT_ROS_CPP_H
#define SPOT_ROS_CPP_H

#include "helpers.h"

/* SpotPoller: polls Spot for state data and updates public variables w/ new information to be published by ROS node */
class SpotROSWrapper {
public:
    SpotROSWrapper(std::shared_ptr<Spot> spot);

    void init(std::map<std::string, ros::Publisher> publishers);
    void update_images(const std::string &side);
    void update_metrics();
    void update_state();

private:
    std::shared_ptr<Spot> _spot;

    /* ros publishers */
    std::map<std::string, ros::Publisher> _publishers;

    /* time at which to poll certain things (dependent on frequencies, so per second) */
    std::map<std::string, std::chrono::milliseconds> _rates;

    /* _last_updated: timepoints at which members were last updated */
    std::map<std::string, std::chrono::steady_clock::time_point> _last_updated;

    /* checks whether the member should be updated based on the last updated time and now*/
    bool update_ready(const std::string &member);
};

class SpotROSNode {
public:
    SpotROSNode(int argc, char **argv, const std::string &username, const std::string &password);
    
    void run();
private:
    /* listeners */
    void cmd_vel_callback(const geometry_msgs::Twist &msg);

    /* services */
    bool sit_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool stand_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool power_on_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool power_off_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);

private:
    std::map<std::string, ros::Publisher> _publishers;
    std::map<std::string, ros::Subscriber> _subscribers;
    std::map<std::string, ros::ServiceServer> _services;
private:
    std::shared_ptr<Spot> _spot;
    SpotROSWrapper _wrapper;
    ros::NodeHandle _nh;
    ros::Rate _rate;
    std::chrono::steady_clock::time_point _t_last_non_zero_vel_cmd;
};

#endif