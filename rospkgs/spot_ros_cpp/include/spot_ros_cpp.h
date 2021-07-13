#ifndef SPOT_ROS_CPP_H
#define SPOT_ROS_CPP_H

#include "helpers.h"

class SpotROSNode {
public:
    SpotROSNode();

    void run(int argc, char **argv, const std::string &username, const std::string &password);
private:
    /* listeners */
    void cmd_vel_callback(const geometry_msgs::Twist &msg);

    /* services */
    bool sit_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool stand_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool power_on_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool power_off_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);

    /* utility methods */
    bool update_ready(const std::string &member);

private:
    std::map<std::string, ros::Publisher> _publishers;
    std::map<std::string, ros::Subscriber> _subscribers;
    std::map<std::string, ros::ServiceServer> _services;
private:
    Spot _spot;
    std::shared_ptr<ros::Rate> _rate;
    std::chrono::steady_clock::time_point _t_last_non_zero_vel_cmd;

    /* _last_updated: timepoints at which members were last updated */
    std::map<std::string, std::chrono::steady_clock::time_point> _last_updated;
};

#endif