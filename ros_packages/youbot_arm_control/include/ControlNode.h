#ifndef _CONTROLNODE_
#define _CONTROLNODE_

#include <vector>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointValue.h>
#include <brics_actuator/JointTorques.h>
#include <brics_actuator/JointPositions.h>

#define N 5
enum Status {NOTASK, POSE, TRAJECTORY};
#define STR(A) (static_cast<std::ostringstream*>( &(std::ostringstream() << A) )->str())

class ControlNode{

    Status status;
    ros::Publisher torque_pub;
    ros::Subscriber js_sub, pose_sub;
    ros::NodeHandle nh, nh_for_params;
    brics_actuator::JointTorques tau_e;
    std::vector<double> qd;
    std::vector<double> kpp, kdp;
    std::vector<double> M, C, G, tau;

    void calcMC(std::vector<double> q, std::vector<double> dq);
    void calcGTau(std::vector<double> q, std::vector<double> dq);
    void jsCallback(const sensor_msgs::JointState &msg);
    void poseCallback(const brics_actuator::JointPositions &msg);

public:

    ControlNode();
    ~ControlNode();
    void work();
};

#endif //_CONTROLNODE_
