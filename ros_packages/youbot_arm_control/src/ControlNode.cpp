#include <ControlNode.h>


ControlNode::ControlNode() : nh_for_params("~"){
    ROS_INFO_STREAM("Initialization...");
    status = NOTASK;
    qd.resize(N);

    M.resize(N*(N+1)/2);
    C.resize(N*N);
    G.resize(N);
    tau.resize(N);

    tau_e.torques.resize(N);
    for (int i = 0; i < N; i++) {
        tau_e.torques[i].unit = "m^2 kg s^-2 rad^-1";
        tau_e.torques[i].joint_uri = "arm_joint_" + STR(i+1);
    }

    nh_for_params.param < std::vector<double> >("pose_regulation/kp", kpp, std::vector<double>(N, 1.0));
    nh_for_params.param < std::vector<double> >("pose_regulation/kd", kdp, std::vector<double>(N, 1.0));

    pose_sub = nh.subscribe("target_pose", 1, &ControlNode::poseCallback, this);
    js_sub = nh.subscribe("joint_states", 1, &ControlNode::jsCallback, this);
    torque_pub = nh.advertise<brics_actuator::JointTorques>("arm_1/arm_controller/torques_command", 1, true);
    ROS_INFO_STREAM("Initialization done");
}


ControlNode::~ControlNode(){
    torque_pub.shutdown();
    js_sub.shutdown();
}


void ControlNode::calcMC(std::vector<double> q, std::vector<double> dq){
}


void ControlNode::calcGTau(std::vector<double> q, std::vector<double> dq){
    G[0] = 0.0;
    G[1] = 0.0;
    G[2] = 0.0;
    G[3] = 0.0;
    G[4] = 0.0;

    tau[0] = 0.0;
    tau[1] = 0.0;
    tau[2] = 0.0;
    tau[3] = 0.0;
    tau[4] = 0.0;
}


void ControlNode::jsCallback(const sensor_msgs::JointState &msg){
    switch(status){
        case TRAJECTORY:
            calcMC(msg.position, msg.velocity);
        case POSE:
            calcGTau(msg.position, msg.velocity);
            break;
        default:
            return;
    }

    switch(status){
        case POSE:
            for (int i = 0; i < N; i++) {
                tau_e.torques[i].value = kpp[i] * (qd[i] - msg.position[i]) - kdp[i] * msg.velocity[i] + G[i] + tau[i];
                tau_e.torques[i].timeStamp = ros::Time::now();
            }
            break;
        default:
            break;
    }
    torque_pub.publish(tau_e);
}


void ControlNode::poseCallback(const brics_actuator::JointPositions &msg){
    ROS_INFO_STREAM("New target pose received");
    status = POSE;
    for (int i = 0; i < N; i++) {
        qd[i] = msg.positions[i].value;
    }
}


void ControlNode::work(){
    ROS_INFO_STREAM("Start waiting for tasks");
    ros::spin();
    ROS_INFO_STREAM("Shutdowning...");
}
