#include <ControlNode.h>

int main (int argc, char **argv){
    ros::init(argc, argv, "control_node");
    ControlNode node;
    node.work();
    return 0;
}
