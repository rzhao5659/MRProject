#include "explore_node.h"

#include "ros/ros.h"

ExploreNode::ExploreNode() {}

int main(int argc, char** argv) {
    ros::init(argc, argv, "explore_node");
    ExploreNode explorenode;
    ros::spin();
    return 0;
}
