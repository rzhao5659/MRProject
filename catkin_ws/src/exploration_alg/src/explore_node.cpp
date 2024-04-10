#include "explore_node.h"

#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hi");
    ExploreNode explorenode;
    ros::spin();
    return 0;
}
