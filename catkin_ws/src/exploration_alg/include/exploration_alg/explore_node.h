#pragma once
#include "ros/ros.h"

/**
 * A ROS node that handles exploration. This will run frontier detection and select an exploration goal.
 */
class ExploreNode {
   private:
    ros::NodeHandle node;

   public:
    ExploreNode();
};
