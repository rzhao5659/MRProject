#pragma once

#include "exploration_alg/ExplorationGoalRequest.h"
#include "frontier_detector.h"
#include "map.h"
#include "map_client.h"
#include "pose_listener.h"
#include "ros/ros.h"

/**
 * A ROS node that handles exploration. This will run frontier detection and select an exploration goal.
 */
class ExploreNode {
   private:
    ros::NodeHandle node_;
    PoseListener pose_listener_;
    Map2DClient map_client_;
    FrontierDetector frontier_detector_;
    ros::ServiceServer exploration_goal_srv_;
    int min_frontier_size_;
    double vmax_;
    double wmax_;

    bool getExplorationGoal(exploration_alg::ExplorationGoalRequest::Request &req, exploration_alg::ExplorationGoalRequest::Response &res);
    double scorePose(double wx, double wy, double wtheta);

   public:
    ExploreNode();
};
