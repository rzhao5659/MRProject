#pragma once

#include <list>
#include <string>

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
    double exploration_goal_[3];     // Current exploration goal.
    bool is_exploration_done_;       // If exploration is done, then it means there is no more exploration goal.
    double exploration_goal_score_;  // current exploration goal score.
    // Populated by ROS parameters.
    int min_frontier_size_;
    double vmax_;
    double wmax_;
    std::string global_frame_;
    // If enabled by visualize_, these will handle rviz visualization.
    bool visualize_;
    ros::Publisher rviz_marker_pub_;
    ros::Timer rviz_pub_timer_;
    // If enabled by publish_map_, these will publish maps.
    bool publish_map_;
    ros::Publisher map_pub_;
    ros::Publisher visited_pub_;
    ros::Timer map_pub_timer_;

    bool getExplorationGoal(exploration_alg::ExplorationGoalRequest::Request& req, exploration_alg::ExplorationGoalRequest::Response& res);
    double scorePose(double wx, double wy, double wtheta);
    void visualizeFrontiers();
    void visualizeActiveArea();
    void visualizeExplorationGoal();
    void visualizeCallback();
    void clearMarkers();
    void publishMap();

   public:
    ExploreNode();
};
