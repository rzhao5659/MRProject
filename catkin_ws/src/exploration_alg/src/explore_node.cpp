#include "explore_node.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "exploration_alg/ExplorationGoalRequest.h"
#include "frontier_detector.h"
#include "map_client.h"
#include "ros/ros.h"

using namespace exploration_alg;

ExploreNode::ExploreNode() : node_(), pose_listener_(&node_), map_client_(node_, pose_listener_), frontier_detector_(node_, map_client_.getMap(), &pose_listener_) {
    this->node_.param<int>("min_frontier_size", this->min_frontier_size_, 5);
    this->node_.param<double>("max_linear_vel", this->vmax_, 0.5);
    this->node_.param<double>("max_angular_vel", this->wmax_, 0.5);
    this->exploration_goal_srv_ = this->node_.advertiseService("/explore/get_goal", &ExploreNode::getExplorationGoal, this);
}

double ExploreNode::scorePose(double wx, double wy, double wtheta) {
    // Compute information gain. TODO
    double information_gain = 1;

    // Get robot's current pose.
    double robot_wx, robot_wy, robot_wtheta;
    this->pose_listener_.getRobotWorldPosition(robot_wx, robot_wy, robot_wtheta);

    // Compute a time to reach with simple heuristic (assuming that we are driving at vmax and wmax, and straight path is collision free)
    double heading_time_to_reach = abs(wtheta - robot_wtheta) / this->wmax_;
    double position_time_to_reach = sqrt(pow(wx - robot_wx, 2) + pow(wy - robot_wy, 2)) / this->vmax_;
    double time_to_reach = std::max(heading_time_to_reach, position_time_to_reach);

    return information_gain / time_to_reach;
}

// Using a simple strategy for now, until i have finished testing everything else.
bool ExploreNode::getExplorationGoal(ExplorationGoalRequest::Request &req, ExplorationGoalRequest::Response &res) {
    // Filter out small size frontiers.
    std::vector<Frontier *> filtered_frontiers;
    for (Frontier &frontier : this->frontier_detector_.frontiers) {
        if (frontier.size >= this->min_frontier_size_) {
            filtered_frontiers.emplace_back(&frontier);
        }
    }

    // Score each frontier centroid.
    double best_score = -1;
    double best_exploration_goal[3];  // x,y,theta
    // Get robot current pose. Use current heading as goal heading.
    double robot_wx, robot_wy, robot_wtheta;
    this->pose_listener_.getRobotWorldPosition(robot_wx, robot_wy, robot_wtheta);
    for (Frontier *frontier : filtered_frontiers) {
        double score = scorePose(frontier->centroid[0], frontier->centroid[1], robot_wtheta);
        if (score > best_score) {
            best_score = score;
            best_exploration_goal[0] = frontier->centroid[0];
            best_exploration_goal[1] = frontier->centroid[1];
            best_exploration_goal[2] = robot_wtheta;
        }
    }

    // Respond with the selected goal.
    if (best_score != -1) {
        res.is_valid = true;
        res.x = best_exploration_goal[0];
        res.y = best_exploration_goal[1];
        res.theta = best_exploration_goal[2];
        ROS_INFO("ExplorationGoal(%.3f, %.3f, %.3f)", res.x, res.y, res.theta);
    } else {
        res.is_valid = false;
        ROS_INFO("Exploration is completed: No more exploration goal was found.");
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "explore_node");
    ExploreNode explorenode;
    ros::spin();
    return 0;
}
