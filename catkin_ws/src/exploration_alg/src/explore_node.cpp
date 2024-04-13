#include "explore_node.h"

#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "exploration_alg/ExplorationGoalRequest.h"
#include "frontier_detector.h"
#include "map_client.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"

using namespace exploration_alg;

ExploreNode::ExploreNode() : node_(), pose_listener_(&node_), map_client_(node_, pose_listener_), frontier_detector_(node_, map_client_.getMap(), &pose_listener_) {
    this->node_.param<int>("min_frontier_size", this->min_frontier_size_, 15);
    this->node_.param<double>("max_linear_vel", this->vmax_, 0.5);
    this->node_.param<double>("max_angular_vel", this->wmax_, 0.5);
    this->node_.param<bool>("visualize", visualize_, true);
    this->node_.param<bool>("publish_map", publish_map_, false);  // useful for testing, otherwise don't
    this->node_.param<std::string>("global_frame", this->global_frame_, "map");

    this->exploration_goal_srv_ = this->node_.advertiseService("/explore/get_goal", &ExploreNode::getExplorationGoal, this);
    this->is_exploration_done_ = true;  // this is just the default value.
    this->exploration_goal_score_ = -1;

    if (visualize_) {
        this->rviz_marker_pub_ = this->node_.advertise<visualization_msgs::MarkerArray>("/explore/rviz/frontiers", 1);
        this->rviz_pub_timer_ = this->node_.createTimer(ros::Duration(0.1), std::bind(&ExploreNode::visualizeCallback, this));
    }
    if (publish_map_) {
        this->map_pub_ = this->node_.advertise<nav_msgs::OccupancyGrid>("/explore/map/occupied", 10);
        this->visited_pub_ = this->node_.advertise<nav_msgs::OccupancyGrid>("/explore/map/visited", 10);
        this->map_pub_timer_ = this->node_.createTimer(ros::Duration(1.0), std::bind(&ExploreNode::publishMap, this));
    }

    ROS_INFO("ExploreNode initialized");
}

void ExploreNode::publishMap() {
    nav_msgs::OccupancyGrid occupancy_grid_msg;
    occupancy_grid_msg.header.stamp = ros::Time::now();
    occupancy_grid_msg.header.frame_id = "map";
    auto map = this->map_client_.getMap();
    occupancy_grid_msg.info.resolution = map->resolution;        // meters per pixel
    occupancy_grid_msg.info.width = map->size_x;                 // number of cells in x direction
    occupancy_grid_msg.info.height = map->size_y;                // number of cells in y direction
    occupancy_grid_msg.info.origin.position.x = map->origin_wx;  // origin of the map in meters
    occupancy_grid_msg.info.origin.position.y = map->origin_wy;
    occupancy_grid_msg.info.origin.position.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.x = 0.0;
    occupancy_grid_msg.info.origin.orientation.y = 0.0;
    occupancy_grid_msg.info.origin.orientation.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.w = 1.0;
    // Occupancy state data.
    occupancy_grid_msg.data.resize(occupancy_grid_msg.info.width * occupancy_grid_msg.info.height);
    for (int i = 0; i < map->size_x * map->size_y; i++) {
        occupancy_grid_msg.data[i] = map->map_data[i].occupancy_state;
    }
    this->map_pub_.publish(occupancy_grid_msg);
    // Visited state data
    for (int i = 0; i < map->size_x * map->size_y; i++) {
        occupancy_grid_msg.data[i] = map->map_data[i].visited_state == true ? 100 : -1;
    }
    this->visited_pub_.publish(occupancy_grid_msg);
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

bool ExploreNode::getExplorationGoal(ExplorationGoalRequest::Request& req, ExplorationGoalRequest::Response& res) {
    ROS_INFO("Explore Node received an exploration goal request.");

    // Filter out small size frontiers.
    std::vector<Frontier*> filtered_frontiers;
    for (Frontier& frontier : this->frontier_detector_.frontiers) {
        if (frontier.size >= this->min_frontier_size_) {
            filtered_frontiers.emplace_back(&frontier);
        }
    }

    // Check whether there are any frontiers left.
    if (filtered_frontiers.size() == 0) {
        res.is_exploration_done = true;
        ROS_INFO("Exploration is completed: No more exploration goal was found.");
        return true;
    }

    // If there are frontiers:
    // Score each frontier centroid.
    // Get robot current pose. Use current heading as goal heading.
    double robot_wx, robot_wy, robot_wtheta;
    this->pose_listener_.getRobotWorldPosition(robot_wx, robot_wy, robot_wtheta);
    double best_score = -1;
    double best_exploration_goal[3];  // x,y,theta
    for (Frontier* frontier : filtered_frontiers) {
        double score = scorePose(frontier->centroid[0], frontier->centroid[1], robot_wtheta);
        ROS_DEBUG("Pose(%.3f, %.3f, %.3f) received score %.3f", frontier->centroid[0], frontier->centroid[1], robot_wtheta, score);
        if (score > best_score) {
            best_score = score;
            best_exploration_goal[0] = frontier->centroid[0];
            best_exploration_goal[1] = frontier->centroid[1];
            best_exploration_goal[2] = robot_wtheta;
        }
    }

    // Store best score and best goal.
    this->exploration_goal_score_ = best_score;
    memcpy(this->exploration_goal_, best_exploration_goal, sizeof(best_exploration_goal));
    ROS_DEBUG("Best exploration goal is Pose(%.3f, %.3f, %.3f) with score %.3f", best_exploration_goal[0], best_exploration_goal[1], best_exploration_goal[2], best_score);

    // Respond with the selected goal.
    res.is_exploration_done = false;
    res.x = best_exploration_goal[0];
    res.y = best_exploration_goal[1];
    res.theta = best_exploration_goal[2];
    ROS_INFO("Computed next explorationGoal(%.3f, %.3f, %.3f)", res.x, res.y, res.theta);
    return true;
}

void ExploreNode::visualizeCallback() {
    clearMarkers();
    visualizeFrontiers();
    visualizeActiveArea();
    visualizeExplorationGoal();
}

void ExploreNode::visualizeExplorationGoal() {
    if (!is_exploration_done_) {
        visualization_msgs::MarkerArray markers_msg;
        std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
        visualization_msgs::Marker arrow;
        arrow.ns = "exploration_goal";
        arrow.header.frame_id = this->global_frame_;
        arrow.header.stamp = ros::Time::now();
        arrow.pose.position.x = this->exploration_goal_[0];
        arrow.pose.position.y = this->exploration_goal_[1];
        arrow.pose.position.z = 0.0;
        arrow.pose.orientation.z = sin(this->exploration_goal_[2] / 2);  // yaw
        arrow.pose.orientation.w = cos(this->exploration_goal_[2] / 2);
        arrow.scale.x = 0.5;  // arrow length
        arrow.scale.y = 0.1;  // arrow width
        arrow.scale.z = 0.1;  // arrow height
        arrow.color.g = 1.0;  // green.
        arrow.color.a = 1.0;
        arrow.lifetime = ros::Duration();  // forever.
        markers.push_back(arrow);
        this->rviz_marker_pub_.publish(markers_msg);
    }
}

void ExploreNode::clearMarkers() {
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    // Create a reset marker that deletes all markers (regardless of namespace).
    visualization_msgs::Marker reset_marker;
    reset_marker.header.frame_id = this->global_frame_;
    reset_marker.header.stamp = ros::Time::now();
    reset_marker.pose.orientation.w = 1.0;
    reset_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.push_back(reset_marker);
    this->rviz_marker_pub_.publish(markers_msg);
}

void ExploreNode::visualizeActiveArea() {
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    // Common header for all markers.
    std_msgs::Header header;
    header.frame_id = this->global_frame_;
    header.stamp = ros::Time::now();

    // Set up rviz markers for active area
    visualization_msgs::Marker line_strip;
    line_strip.header = header;
    line_strip.ns = "active_area";
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.pose.orientation.w = 1.0;
    line_strip.scale.x = 0.05;  // x is the width of line.
    line_strip.color.r = 1.0;   // red line for active area
    line_strip.color.a = 1.0;
    line_strip.lifetime = ros::Duration();  // forever.
    line_strip.id = 0;
    line_strip.frame_locked = true;

    // Add current active area.
    std::shared_ptr<Map2D> map = this->map_client_.getMap();
    int xmin = map->active_area[0];
    int ymin = map->active_area[1];
    int xmax = map->active_area[2];
    int ymax = map->active_area[3];

    geometry_msgs::Point p;
    p.z = 0.0;
    // convert map coordinates to world coordinates
    map->mapToWorld(xmin, ymin, p.x, p.y);
    line_strip.points.push_back(p);
    map->mapToWorld(xmax, ymin, p.x, p.y);
    line_strip.points.push_back(p);
    map->mapToWorld(xmax, ymax, p.x, p.y);
    line_strip.points.push_back(p);
    map->mapToWorld(xmin, ymax, p.x, p.y);
    line_strip.points.push_back(p);
    map->mapToWorld(xmin, ymin, p.x, p.y);
    line_strip.points.push_back(p);
    markers.push_back(line_strip);

    this->rviz_marker_pub_.publish(markers_msg);
}

void ExploreNode::visualizeFrontiers() {
    // Filter out small size frontiers.
    std::vector<Frontier*> filtered_frontiers;
    int num_filtered_out = 0;
    for (Frontier& frontier : this->frontier_detector_.frontiers) {
        if (frontier.size >= this->min_frontier_size_) {
            filtered_frontiers.emplace_back(&frontier);
            continue;
        }
        num_filtered_out++;
    }

    ROS_DEBUG("Total number of frontiers: %ld, filtered out %d", this->frontier_detector_.frontiers.size(), num_filtered_out);
    ROS_DEBUG("Visualizing remaining %lu frontiers", filtered_frontiers.size());
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;

    // Common header for all markers.
    std_msgs::Header header;
    header.frame_id = this->global_frame_;
    header.stamp = ros::Time::now();

    // Set up rviz markers for frontier cells.
    visualization_msgs::Marker points;
    points.header = header;
    points.ns = "frontier_cells";
    points.type = visualization_msgs::Marker::POINTS;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.05;  // x and y scale width/height of point.
    points.scale.y = 0.05;
    points.color.b = 1.0;  // blue lines between points.
    points.color.a = 1.0;
    points.lifetime = ros::Duration();  // forever.
    points.frame_locked = true;

    // Setup rviz marker for frontier centroid
    visualization_msgs::Marker sphere;
    sphere.header = header;
    sphere.ns = "frontier_centroid";
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = 0.3;
    sphere.scale.y = 0.3;
    sphere.scale.z = 0.3;
    sphere.color.b = 1.0;  // Blue color sphere.
    sphere.color.a = 1.0;
    sphere.lifetime = ros::Duration();  // forever.
    sphere.frame_locked = true;

    int id = 0;
    for (const Frontier* frontier : filtered_frontiers) {
        points.id = id;
        sphere.id = id;
        points.points.clear();
        id++;

        // Draw Frontier cells
        for (const auto& frontier_cell : frontier->cells) {
            geometry_msgs::Point p;
            // convert map coordinates to world coordinates
            std::shared_ptr<Map2D> map = this->map_client_.getMap();
            map->mapToWorld(frontier_cell.x, frontier_cell.y, p.x, p.y);
            p.z = 0.0;
            // add to points.
            points.points.push_back(p);
        }

        // Draw Frontier centroid
        sphere.pose.position.x = frontier->centroid[0];
        sphere.pose.position.y = frontier->centroid[1];

        // Add them to marker array
        markers.push_back(points);
        markers.push_back(sphere);
    }

    // Publish new markers
    this->rviz_marker_pub_.publish(markers_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "explore_node");
    ExploreNode explorenode;
    ros::spin();
    return 0;
}
