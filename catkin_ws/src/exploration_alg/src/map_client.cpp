#include "map_client.h"

#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <string>

#include "geometry_msgs/TransformStamped.h"
#include "map.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

Map2DClient::Map2DClient(ros::NodeHandle& node) {
    // Get parameter values from parameter server. If not found, set to default values.
    std::string global_frame;                 // Map frame id
    std::string base_frame;                   // Robot frame id.
    double map_resolution;                    // The resolution of the map in meters/cell. This should match to the resolution of the OccupancyGrid map you are receiving.
    double map_width;                         // The width of the map in meters.
    double map_height;                        // The height of the map in meters.
    std::string occupancy_grid_topic;         // Topic name for message nav_msgs/OccupancyGrid.
    std::string occupancy_grid_update_topic;  // Topic name for message map_msgs/OccupancyGridUpdate.

    node.param<double>("resolution", map_resolution, 0.05);
    node.param<double>("width", map_width, 10.0);
    node.param<double>("height", map_height, 10.0);
    node.param<std::string>("global_frame", global_frame, "/map");
    node.param<std::string>("base_frame", base_frame, "/base_footprint");
    node.param<std::string>("occupancy_grid_topic", occupancy_grid_topic, "/move_base/global_costmap/costmap");
    node.param<std::string>("occupancy_grid_update_topic", occupancy_grid_update_topic, "/move_base/global_costmap/costmap_updates");

    // Initialize a Map2D object:
    // Spin listening to /tf until we get the robot's current position.
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped transformStamped;
    while (node.ok()) {
        try {
            transformStamped = tf_buffer.lookupTransform(global_frame, base_frame, ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        break;
    }
    double robot_wx = transformStamped.transform.translation.x;
    double robot_wy = transformStamped.transform.translation.y;
    ROS_DEBUG("Robot's current position is (%.3f, %.3f).", robot_wx, robot_wy);
    // Initialize a Map2D object
    this->map_ = std::make_shared<Map2D>(map_resolution, map_width, map_height, robot_wx, robot_wy);

    // Subscribe to OccupancyGrid and OccupancyGridUpdate
    this->occupancy_grid_update_sub_ = node.subscribe<map_msgs::OccupancyGridUpdate>(occupancy_grid_topic, 10, updatePartialMap);
    this->occupancy_grid_sub_ = node.subscribe<nav_msgs::OccupancyGrid>(occupancy_grid_update_topic, 10, updateMap);
}

void Map2DClient::updateMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    unsigned int received_map_size_x = msg->info.width;
    unsigned int received_map_size_y = msg->info.height;
    double received_map_resolution = msg->info.resolution;
    double received_map_origin_wx = msg->info.origin.position.x;
    double received_map_origin_wy = msg->info.origin.position.y;
    ROS_DEBUG("Received a full map update (size_x:%d, size_y:%d, resolution:%d, origin_x:%.3f, origin_y:%.3f)", received_map_size_x, received_map_size_y, received_map_resolution,
              received_map_origin_wx, received_map_origin_wy);

    // Get the grid coordinates corresponding to the received map origin.
    unsigned int received_map_origin_gx, received_map_origin_gy;
    if (!this->map_->worldToMap(received_map_origin_wx, received_map_origin_wy, received_map_origin_gx, received_map_origin_gy)) {
        ROS_ERROR("updateMap failed: the received OccupancyGrid map is not within the static map boundary");
        return;
    }
    int origin_idx = this->map_->getIndex(received_map_origin_gx, received_map_origin_gy);

    // Store this received map origin grid coordinates.
    // This is because global_costmap can publish a OccupancyGridUpdate message, which contains data with respect to the last OccupancyGrid origin.
    this->last_received_map_origin_gx_ = received_map_origin_gx;
    this->last_received_map_origin_gy_ = received_map_origin_gy;

    // Update the region of map corresponding to the OccupancyGrid cells.
    for (int i = 0; i < received_map_size_x; i++) {
        for (int j = 0; j < received_map_size_y; j++) {
            this->map_->map_data[origin_idx + i + j * this->map_->size_x].occupancy_state = msg->data[i + j * received_map_size_x];
        }
    }

    // Update active area with this region boundary.
    updateActiveArea(received_map_origin_gx, received_map_origin_gy, received_map_origin_gx + received_map_size_x - 1, received_map_origin_gy + received_map_size_y - 1);
}

void Map2DClient::updatePartialMap(const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
    ROS_DEBUG("Received a partial map update");
    int x0 = last_received_map_origin_gx_ + msg->x;
    int y0 = last_received_map_origin_gy_ + msg->y;
    int xn = msg->width - 1 + x0;
    int yn = msg->height - 1 + y0;

    if (xn >= this->map_->size_x || x0 >= this->map_->size_x || yn >= this->map_->size_y || y0 >= this->map_->size_y) {
        ROS_WARN(
            "received update doesn't fully fit into existing map, "
            "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
            "map is: [0, %lu], [0, %lu]",
            x0, xn, y0, yn, this->map_->size_x - 1, this->map_->size_y - 1);
    }

    // Update map with data.
    int i = 0;
    for (int y = y0; y <= yn && y < this->map_->size_y; y++) {
        for (int x = x0; x <= xn && x < this->map_->size_x; x++) {
            int idx = this->map_->getIndex(x, y);
            this->map_->map_data[idx].occupancy_state = msg->data[i];
            i++;
        }
    }

    // Update active area with this region boundary.
    updateActiveArea(x0, y0, xn, yn);
}

std::shared_ptr<Map2D> Map2DClient::getMap() { return this->map_; }

void Map2DClient::updateActiveArea(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn) {
    this->map_->active_area[0] = std::min(this->map_->active_area[0], x0);  // xmin
    this->map_->active_area[1] = std::min(this->map_->active_area[1], y0);  // ymin
    this->map_->active_area[2] = std::max(this->map_->active_area[2], xn);  // ymin
    this->map_->active_area[3] = std::max(this->map_->active_area[3], yn);  // ymin
}