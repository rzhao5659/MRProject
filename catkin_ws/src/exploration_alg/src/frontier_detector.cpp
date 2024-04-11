#include "frontier_detector.h"

#include <tf2_ros/transform_listener.h>

#include <functional>
#include <iostream>
#include <memory>
#include <queue>
#include <unordered_set>
#include <vector>

#include "RTree.h"
#include "map.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pose_listener.h"
#include "ros/ros.h"

// Private helper functions not relevant to Frontier Detector.
template <class T>
inline void hash_combine(std::size_t& seed, const T& v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

// Hash function for cell2d_t
struct hash_cell2d_t {
    std::size_t operator()(const cell2d_t& p) const {
        std::size_t hash;
        hash_combine(hash, p.x);
        hash_combine(hash, p.y);
        return hash;
    };
};

void FrontierDetector::runDetection() {
    detectNewFrontierCells();
    getFrontiers();
}

FrontierDetector::FrontierDetector(ros::NodeHandle& node, std::shared_ptr<Map2D> map, PoseListener* pose_listener) : map_(map), pose_listener_(pose_listener) {
    double detection_frequency;
    node.param<double>("detection_frequency", detection_frequency, 10);
    // Create a timer that periodically executes the frontier detection.
    this->execute_timer_ = node.createTimer(ros::Duration(1 / detection_frequency), std::bind(&FrontierDetector::runDetection, this));
}

void FrontierDetector::resetActiveArea() {
    this->map_->active_area[0] = 0;
    this->map_->active_area[1] = 0;
    this->map_->active_area[2] = 0;
    this->map_->active_area[3] = 0;
}

cell2d_t FrontierDetector::getRobotMapPosition() {
    double wx, wy, wtheta;
    int gx, gy;
    this->pose_listener_->getRobotWorldPosition(wx, wy, wtheta);
    this->map_->worldToMap(wx, wy, gx, gy);
    return {gx, gy};
}

void FrontierDetector::detectNewFrontierCells() {
    std::queue<cell2d_t> bfs_queue;

    // Initialize bfs_queue.
    if (this->frontier_cells_.isEmpty()) {
        // When exploration has just started, there are no frontier cells stored.
        // Start BFS for new frontiers from robot's initial position.
        bfs_queue.push(getRobotMapPosition());
    } else {
        // Start BFS for new frontiers from past frontier cells in the active area.
        // The idea is that explored free cells will stay free and never be a frontier.
        // It's only the unexplored free cells (which happens when robot observes beyond past frontier cells) that can be frontiers.
        // Hence, start exploring new free cells around the past frontier cells, and detect potential new frontier cells.

        // Perform range query for frontier cells in the active area.
        int xmin = this->map_->active_area[0];
        int ymin = this->map_->active_area[1];
        int xmax = this->map_->active_area[2];
        int ymax = this->map_->active_area[3];
        std::list<cell2d_t*> frontier_cells_in_active_area;
        this->frontier_cells_.rangeSearch(xmin, ymin, xmax, ymax, &frontier_cells_in_active_area);

        // Append them to the BFS queue.
        for (const auto& frontier_cell : frontier_cells_in_active_area) {
            cell2d_t cell = *frontier_cell;  // create a copy to push to bfs queue.
            bfs_queue.push(cell);
        }
    }

    // Perform BFS on free cells in the active area to detect new frontier cells.
    while (!bfs_queue.empty()) {
        cell2d_t cell = bfs_queue.front();
        bfs_queue.pop();

        markCellVisited(cell);

        if (isCellFree(cell)) {
            if (isCellFrontier(cell)) {
                // This cell is a frontier cell, so insert it.
                // The insert operation already checks for duplicate before actually inserting
                // (which can happen if cell was a frontier cell before, and robot didn't observe it in this iteration).
                this->frontier_cells_.insert(cell);
            } else if (wasCellFrontier(cell)) {
                // This cell was a frontier cell, but after observations, it becomes a free cell.
                // Remove it from the list of frontier cells
                cell2d_t* corresponding_cell = this->frontier_cells_.pointSearch(cell);
                this->frontier_cells_.remove(corresponding_cell);
            }

            // Get adjacent cells and push those unvisited into the queue.
            std::list<cell2d_t> adj_cells;
            getFourAdjacentCells(cell, adj_cells);
            for (auto& adj_cell : adj_cells) {
                if (!isCellVisited(adj_cell)) {
                    bfs_queue.push(adj_cell);
                }
            }
        }
    }

    // Reset active area, to indicate that frontier detection has finished processing
    // the area with potential new frontier cells.
    resetActiveArea();
}

void FrontierDetector::getFrontiers() {
    // Create a set(k) where key is frontier cell. The set will contain explored frontier cells.
    std::unordered_set<cell2d_t, hash_cell2d_t> explored_set;

    // Iterate through all frontier cells.
    RTree_t::Iterator it;
    RTree_t* tree = this->frontier_cells_.getTree();
    for (tree->GetFirst(it); !tree->IsNull(it); tree->GetNext(it)) {
        cell2d_t frontier_cell = *(tree->GetAt(it));

        // If not visited, create a new frontier,
        // and start BFS from this frontier cell.
        // The queue will only be inserted with frontier cells.
        if (explored_set.find(frontier_cell) == explored_set.end()) {
            Frontier frontier;
            std::queue<cell2d_t> bfs_queue;
            bfs_queue.emplace(frontier_cell);
            explored_set.insert(frontier_cell);  // mark as visited
            frontier.add(frontier_cell);         // store this cell as part of the frontier.

            // Start BFS
            while (!bfs_queue.empty()) {
                cell2d_t cell = bfs_queue.front();
                bfs_queue.pop();

                std::list<cell2d_t> adj_cells;
                getEightAdjacentCells(cell, adj_cells);
                for (auto& adj_cell : adj_cells) {
                    if (isCellFrontier(adj_cell)) {
                        explored_set.insert(frontier_cell);  // mark as visited
                        frontier.add(frontier_cell);         // store this cell as part of the frontier.
                    }
                }
            }

            // End of BFS. Append this frontier to the list of frontiers
            this->frontiers.emplace_back(frontier);
        }
    }
}

void FrontierDetector::markCellVisited(cell2d_t& cell) {
    int cell_idx = this->map_->getIndex(cell.x, cell.y);
    this->map_->map_data[cell_idx].visited_state = true;
}

bool FrontierDetector::isCellFree(cell2d_t& cell) {
    int cell_idx = this->map_->getIndex(cell.x, cell.y);
    return this->map_->map_data[cell_idx].occupancy_state == FREE_SPACE;
}

bool FrontierDetector::isCellUnknown(cell2d_t& cell) {
    int cell_idx = this->map_->getIndex(cell.x, cell.y);
    return this->map_->map_data[cell_idx].occupancy_state == UNKNOWN_SPACE;
}

bool FrontierDetector::isCellVisited(cell2d_t& cell) {
    int cell_idx = this->map_->getIndex(cell.x, cell.y);
    return this->map_->map_data[cell_idx].visited_state;
}

bool FrontierDetector::isCellFrontier(cell2d_t& cell) {
    if (isCellFree(cell)) {
        std::list<cell2d_t> adj_cells;
        getFourAdjacentCells(cell, adj_cells);
        for (auto& adj_cell : adj_cells) {
            if (isCellUnknown(adj_cell)) {
                return true;
            }
        }
    }
    return false;
}

bool FrontierDetector::wasCellFrontier(cell2d_t& cell) {
    if (this->frontier_cells_.pointSearch(cell) == nullptr) return false;
    return true;
}

void FrontierDetector::getFourAdjacentCells(cell2d_t& cell, std::list<cell2d_t>& adj_cells) {
    cell2d_t top_cell = {cell.x, cell.y + 1};
    cell2d_t bottom_cell = {cell.x, cell.y - 1};
    cell2d_t left_cell = {cell.x - 1, cell.y};
    cell2d_t right_cell = {cell.x + 1, cell.y};
    adj_cells.push_back(top_cell);
    adj_cells.push_back(bottom_cell);
    adj_cells.push_back(left_cell);
    adj_cells.push_back(right_cell);
}

void FrontierDetector::getEightAdjacentCells(cell2d_t& cell, std::list<cell2d_t>& adj_cells) {
    getFourAdjacentCells(cell, adj_cells);
    cell2d_t top_left_cell = {cell.x - 1, cell.y + 1};
    cell2d_t top_right_cell = {cell.x + 1, cell.y + 1};
    cell2d_t bottom_left_cell = {cell.x - 1, cell.y - 1};
    cell2d_t bottom_right_cell = {cell.x + 1, cell.y - 1};
    adj_cells.push_back(top_left_cell);
    adj_cells.push_back(top_right_cell);
    adj_cells.push_back(bottom_left_cell);
    adj_cells.push_back(bottom_right_cell);
}
