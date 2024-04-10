#pragma once

#include <memory>
#include <mutex>

const char UNKNOWN_SPACE = -1;
const char FREE_SPACE = 0;
const char OCCUPIED_SPACE = 100;

/**
 * Represents the map's data at each grid position.
 */
typedef struct MapNodeData {
    bool visited_state;
    char occupancy_state;
} map_node_t;

/**
 * A class that represents a 2D static grid map.  Its origin (0,0) is at the bottom-left corner and its real world position is represented by origin_wx and origin_wy.
 * The map size in meters is (width, height) and this doesn't change dynamically.
 */
class Map2D {
   public:
    std::shared_ptr<map_node_t[]> map_data;  // Map data.
    unsigned int size_x;                     // [#cells]
    unsigned int size_y;                     // [#cells]
    double resolution;
    double width;
    double height;
    double origin_wx;  // origin is the bottom-left corner of the grid (0,0).  wx means x in world coordinate.
    double origin_wy;

    // A box (xmin,ymin,xmax,ymax) that expresses the area where occupancy state has changed.
    // This will keep becoming bigger as more occupancy grid data is received.
    // When frontier detection runs, it will process this area and reset this value to 0.
    unsigned int active_area[4] = {0, 0, 0, 0};

   public:
    Map2D(double resolution, double width, double height, double robot_wx, double robot_wy);
    /**
     * Map given (gx, gy) map coordintes to (wx, wy) world coordinates.
     */
    void mapToWorld(unsigned int gx, unsigned int gy, double& wx, double& wy) const;

    /**
     * Map given (wx, wy) world coordinates to (gx, gy) map coordintes.
     * Returns true if succesfull. May fail due to out of map boundary.
     */
    bool worldToMap(double wx, double wy, unsigned int& gx, unsigned int& gy) const;

    /**
     * Return the corresponding index in the 1D array map for the 2D grid coordinates (gx, gy).
     */
    unsigned int getIndex(unsigned int gx, unsigned int gy);
};