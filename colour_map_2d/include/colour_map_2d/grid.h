#include "ros/ros.h"
#include "colour_map_2d/grid_cell.h"
#include "colour_map_2d/map_transform.h"

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <vector>
#include <mutex>

#ifndef GRID_H
#define GRID_H

namespace ColourMap2D{

class Grid
{
private:
    ////MEMBERS
    std::vector<GridCell> grid_cells_;
    nav_msgs::MapMetaData grid_info_;
    mutable std::mutex mutex_;

    ////FILTER MEMBERS
    double hit_prob_;
    double miss_prob_;
    double min_prob_;
    double prob_thresh_;

    ////CELL STATE MEMBERS
    int cell_obstacle_;
    int cell_occupied_;
    int cell_empty_;
    int cell_unknown_;
    int frame_;

    ////SPAWN MEMBERS
    double spawn_noise_;
    int spawn_rate_;
    int spawn_size_;

    ////BOOL MEMBERS
    bool initialized_;
    
    ////PRIVATE METHODS
    double findMaxProb(int spaces, int index);
public:
    ////CONSTRUCTORS
    Grid();
    Grid(double hit_prob, double miss_prob, double min_prob, double prob_thresh,
         int cell_occupied, int cell_obstacle, int cell_empty, int cell_unknown,
         int spawn_rate, double spawn_noise, int spawn_size, int frame);

    ////COPYASSIGNMENT
    Grid& operator = (const Grid& other);

    ////GETTERS
    bool initialized() const;

    ////METHODS
    void processOGMap(const nav_msgs::OccupancyGrid og_map);
    void initializeGrid(nav_msgs::MapMetaData og_map_data);
    void processPoints(std::vector<ColourPoint> &points);
    void updateImage(cv::Mat &image);
    void initializeMapImage(cv::Mat &image);
    void print();
};

}
#endif // GRID_H
