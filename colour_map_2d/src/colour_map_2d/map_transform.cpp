#include "colour_map_2d/map_transform.h"

namespace ColourMap2D{

geometry_msgs::Pose2D MapTransform::gridtoPose(int col, int row, nav_msgs::MapMetaData map_data)
{
    geometry_msgs::Pose2D pose;
    pose.x = map_data.origin.position.x + col * map_data.resolution;
    pose.y = map_data.origin.position.y + row * map_data.resolution;
    return pose;
}

std::pair<int, int> MapTransform::posetoGrid(geometry_msgs::Pose2D pose, nav_msgs::MapMetaData map_data)
{
    std::pair<int, int> grid_cell;
    int col = round((pose.x - map_data.origin.position.x) / map_data.resolution);
    int row = round((pose.y - map_data.origin.position.y) / map_data.resolution);

    if(col >= map_data.width) col = map_data.width - 1;
    if(col < 0) col = 0;
    if(row >= map_data.height) row = map_data.height -1;
    if(row < 0) row = 0;

    grid_cell.second = row;
    grid_cell.first = col;

    return grid_cell;
}

int MapTransform::posetoIndex(geometry_msgs::Pose2D pose, nav_msgs::MapMetaData map_data)
{
    int col = round((pose.x - map_data.origin.position.x) / map_data.resolution);
    int row = round((pose.y - map_data.origin.position.y) / map_data.resolution);

    if(col >= map_data.width) col = map_data.width - 1;
    if(col < 0) col = 0;
    if(row >= map_data.height) row = map_data.height -1;
    if(row < 0) row = 0;

    int index = row * map_data.width + col;
    return index;

}

geometry_msgs::Pose2D MapTransform::indextoPose(int index, nav_msgs::MapMetaData map_data)
{
    std::pair<int, int> grid_cell = indextoGrid(index, map_data);
    return gridtoPose(grid_cell.first, grid_cell.second, map_data);
}

std::pair<int, int> MapTransform::indextoGrid(int index, nav_msgs::MapMetaData map_data)
{
    std::pair<int, int> grid_cell;

    int col = index % map_data.width;
    int row = index / map_data.width;

    if(col >= map_data.width) col = map_data.width - 1;
    if(col < 0) col = 0;
    if(row >= map_data.height) row = map_data.height -1;
    if(row < 0) row = 0;

    grid_cell.second = row;
    grid_cell.first = col;

    return grid_cell;
}

int MapTransform::gridtoIndex(int col, int row, nav_msgs::MapMetaData map_data)
{
    return row * map_data.width + col;
}

int MapTransform::xytoIndex(double x, double y, nav_msgs::MapMetaData map_data)
{
    int col = round((x - map_data.origin.position.x) / map_data.resolution);
    int row = round((y - map_data.origin.position.y) / map_data.resolution);

    if(col >= map_data.width) col = map_data.width - 1;
    if(col < 0) col = 0;
    if(row >= map_data.height) row = map_data.height -1;
    if(row < 0) row = 0;

    int index = row * map_data.width + col;
    return index;
}

}

