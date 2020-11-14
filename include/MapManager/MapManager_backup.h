//
// Created by chrisliu on 2020/9/9.
//

#ifndef PATH_PLANNING_MAPMANAGER_H
#define PATH_PLANNING_MAPMANAGER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "node.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

class MapManager
{
public:
    double resolution;

    void visOstacle();

protected:
    bool is_Map = false;

    std::vector<int8_t> data;//   地图数据(-1 = free, 0 = unknown, +1 = occ)

    /**
     * 大写的都是矩阵相关的（单位）
     * 小写的是真实地图相关（m）
     */
    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
    int GLXYZ_SIZE,GLXY_SIZE, GLYZ_SIZE;

    double inv_resolution;
    double gl_xl, gl_yl, gl_zl;//   地图的最小（lower）位置（m)
    double gl_xu, gl_yu, gl_zu;//   地图的最大（upper）位置（m)

    geometry_msgs::Pose origin;

    bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isOccupied(const Eigen::Vector3i & index) const;
    bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isFree(const Eigen::Vector3i & index) const;

    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

public:
    MapManager(){
    };
    ~MapManager(){};
    void resetGrid(GridNodePtr ptr);
    void resetUsedGrids();

    void initGridMap(nav_msgs::OccupancyGrid::ConstPtr map_msg);
    void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
    void setObs(const double coord_x, const double coord_y, const double coord_z);

    Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
};

#endif //PATH_PLANNING_MAPMANAGER_H
