//
// Created by chrisliu on 2020/9/9.
//

#ifndef MAPMANAGER_H
#define MAPMANAGER_H

#include <iostream>
#include <Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

class MapManager
{
private:


protected:
    double _inflation_dist; //  膨胀半径(m)

    std::vector<int8_t> data;//   地图数据(-1 = free, 0 = unknown, +1 = occ)

    double inv_resolution;
    double gl_xl, gl_yl, gl_zl;//   地图的最小（lower）位置（m)
    double gl_xu, gl_yu, gl_zu;//   地图的最大（upper）位置（m)

    geometry_msgs::Pose origin;

    void setObs(const double coord_x, const double coord_y, const double coord_z);

    bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;

public:
    MapManager(double inflation_dist = 0){
        _inflation_dist = inflation_dist;
    };
    ~MapManager(){};

    bool is_Map = false;
    double resolution;
    /**
     * 大写的都是矩阵相关的（单位）
     * 小写的是真实地图相关（m）
     */
    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
    int GLXYZ_SIZE,GLXY_SIZE, GLYZ_SIZE;

    void initMap(nav_msgs::OccupancyGrid::ConstPtr map_msg);

    bool isOccupied(const Eigen::Vector3i & index);
    bool isFree(const Eigen::Vector3i & index);

    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);
    Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);

    visualization_msgs::Marker get_visOstacle();
};

#endif //MAPMANAGER_H
