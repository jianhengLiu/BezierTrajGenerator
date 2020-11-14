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

    void setObs(const double coord_x, const double coord_y, const double coord_z);



public:
    MapManager(double inflation_dist = 0){
        _inflation_dist = inflation_dist;
    };
    ~MapManager(){};

    bool is_Map = false;
    double resolution;

    geometry_msgs::Pose origin;
    /**
     * GLX_SIZE,GLY_SIZE也同时表示了地图的宽（width）和高（height）
     */
    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;//   地图的最小（lower）位置（单位栅格)
    int GLXYZ_SIZE,GLXY_SIZE, GLYZ_SIZE;//   地图的最大（upper）位置（单位栅格)

    void initMap(nav_msgs::OccupancyGrid::ConstPtr map_msg);

    bool isOccupied(const Eigen::Vector3i & index);
    bool isFree(const Eigen::Vector3i & index);

    bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;

    // 由栅格坐标返回地图坐标
    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
    // 由地图坐标返回栅格坐标
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);
    Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);

    visualization_msgs::Marker get_visOstacle();
};

#endif //MAPMANAGER_H