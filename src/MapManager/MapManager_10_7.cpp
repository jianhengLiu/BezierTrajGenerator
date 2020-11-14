//
// Created by chrisliu on 2020/9/9.
//

#include "MapManager/MapManager.h"

using namespace std;
using namespace Eigen;

void MapManager::initMap(nav_msgs::OccupancyGrid::ConstPtr map_msg)
{
    if(is_Map)
    {
        return;
    }
    resolution = map_msg->info.resolution;//    [m/cell]
    inv_resolution = 1.0 / resolution;

    GLX_SIZE = map_msg->info.width;
    GLY_SIZE = map_msg->info.height;
    GLZ_SIZE = 1;
    GLXY_SIZE = GLX_SIZE*GLY_SIZE;
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    gl_xl = -(double)GLX_SIZE/2*resolution;
    gl_yl = -(double)GLY_SIZE/2*resolution;
    gl_zl = 0;

    gl_xu = origin.position.x;
    gl_yu = origin.position.y;
    gl_zu = origin.position.z;

    origin = map_msg->info.origin;

    int inflation_cells = _inflation_dist*inv_resolution;
    data.clear();
    data.assign(GLXYZ_SIZE,-1);
    for(int idx = 0;idx<map_msg->data.size();++idx)
    {
        {
            if(map_msg->data[idx]>0)
            {
                for(int x=-inflation_cells;x<=inflation_cells;x++)
                {
                    for(int y=-inflation_cells;y<=inflation_cells;y++)
                    {
                        int idx_temp = idx+x+y*GLX_SIZE;
                        if(idx_temp>=0)
                        {
                            data[idx_temp] = 1;
                        }
                    }
                }
            }
            else if(data[idx] != 1)
            {
                data[idx] = map_msg->data[idx];
            }
        }

    }

    is_Map = true;
}

visualization_msgs::Marker MapManager::get_visOstacle() {
    visualization_msgs::Marker obs_list;
    int id = 0;
    obs_list.id = id;
    obs_list.header.frame_id = "map";
    obs_list.header.stamp = ros::Time::now();
    obs_list.ns = "wp_path";
    obs_list.action = visualization_msgs::Marker::ADD;
    obs_list.pose.orientation.w = 1.0;
    obs_list.pose.orientation.x = 0.0;
    obs_list.pose.orientation.y = 0.0;
    obs_list.pose.orientation.z = 0.0;

    obs_list.type = visualization_msgs::Marker::POINTS;

    obs_list.scale.x = 0.05;
    obs_list.scale.y = 0.05;
    obs_list.scale.z = 0.05;
    obs_list.color.a = 0.5;// 透明度

    obs_list.color.r = 0.0;
    obs_list.color.g = 1;
    obs_list.color.b = 0.0;

    obs_list.points.clear();

    geometry_msgs::Point pt;
    for(int idx = 0;idx<data.size();++idx)
    {
        if(data[idx]>0)
        {
            pt.x = (idx%GLX_SIZE)*resolution+origin.position.x;
            pt.y = (idx/GLX_SIZE)*resolution+origin.position.y;
            pt.z = 1;
            obs_list.points.push_back(pt);
        }
    }
    return obs_list;
}

void MapManager::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;

    int idx_x = static_cast<int>((coord_x - origin.position.x) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - origin.position.y) * inv_resolution);
    int idx_z = static_cast<int>((coord_z - origin.position.z) * inv_resolution);

    data[idx_x + idx_y * GLX_SIZE + idx_z*GLXY_SIZE] = 1;
}

Vector3d MapManager::gridIndex2coord(const Vector3i &index)
{
    Vector3d pt;
    pt(0) = ((double) index(0)) * resolution + origin.position.x;
    pt(1) = ((double) index(1)) * resolution + origin.position.y;
    pt(2) = ((double) index(2)) * resolution + gl_zl;

    return pt;
}

Vector3i MapManager::coord2gridIndex(const Vector3d &pt)
{
    Vector3i idx;
    idx << min(max(int((pt(0) - origin.position.x) * inv_resolution), 0), GLX_SIZE - 1),
            min(max(int((pt(1) - origin.position.y) * inv_resolution), 0), GLY_SIZE - 1),
            min(max(int((pt(2) - origin.position.z) * inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

Eigen::Vector3d MapManager::coordRounding(const Eigen::Vector3d &coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

bool MapManager::isOccupied(const Eigen::Vector3i &index)
{
    return isOccupied(index(0), index(1), index(2));
}

bool MapManager::isFree(const Eigen::Vector3i &index)
{
    return isFree(index(0), index(1), index(2));
}

inline bool MapManager::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x + idx_y * GLX_SIZE + idx_z*GLXY_SIZE] == 1));
}

inline bool MapManager::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x + idx_y * GLX_SIZE + idx_z*GLXY_SIZE] < 1));
}