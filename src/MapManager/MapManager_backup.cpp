//
// Created by chrisliu on 2020/9/9.
//

#include "MapManager/MapManager.h"

using namespace std;
using namespace Eigen;

void MapManager::initGridMap(nav_msgs::OccupancyGrid::ConstPtr map_msg)
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

    gl_xl = -(double)GLX_SIZE/2*resolution;// global_xyz_l(0);
    gl_yl = -(double)GLY_SIZE/2*resolution;//global_xyz_l(1);
    gl_zl = 0;

    gl_xu = origin.position.x;//global_xyz_u(0);
    gl_yu = origin.position.y;//global_xyz_u(1);
    gl_zu = origin.position.z;//global_xyz_u(2);

    origin = map_msg->info.origin;

//    创建地图空间
    GridNodeMap = new GridNodePtr **[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                Vector3i tmpIdx(i, j, k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }

    double inflation_dist = 0.5;
    int inflation_cells = inflation_dist*inv_resolution;
    data.clear();
    data.assign(GLXYZ_SIZE,-1);
    for(int idx = 0;idx<map_msg->data.size();++idx)
    {
//        if(data[idx]==-1)
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

void MapManager::visOstacle() {
    visualization_msgs::Marker line_list;
    int id = 0;
    line_list.id = id;
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "wp_path";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.pose.orientation.x = 0.0;
    line_list.pose.orientation.y = 0.0;
    line_list.pose.orientation.z = 0.0;

    line_list.type = visualization_msgs::Marker::POINTS;

    line_list.scale.x = 0.05;
    line_list.scale.y = 0.05;
    line_list.scale.z = 0.05;
    line_list.color.a = 0.5;// 透明度

    line_list.color.r = 0.0;
    line_list.color.g = 1;
    line_list.color.b = 0.0;

    line_list.points.clear();

    geometry_msgs::Point pt;
    for(int idx = 0;idx<data.size();++idx)
    {
        if(data[idx]>0)
        {
            pt.x = (idx%GLX_SIZE)*resolution+origin.position.x;
            pt.y = (idx/GLX_SIZE)*resolution+origin.position.y;
            pt.z = 1;
            line_list.points.push_back(pt);
        }
    }
    _wp_occ_vis_pub.publish(line_list);
}

void MapManager::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id,
                             int max_y_id, int max_z_id)
{
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    data.clear();
    data.resize(GLXYZ_SIZE);
    memset(data.data(),-1,data.size());
//    data = new int8_t[GLXYZ_SIZE];
//    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

//    创建地图空间
    GridNodeMap = new GridNodePtr **[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                Vector3i tmpIdx(i, j, k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void MapManager::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void MapManager::resetUsedGrids()
{
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void MapManager::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;

    int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> MapManager::getVisitedNodes()
{
    vector<Vector3d> visited_nodes;
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
//                if (GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", (int)visited_nodes.size());
    return visited_nodes;
}

Vector3d MapManager::gridIndex2coord(const Vector3i &index)
{
    Vector3d pt;

//    pt(0) = ((double) index(0) + 0.5) * resolution + gl_xl;
//    pt(1) = ((double) index(1) + 0.5) * resolution + gl_yl;
//    pt(2) = ((double) index(2) + 0.5) * resolution + gl_zl;

    pt(0) = ((double) index(0)) * resolution + origin.position.x;
    pt(1) = ((double) index(1)) * resolution + origin.position.y;
    pt(2) = ((double) index(2)) * resolution + gl_zl;

//    pt(0) = (idx%GLX_SIZE)*resolution+origin.position.x;
//    pt(1) = (idx/GLX_SIZE)*resolution+origin.position.y;
//    pt(2) = ((double) index(2)) * resolution + gl_zl;

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

inline bool MapManager::isOccupied(const Eigen::Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool MapManager::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool MapManager::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x + idx_y * GLX_SIZE + idx_z*GLXY_SIZE] == 1));
//            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool MapManager::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x + idx_y * GLX_SIZE + idx_z*GLXY_SIZE] < 1));
//            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void MapManager::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets,
                                     vector<double> &edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself
    please write your code below
    *
    *
    */

    Eigen::Vector3i neighborIdx;
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dz = -1; dz <= 1; ++dz)
            {
                if (dx == 0 && dy == 0 && dz == 0)
                {
                    continue;
                }

                neighborIdx.x() = currentPtr->index.x() + dx;
                neighborIdx.y() = currentPtr->index.y() + dy;
                neighborIdx.z() = currentPtr->index.z() + dz;

                if (neighborIdx.x() < 0 || neighborIdx.x() >= GLX_SIZE
                    || neighborIdx.y() < 0 || neighborIdx.y() >= GLY_SIZE
                    || neighborIdx.z() < 0 || neighborIdx.z() >= GLZ_SIZE)
                {
                    continue;
                }

                neighborPtrSets.push_back(GridNodeMap[neighborIdx.x()][neighborIdx.y()][neighborIdx.z()]);
                edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
            }
        }
    }
}

double MapManager::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /*
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */

    double h = 0;
    //    Manhattan
//    Eigen::Vector3d vector_node2goal = node2->index-node1->index;
//    h = vector_node2goal.x() + vector_node2goal.y() + vector_node2goal.z());

//    Euclidean
//    h = (node2->index - node1->index).norm();

//    Diagonal
//  假设x>y>z
//  h = (x-z-(y-z))+sqrt(2)*(y-z)+sqrt(3)*z
    Vector3i delta = node2->index - node1->index;
    int dx = abs(delta.x());
    int dy = abs(delta.y());
    int dz = abs(delta.z());

    int minDelta = min(min(dx,dy),dz);

    dx -=minDelta;
    dy -=minDelta;
    dz -=minDelta;

    if(dx==0)
    {
        h = 1.0*sqrt(3.0)*minDelta + sqrt(2.0)*min(dy,dz)+1.0*abs(dy-dz);
    }
    if(dy==0)
    {
        h = 1.0*sqrt(3.0)*minDelta + sqrt(2.0)*min(dx,dz)+1.0*abs(dx-dz);
    }
    if(dz==0)
    {
        h = 1.0*sqrt(3.0)*minDelta + sqrt(2.0)*min(dx,dy)+1.0*abs(dx-dy);
    }

    ///////////////////////////////
    //add tie_breaker
    double tie_breaker = 1.0 + 1.0 / 10000;
    return h * tie_breaker;


}

void MapManager::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr = new GridNode(end_idx, end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr->id = 1;
    startPtr->coord = start_pt;
    openSet.insert(make_pair(startPtr->fScore, startPtr));
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while (!openSet.empty())
    {
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below

        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */

//        获取fn最低的点，作为当前拓展点
        currentPtr = openSet.begin()->second;

//        将拓展点从openset中移除
        openSet.erase(openSet.begin());
//        将当前点标记为在closedset里
        currentPtr->id = -1;

        // if the current node is the goal
        if (currentPtr->index == goalIdx)
        {
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0,
                     currentPtr->gScore * resolution);
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets,
                     edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *
        */
        for (int i = 0; i < (int) neighborPtrSets.size(); i++)
        {
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below

            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : in closed set
            neighborPtrSets[i]->id = 1 : in open set
            *
            */
            neighborPtr = neighborPtrSets[i];
//            判断该点是否有障碍物
            if (isOccupied(neighborPtr->index))
            {
                continue;
            }
            if (neighborPtr->id == 0)
            { //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *
                */
                neighborPtr->id = 1;//标记在openset中
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                neighborPtr->cameFrom = currentPtr;
//              multimap的insert() 会返回一个指向插入元素的迭代器，等于知道了新加的点在openset的位置
                neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));

                continue;
            } else if (neighborPtr->id == 1)
            { //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *
                */
                if (neighborPtr->gScore > currentPtr->gScore + edgeCostSets[i])
                {
                    neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                    neighborPtr->cameFrom = currentPtr;
//                    删除未更新的旧点，因为multimap允许有相同键的元素序列
                    openSet.erase(neighborPtr->nodeMapIt);
//              multimap的insert() 会返回一个指向插入元素的迭代器，等于知道了新加的点在openset的位置
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                }

                continue;
            } else
            {//this node is in closed set
                /*
                *
                please write your code below
                *
                */
                continue;
            }
        }
    }

    //if search fails
    ros::Time time_2 = ros::Time::now();
    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec());
}


vector<Vector3d> MapManager::getPath()
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *
    */
    GridNodePtr currentPtr = terminatePtr;
    while (currentPtr->cameFrom != NULL)
    {
        gridPath.push_back(currentPtr);
        currentPtr = currentPtr->cameFrom;
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);

    reverse(path.begin(), path.end());

    return path;
}