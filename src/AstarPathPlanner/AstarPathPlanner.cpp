//
// Created by chrisliu on 2020/9/9.
//

#include "AstarPathPlanner/AstarPathPlanner.h"

using namespace std;
using namespace Eigen;

void AstarPathPlanner::initGridMap()
{
    if(!_MapManager.is_Map)
    {
        return;
    }
//    创建地图空间
    GridNodeMap = new GridNodePtr **[_MapManager.GLX_SIZE];
    for (int i = 0; i < _MapManager.GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr *[_MapManager.GLY_SIZE];
        for (int j = 0; j < _MapManager.GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr[_MapManager.GLZ_SIZE];
            for (int k = 0; k < _MapManager.GLZ_SIZE; k++)
            {
                Vector3i tmpIdx(i, j, k);
                Vector3d pos = _MapManager.gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathPlanner::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathPlanner::resetUsedGrids()
{
    for (int i = 0; i < _MapManager.GLX_SIZE; i++)
        for (int j = 0; j < _MapManager.GLY_SIZE; j++)
            for (int k = 0; k < _MapManager.GLZ_SIZE; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

vector<Vector3d> AstarPathPlanner::getVisitedNodes()
{
    vector<Vector3d> visited_nodes;
    for (int i = 0; i < _MapManager.GLX_SIZE; i++)
        for (int j = 0; j < _MapManager.GLY_SIZE; j++)
            for (int k = 0; k < _MapManager.GLZ_SIZE; k++)
            {
                if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
//                if (GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", (int)visited_nodes.size());
    return visited_nodes;
}

void AstarPathPlanner::visVisitedNode( std::vector<Eigen::Vector3d> nodes )
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _MapManager.resolution;
    node_vis.scale.y = _MapManager.resolution;
    node_vis.scale.z = _MapManager.resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Eigen::Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}

void AstarPathPlanner::visGridPath( std::vector<Eigen::Vector3d> nodes, bool is_use_jps )
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();

    if(is_use_jps)
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    if(is_use_jps){
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else{
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }


    node_vis.scale.x = _MapManager.resolution;
    node_vis.scale.y = _MapManager.resolution;
    node_vis.scale.z = _MapManager.resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Eigen::Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

inline void AstarPathPlanner::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets,
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

                if (neighborIdx.x() < 0 || neighborIdx.x() >= _MapManager.GLX_SIZE
                    || neighborIdx.y() < 0 || neighborIdx.y() >= _MapManager.GLY_SIZE
                    || neighborIdx.z() < 0 || neighborIdx.z() >= _MapManager.GLZ_SIZE)
                {
                    continue;
                }

                neighborPtrSets.push_back(GridNodeMap[neighborIdx.x()][neighborIdx.y()][neighborIdx.z()]);
                edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
            }
        }
    }
}

double AstarPathPlanner::getHeu(GridNodePtr node1, GridNodePtr node2)
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
    /**
     * Manhattan
     */
//    Eigen::Vector3d vector_node2goal = node2->index-node1->index;
//    h = vector_node2goal.x() + vector_node2goal.y() + vector_node2goal.z());

    /**
     * Euclidean
     */
//    h = (node2->index - node1->index).norm();

    /**
     * Diagonal
     * 假设x>y>z
     * h = (x-z-(y-z))+sqrt(2)*(y-z)+sqrt(3)*z
     */
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

void AstarPathPlanner::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();

    //index of start_point and end_point
    Vector3i start_idx = _MapManager.coord2gridIndex(start_pt);
    Vector3i end_idx = _MapManager.coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = _MapManager.gridIndex2coord(start_idx);
    end_pt = _MapManager.gridIndex2coord(end_idx);

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
                     currentPtr->gScore * _MapManager.resolution);
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
            if (_MapManager.isOccupied(neighborPtr->index))
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

vector<Vector3d> AstarPathPlanner::getPath()
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