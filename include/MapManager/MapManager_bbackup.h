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
    ros::NodeHandle nh;

    ros::Subscriber sub_map,sub_goal;

    ros::Publisher _wp_occ_vis_pub,_grid_path_vis_pub,_visited_nodes_vis_pub;
public:
    double resolution;
    void visGridPath( std::vector<Eigen::Vector3d> nodes, bool is_use_jps )
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


        node_vis.scale.x = resolution;
        node_vis.scale.y = resolution;
        node_vis.scale.z = resolution;

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

    void visVisitedNode( std::vector<Eigen::Vector3d> nodes )
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

        node_vis.scale.x = resolution;
        node_vis.scale.y = resolution;
        node_vis.scale.z = resolution;

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

    void visOstacle();

    void callback_map(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
    {
        initGridMap(map_msg);

        visOstacle();
    }

    void callback_goal(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
    {
        Eigen::Vector3d start_pt(0.6,-0.7,0);
        Eigen::Vector3d target_pt(goal_msg->pose.position.x,goal_msg->pose.position.y,goal_msg->pose.position.z);

        AstarGraphSearch(start_pt,target_pt);

        //Retrieve the path
        auto grid_path     = getPath();
        auto visited_nodes = getVisitedNodes();

        //Visualize the result
        visGridPath (grid_path, false);
        visVisitedNode(visited_nodes);

        //Reset map for next call
        resetUsedGrids();
    }

protected:
    bool is_Map = false;

    std::vector<int8_t> data;//   地图数据(-1 = free, 0 = unknown, +1 = occ)
    GridNodePtr *** GridNodeMap;
    Eigen::Vector3i goalIdx;

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

    GridNodePtr terminatePtr;
    std::multimap<double, GridNodePtr> openSet;

    double getHeu(GridNodePtr node1, GridNodePtr node2);
    void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);

    bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isOccupied(const Eigen::Vector3i & index) const;
    bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isFree(const Eigen::Vector3i & index) const;

    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

public:
    MapManager(){
        _wp_occ_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_obstacle", 1);
        _grid_path_vis_pub = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
        _visited_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);

        sub_map = nh.subscribe("/map",10,&MapManager::callback_map,this);
        sub_goal = nh.subscribe("/move_base_simple/goal",1,&MapManager::callback_goal,this);
    };
    ~MapManager(){};
    void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    void resetGrid(GridNodePtr ptr);
    void resetUsedGrids();

    void initGridMap(nav_msgs::OccupancyGrid::ConstPtr map_msg);
    void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
    void setObs(const double coord_x, const double coord_y, const double coord_z);

    Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
    std::vector<Eigen::Vector3d> getPath();
    std::vector<Eigen::Vector3d> getVisitedNodes();
};

#endif //PATH_PLANNING_MAPMANAGER_H
