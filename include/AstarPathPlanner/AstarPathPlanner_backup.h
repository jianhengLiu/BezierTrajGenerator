//
// Created by chrisliu on 2020/9/9.
//

#ifndef ASTART_PATH_PLANNER_H
#define ASTART_PATH_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include "MapManager/MapManager.h"

class AstarPathPlanner
{
    ros::NodeHandle nh;

    ros::Subscriber sub_map,sub_goal;

    ros::Publisher _wp_occ_vis_pub,_grid_path_vis_pub,_visited_nodes_vis_pub;
private:
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

    void callback_map(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
    {
        _MapManager.initGridMap(map_msg);

        _MapManager.visOstacle();
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
        visGridPath(grid_path,false);
        visVisitedNode(visited_nodes);

        //Reset map for next call
        _MapManager.resetUsedGrids();
    }

protected:
    MapManager _MapManager;

    Eigen::Vector3i goalIdx;

    GridNodePtr terminatePtr;
    std::multimap<double, GridNodePtr> openSet;

    double getHeu(GridNodePtr node1, GridNodePtr node2);
    void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);

public:
    AstarPathPlanner(){
        _wp_occ_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_obstacle", 1);
        _grid_path_vis_pub = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
        _visited_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);

        sub_map = nh.subscribe("/map",10,&AstarPathPlanner::callback_map,this);
        sub_goal = nh.subscribe("/move_base_simple/goal",1,&AstarPathPlanner::callback_goal,this);
    };
    ~AstarPathPlanner(){};
    void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    void resetGrid(GridNodePtr ptr);
    void resetUsedGrids();

    std::vector<Eigen::Vector3d> getPath();
    std::vector<Eigen::Vector3d> getVisitedNodes();
};

#endif //PATH_PLANNING_MAPMANAGER_H
