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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "node.h"
#include "MapManager/MapManager.h"

class AstarPathPlanner
{
    ros::NodeHandle nh;

    ros::Subscriber sub_map,sub_goal,sub_odom;

    ros::Publisher _obs_vis_pub,_grid_path_vis_pub,_visited_nodes_vis_pub;
public:
    void visGridPath( std::vector<Eigen::Vector3d> nodes, bool is_use_jps );

    void visVisitedNode( std::vector<Eigen::Vector3d> nodes );

    void callback_map(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
    {
        _MapManager.initMap(map_msg);
        _obs_vis_pub.publish(_MapManager.get_visOstacle());

        initGridMap();
    }

    void callback_odom(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
    {
        currentPos.x() = pose_msg->pose.pose.position.x;
        currentPos.y() = pose_msg->pose.pose.position.y;
        currentPos.z() = pose_msg->pose.pose.position.z;
    }

    void callback_goal(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
    {
        //Reset map
        resetUsedGrids();

        Eigen::Vector3d start_pt = currentPos;
        Eigen::Vector3d target_pt(goal_msg->pose.position.x,goal_msg->pose.position.y,goal_msg->pose.position.z);

        if((target_pt - start_pt).norm()<0.05)
        {
            return;
        }
        else
        {
            AstarGraphSearch(start_pt,target_pt);
        }

        //Retrieve the path
        auto grid_path     = getPath();
        auto visited_nodes = getVisitedNodes();

        //Visualize the result
        visGridPath (grid_path, false);
        visVisitedNode(visited_nodes);

        is_Path = true;
    }

protected:

    GridNodePtr *** GridNodeMap;
    Eigen::Vector3d currentPos = Eigen::Vector3d(0,0,0);
    Eigen::Vector3i goalIdx;

    GridNodePtr terminatePtr;
    std::multimap<double, GridNodePtr> openSet;

    void initGridMap();

    double getHeu(GridNodePtr node1, GridNodePtr node2);
    void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);
    void resetGrid(GridNodePtr ptr);

public:
    MapManager _MapManager;
    bool is_Path = false;

    AstarPathPlanner(){
        _MapManager = MapManager(0.3);

        _obs_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_obstacle", 1);
        _grid_path_vis_pub = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
        _visited_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);

        sub_map = nh.subscribe("/map",1,&AstarPathPlanner::callback_map,this);
        sub_goal = nh.subscribe("/move_base_simple/goal",1,&AstarPathPlanner::callback_goal,this);
        sub_odom = nh.subscribe("/amcl_pose",1,&AstarPathPlanner::callback_odom,this);
    };
    ~AstarPathPlanner(){};

    void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

    void resetUsedGrids();

    std::vector<Eigen::Vector3d> getPath();
    std::vector<Eigen::Vector3d> getVisitedNodes();
};

#endif //ASTART_PATH_PLANNER_H
