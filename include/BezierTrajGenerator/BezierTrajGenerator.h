//
// Created by chrisliu on 2020/10/8.
//

#ifndef TRAJECTORY_PLANNING_BEZIERTRAJGENERATOR_H
#define TRAJECTORY_PLANNING_BEZIERTRAJGENERATOR_H

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <math.h>

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "BezierTrajGenerator/Cube.h"

#include "OsqpEigen/OsqpEigen.h"

using namespace std;
using namespace Eigen;

class BezierTrajGenerator
{

private:
    const int n_order = 7;   // 八个控制点,对应八个贝赛尔曲线系数,对应七阶多项式

    const int n_coeff = 7 + 1;   // 八个控制点,对应八个贝赛尔曲线系数

    int n_all_coeff;

    int n_seg;

    vector<Cube> _corridor;

    double _max_t_seg = 1;

    Eigen::Vector3d _start_pt;
    Eigen::Vector3d _end_pt;

//    vector<Eigen::Vector3d> _Path;

    double _Vel_max, _Acc_max;

    vector<Eigen::VectorXd> _BezierSolution;

    void timeAllocation(); //时间分配函数

    /**
     *
     * @param k : 第几段轨迹
     * @param t_seg 对应轨迹段里的时间点
     * @param order :p=0,v=1,a=2,...
     * @return 轨迹段里的时间点对应order的x,y,z
     */
    Eigen::Vector3d getPolyStates(int k, double t_seg, int order);

    Eigen::MatrixXd getAeq();

    Eigen::VectorXd getbeq(int axis);

    Eigen::MatrixXd getAieq();

    Eigen::MatrixXd getAieq_plus();

    Eigen::VectorXd getbieq_plus(int axis);

    Eigen::VectorXd getbieq_minus(int axis);

    Eigen::MatrixXd getM();

    Eigen::MatrixXd getQ_k(double t_k);

    Eigen::MatrixXd getQ();

public:

    bool isTraj = false;

    double _totalTime = 0;

    Eigen::MatrixXd _polyCoeff;
    Eigen::VectorXd _polyTime;

    BezierTrajGenerator(double Vel_max,double Acc_max){
        _Vel_max = Vel_max;
        _Acc_max = Acc_max;

    };

    ~BezierTrajGenerator(){};

    bool TrajGeneration(Eigen::Vector3d start_pt,Eigen::Vector3d end_pt,vector<Cube> corridor);
    bool TestTrajGeneration();
    /**
     *
     * @param t 此处t是对应全局时间，即轨迹开始到t的时间
     * @param order :p=0,v=1,a=2,...
     * @return 包含对应order的x,y,z信息,组成如下
     * p:x,y,z
     * v:x,y,z
     * a:x,y,z
     * ...
     */
    Eigen::Vector3d getTrajectoryStates(double time_from_start, int order);

    visualization_msgs::Marker visBezierTraj();//返回用于可视化的路径Marker
    visualization_msgs::MarkerArray visBezierPt();


};

#endif //TRAJECTORY_PLANNING_BEZIERTRAJGENERATOR_H
