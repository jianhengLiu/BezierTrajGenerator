#include "BezierTrajGenerator/BezierTrajGenerator.h"

using namespace std;
using namespace Eigen;

double eps(double x)
{
    x = std::fabs(x);
    const auto tol =  std::nextafter(x, x + 1.0);
    const auto res = tol - x;
    return res;
}

bool isSPD(MatrixXd A)
{
    // check if sigma_bar is already SPD
    MatrixXd L(A.llt().matrixL());

    //reassemble sigma bar
    MatrixXd A_test = L * L.transpose();


    if (A.isApprox(A_test))
    {
        // std::cout << "Matrix is SPD" << std::endl;
        return true;
    }

    std::cout << "Matrix is NOT SPD" << std::endl;
    return false;
}

MatrixXd nearestSPD(MatrixXd A)
{
    // symmetrize A into B
    MatrixXd B = 0.5 * (A + A.transpose());

    // Compute the symmetric polar factor of B. Call it H.
    // Clearly H is itself SPD.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(B, Eigen::ComputeFullU |
                                             Eigen::ComputeFullV);

    MatrixXd S = svd.singularValues().asDiagonal();
    MatrixXd V = svd.matrixV();

    MatrixXd H = V * S * V.transpose();

    // get Ahat in the above formula
    MatrixXd A_hat = 0.5 * (B + H);

    // ensure symmetry
    A_hat = 0.5 * (A_hat + A_hat.transpose());

    // test that Ahat is in fact PD. if it is not so, then tweak it just a bit
    bool flag = false;
    int k = 0;

    while(!flag)
    {
        flag = isSPD(A_hat);

        if (!flag)
        {
            // std::cout << "Adjust Ahat" << std::endl;

            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A_hat);
            double mineig = es.eigenvalues().minCoeff();

            MatrixXd I = MatrixXd::Identity(A_hat.rows(), A_hat.cols());

            A_hat = A_hat + (-mineig*(k*k) + eps(mineig))*I;
        }
        k++;
    }
    return A_hat;
    // end while
}

//define factorial function, input i, output i!
double Factorial(int x)
{
    double fac = 1;
    for (int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

double nchoosek(int n,int k)
{
    return Factorial(n)/(Factorial(n-k)*Factorial(k));
}

//bool BezierTrajGenerator::TrajGeneration(Eigen::Vector3d start_pt,Eigen::Vector3d end_pt,vector<Cube> corridor)
//{
//    isTraj= false;
//
//    _corridor = corridor;
//    _start_pt = start_pt;
//    _end_pt = end_pt;
//
//    n_seg = corridor.size();//  有多少个走廊就有多少段轨迹
//    n_all_coeff = n_seg*n_coeff;
//
//    timeAllocation();
//
//    auto M = getM();
//    auto Q = getQ();
//    auto Q_0 = M.transpose()*Q*M;
//
////  将Q_0优化为最近半正定对称矩阵
//    MatrixXd Q_0_nearest = nearestSPD(Q_0);
//
//    // allocate QP problem matrices and vectores
//    Eigen::SparseMatrix<double> hessian; //P or H
//    Eigen::VectorXd gradient;  //f or q
//    Eigen::SparseMatrix<double> linearMatrix;  //A
//    Eigen::VectorXd lowerBound; //l
//    Eigen::VectorXd upperBound; //u
//
//    hessian = Q_0_nearest.sparseView();
//    gradient = Eigen::VectorXd::Zero(Q_0_nearest.rows());
//
//    auto Aeq = getAeq();
//    auto Aieq = getAieq();
//    Eigen::MatrixXd A = MatrixXd::Zero(Aeq.rows()+Aieq.rows(),Aeq.cols());//(Aieq.rows(),Aeq.cols());
//    A<<Aeq,
//            Aieq;
//    int NumberOfVariables = A.cols(); //A矩阵的列数
//    int NumberOfConstraints = A.rows(); //A矩阵的行数
//    linearMatrix = A.sparseView();
//
//
//    // instantiate the solver
//    OsqpEigen::Solver solver;
//
//    // settings
//    solver.settings()->setVerbosity(false);//   设置是否查看迭代过程
////    solver.settings()->setRho(0.5);
//    solver.settings()->setMaxIteraction(1000000);
//
//    // set the initial data of the QP solver
//    //矩阵A为m*n矩阵
//    solver.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
//    solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
//    if(!solver.data()->setHessianMatrix(hessian)) return 1;//设置P矩阵
//    if(!solver.data()->setGradient(gradient)) return 1; //设置q or f矩阵。当没有时设置为全0向量
//    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;//设置线性约束的A矩阵
//
//    _BezierSolution.clear();
//    for(int i = 0;i<2;++i)
//    {
//        solver.clearSolver();
//
//        lowerBound.resize(A.rows());
//        lowerBound<<getbeq(i),
//                Eigen::VectorXd::Constant(Aieq.rows(),-OsqpEigen::INFTY) ;
//
//        upperBound.resize(A.rows());
//        upperBound<<getbeq(i),
//                getbieq(i);
//
//        if(!solver.data()->setLowerBound(lowerBound)) return 1;//设置下边界
//        if(!solver.data()->setUpperBound(upperBound)) return 1;//设置上边界
//
//        // instantiate the solver
//        if(!solver.initSolver()) return 1;
//
//        // solve the QP problem
//        if(!solver.solve())
//            return 1;
//        else
//            isTraj=true;
//
//        // get the controller input
//        _BezierSolution.push_back(solver.getSolution());
//
////        std::cout << "QPSolution:" << std::endl << solver.getSolution() << std::endl;
//    }
//
//
//}

bool BezierTrajGenerator::TrajGeneration(Eigen::Vector3d start_pt,Eigen::Vector3d end_pt,vector<Cube> corridor)
{
    isTraj= false;

    _corridor = corridor;
    _start_pt = start_pt;
    _end_pt = end_pt;

    n_seg = corridor.size();//  有多少个走廊就有多少段轨迹
    n_all_coeff = n_seg*n_coeff;

    timeAllocation();

    auto M = getM();
    auto Q = getQ();
    auto Q_0 = M.transpose()*Q*M;

//  将Q_0优化为最近半正定对称矩阵
    MatrixXd Q_0_nearest = nearestSPD(Q_0);

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian; //P or H
    Eigen::VectorXd gradient;  //f or q
    Eigen::SparseMatrix<double> linearMatrix;  //A
    Eigen::VectorXd lowerBound; //l
    Eigen::VectorXd upperBound; //u

    hessian = Q_0_nearest.sparseView();
    gradient = Eigen::VectorXd::Zero(Q_0_nearest.rows());

    auto Aeq = getAeq();
    auto Aieq = getAieq_plus();
    Eigen::MatrixXd A = MatrixXd::Zero(Aeq.rows()+Aieq.rows(),Aeq.cols());
    A<<Aeq,
            Aieq;
    int NumberOfVariables = A.cols(); //A矩阵的列数
    int NumberOfConstraints = A.rows(); //A矩阵的行数
    linearMatrix = A.sparseView();


    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);//   设置是否查看迭代过程
    solver.settings()->setRelativeTolerance(5e-02);
//    solver.settings()->setRho(0.2);// 设置步长
    solver.settings()->setMaxIteraction(1000000);

    // set the initial data of the QP solver
    //矩阵A为m*n矩阵
    solver.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
    solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
    if(!solver.data()->setHessianMatrix(hessian)) return 1;//设置P矩阵
    if(!solver.data()->setGradient(gradient)) return 1; //设置q or f矩阵。当没有时设置为全0向量
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;//设置线性约束的A矩阵

    _BezierSolution.clear();
    for(int i = 0;i<2;++i)
    {
        solver.clearSolver();

        lowerBound.resize(A.rows());
        lowerBound<<getbeq(i),
                getbieq_minus(i);

        upperBound.resize(A.rows());
        upperBound<<getbeq(i),
                getbieq_plus(i);

        if(!solver.data()->setLowerBound(lowerBound)) return 1;//设置下边界
        if(!solver.data()->setUpperBound(upperBound)) return 1;//设置上边界

        // instantiate the solver
        if(!solver.initSolver()) return 0;

        // solve the QP problem
        if(!solver.solve())
        {
            return 0;
        }
        else
        {
            if(i==1)
            {
                isTraj=true;
            }
            // get the controller input
            _BezierSolution.push_back(solver.getSolution());
//        std::cout << "QPSolution:" << std::endl << solver.getSolution() << std::endl;
        }

    }


}


/**
 * 默认为初始速度和终止速度是0
 * TODO:修改为任意初始和终止速度
 * @param dis
 * @return
 */
void BezierTrajGenerator::timeAllocation()
{
    _totalTime = 0;
    double dis =0;
    for (int i = 0; i < n_seg; ++i)
    {
        if(n_seg==1)
        {
            dis = (_end_pt-_corridor[i].center).norm();
        }
        else if(i ==n_seg-1)
        {
            dis = (_end_pt-_corridor[i].center).norm();
        }
        else
        {
            dis = (_corridor[i+1].center-_corridor[i].center).norm();
        }
        if (dis <= _Vel_max * _Vel_max / _Acc_max)
        {
            _corridor[i].t = 2*sqrt(dis / _Acc_max);//   x2防止突变
        } else
        {
            _corridor[i].t = 2*_Vel_max / _Acc_max + (dis - _Vel_max * _Vel_max / _Acc_max) / _Vel_max;
        }
        cout << "time:" << endl << _corridor[i].t << endl;
        _totalTime += _corridor[i].t;
    }
}


/**
 * 将贝塞尔曲线系数映射为传统多项式系数
 * p = M*c
 * @return
 */
Eigen::MatrixXd BezierTrajGenerator::getM() {
    MatrixXd M_k(n_coeff, n_coeff);
    M_k<< 1  ,  0  ,  0  ,  0  ,  0 ,  0 ,0 , 0,
            -7  ,  7 ,   0 ,   0  ,  0  , 0 , 0 , 0,
            21 , -42 ,  21  ,  0   , 0 ,  0 , 0 , 0,
            -35 , 105, -105 ,  35  ,  0 ,  0 , 0 , 0,
            35, -140 , 210 ,-140  , 35 ,  0  ,0 , 0,
            -21 , 105 ,-210 , 210, -105,  21 , 0 , 0,
            7  ,-42 , 105, -140,  105 ,-42 , 7 , 0,
            -1 ,  7 ,  -21 ,  35,  -35 , 21 ,-7 , 1;

    MatrixXd M = MatrixXd::Zero(n_seg * n_coeff, n_seg * n_coeff);
    for(int i = 0;i<n_seg;++i)
    {
        M.block(i * n_coeff, i * n_coeff, n_coeff, n_coeff) = M_k;
    }
    return M;
}

Eigen::MatrixXd BezierTrajGenerator::getQ_k(double t_k) {
    MatrixXd Q_k(n_coeff, n_coeff);
    /**
     * 这个单纯是七阶多项式的四阶(snap)导的Q矩阵
     * p0,...,pn
     */
    double t1 = pow(t_k,1);
    double t2 = pow(t_k,2);
    double t3 = pow(t_k,3);
    double t4 = pow(t_k,4);
    double t5 = pow(t_k,5);
    double t6 = pow(t_k,6);
    double t7 = pow(t_k,7);

    Q_k<< 0, 0, 0, 0,         0,        0,        0,         0,
            0, 0, 0, 0,       0,        0,        0,         0,
            0, 0, 0, 0,       0,        0,        0,         0,
            0, 0, 0, 0,       0,        0,        0,         0,
            0, 0, 0, 0,  576*t1,  1440*t2,  2880*t3,   5040*t4,
            0, 0, 0, 0, 1440*t2,  4800*t3, 10800*t4,  20160*t5,
            0, 0, 0, 0, 2880*t3, 10800*t4, 25920*t5,  50400*t6,
            0, 0, 0, 0, 5040*t4, 20160*t5, 50400*t6, 100800*t7;
    return Q_k;
}

Eigen::MatrixXd BezierTrajGenerator::getQ() {
    /**
     * f(t) = \int p_i*t^i
     * cost = p^T*Q*p = c^T*M^T*Q*M*c = c^T*Q_0*c = \int (f^(4)(t))^2 dt
     * 所以Q是snap多项式平方积分提出系数p的矩阵系数
     */

    MatrixXd Q = MatrixXd::Zero(n_seg * n_coeff, n_seg * n_coeff);

    for(int i = 0;i<n_seg;++i)
    {
        auto Q_k = getQ_k(_corridor[i].t);
        Q.block(i * n_coeff, i * n_coeff, n_coeff, n_coeff) = Q_k;
    }
    return Q;
}

/**
 * 等式约束(边界约束,连续性约束)构造等式Aeq
 * @return
 */
Eigen::MatrixXd BezierTrajGenerator::getAeq()
{
    int n_2 = n_order*(n_order-1);

    MatrixXd Aeq_start(3, n_coeff);
    /**
     * 贝赛尔曲线求导(系数形式为杨辉三角)
     */
    Aeq_start<<1,    0,  0, 0, 0, 0, 0, 0,  //  p
            -n_order,   n_order,  0, 0, 0, 0, 0, 0,   //  v
            n_2, -2*n_2, n_2, 0, 0, 0, 0, 0;   //  a

    MatrixXd Aeq_end(3, n_coeff);
    Aeq_end<<0,    0,  0, 0, 0, 0, 0, 1,  //  p
            0,   0,  0, 0, 0, 0, -n_order, n_order,   //  v
            0, 0, 0, 0, 0, n_2, -2*n_2, n_2;   //  a

    MatrixXd Aeq_con_p = MatrixXd::Zero(n_seg-1,n_all_coeff);
    MatrixXd Aeq_con_v = MatrixXd::Zero(n_seg-1,n_all_coeff);
    MatrixXd Aeq_con_a = MatrixXd::Zero(n_seg-1,n_all_coeff);

    for(int i=0;i<n_seg-1;++i)
    {
        int index = n_coeff-1 + n_coeff*i;

        /**
         * 连续性条件
         */
        /**
         * p:对应前一控制点与后一控制点相同
         * c_(i)-c_(i+1) = 0
         */
        Aeq_con_p(i,index) = 1;
        Aeq_con_p(i,index+1) = -1;

        Aeq_con_v(i,index-1) = -n_order;    Aeq_con_v(i,index) = n_order;
        Aeq_con_v(i,index+1) = n_order;    Aeq_con_v(i,index+2) = -n_order;

        Aeq_con_a(i,index-2) = n_2;    Aeq_con_a(i,index-1) = -2*n_2;  Aeq_con_a(i,index) = n_2;
        Aeq_con_a(i,index+1) = -n_2;    Aeq_con_a(i,index+2) = 2*n_2;  Aeq_con_a(i,index+3) = -n_2;
    }

    MatrixXd Aeq = MatrixXd::Zero(6+3*(n_seg-1),n_all_coeff);
    Aeq<<Aeq_start,MatrixXd::Zero(3,n_all_coeff-n_coeff),
            MatrixXd::Zero(3,n_all_coeff-n_coeff),Aeq_end,
            Aeq_con_p,
            Aeq_con_v,
            Aeq_con_a;

//    cout<<Aeq<<endl;
//    cout<<"Aeq.rows()"<<Aeq.rows()<<endl;
//    cout<<"Aeq.cols()"<<Aeq.cols()<<endl;
    return Aeq;
}

/**
 * 等式约束(边界约束,连续性约束)构造等式beq
 * @return
 */
Eigen::VectorXd BezierTrajGenerator::getbeq(int axis)
{
    Eigen::VectorXd beq_start(3);
    beq_start<<_start_pt(axis),
            0,
            0;

    Eigen::VectorXd beq_end(3);
    beq_end<<_end_pt(axis),
            0,
            0;

    Eigen::VectorXd beq_con_p = Eigen::VectorXd::Zero(n_seg-1);
    Eigen::VectorXd beq_con_v = Eigen::VectorXd::Zero(n_seg-1);
    Eigen::VectorXd beq_con_a = Eigen::VectorXd::Zero(n_seg-1);

    Eigen::VectorXd beq = Eigen::VectorXd::Zero(6+3*(n_seg-1));
    beq<<beq_start,
            beq_end,
            beq_con_p,
            beq_con_v,
            beq_con_a;

//    cout<<beq<<endl;
//    cout<<"beq.rows()"<<endl;
//    cout<<beq.rows()<<endl;
    return beq;
}

/**
 * 不等式约束(安全约束,动态性能约束)构造不等式Aieq
 * @return
 */
Eigen::MatrixXd BezierTrajGenerator::getAieq() {
    int n_constraint_p = n_seg*(n_coeff-1)-1;   //  去掉首尾,首尾已有等式约束限制

    /**
     * Ac<b
     */
    MatrixXd Aieq_p_plus = MatrixXd::Zero(n_constraint_p, n_all_coeff);
    for(int i = 0;i<n_seg-1;++i)//  n_seg-1:最后一段单独设置(终点是等式约束)
    {
        /**
         * (n_coeff-1)*i:   因为不考虑初始点,同理往后顺延所以带入的是eye(7)
         * 1+n_coeff*i:   不含初始点(因为初始点是等式约束)
         */
        Aieq_p_plus.block((n_coeff-1)*i, 1+n_coeff*i, 7, 7) = MatrixXd::Identity(7, 7);
    }
    Aieq_p_plus.block(n_constraint_p - 6, n_all_coeff - 7, 6, 6) = MatrixXd::Identity(6, 6);//   因为每段轨迹间共用一个控制点,所以最后要留空给终点(等式约束)

    MatrixXd Aieq_p_minus = -Aieq_p_plus;//  a<Ac -> -Ac<-a

    MatrixXd Aieq_p = MatrixXd::Zero(2 * n_constraint_p, n_all_coeff);//   x2:+-约束
    Aieq_p<<Aieq_p_plus,
            Aieq_p_minus;


    int n_constraint_v = n_seg*n_order;
    int d1 = n_order;
    int d2 = n_order*(n_order-1);

    MatrixXd derivate_matrix(7,8);
    derivate_matrix<<-d1, d1,  0,   0,   0,   0,   0,  0,
            0, -d1, d1,   0,   0,   0,   0,  0,
            0,  0, -d1,  d1,   0,   0,   0,  0,
            0,  0,   0, -d1,  d1,   0,   0,  0,
            0,  0,   0,   0, -d1,  d1,   0,  0,
            0,  0,   0,   0,   0, -d1,  d1,  0,
            0,  0,   0,   0,   0,   0, -d1, d1;
    MatrixXd Aieq_v_plus = MatrixXd::Zero(n_constraint_v,n_all_coeff);
    for(int i =0;i<n_seg;++i)
    {
        Aieq_v_plus.block((n_coeff-1)*i,n_coeff*i,7,8) = derivate_matrix;
    }
    MatrixXd Aieq_v_minus = -Aieq_v_plus;
    MatrixXd Aieq_v = MatrixXd::Zero(2*n_constraint_v,n_all_coeff);
    Aieq_v<<Aieq_v_plus,
            Aieq_v_minus;

    int n_constraint_a = n_seg*(n_coeff-2);
    MatrixXd dderivate_matrix(6,8);
    dderivate_matrix<<d2, -2*d2,    d2,     0,     0,     0,     0,  0,
            0,    d2, -2*d2,    d2,     0,     0,     0,  0,
            0,     0,    d2, -2*d2,    d2,     0,     0,  0,
            0,     0,     0,    d2, -2*d2,    d2,     0,  0,
            0,     0,     0,     0,    d2, -2*d2,    d2,  0,
            0,     0,     0,     0,     0,    d2, -2*d2,  d2;
    MatrixXd Aieq_a_plus = MatrixXd::Zero(n_constraint_a,n_all_coeff);
    for(int i =0;i<n_seg;++i)
    {
        Aieq_a_plus.block((n_coeff-2)*i,n_coeff*i,6,8) =dderivate_matrix;
    }
    MatrixXd Aieq_a_minus = -Aieq_a_plus;
    MatrixXd Aieq_a = MatrixXd::Zero(2*n_constraint_a,n_all_coeff);
    Aieq_a<<Aieq_a_plus,
            Aieq_a_minus;

    MatrixXd Aieq(2*(n_constraint_p),n_all_coeff);//+n_constraint_v+n_constraint_a
    Aieq<<Aieq_p;
//            Aieq_v;
//            Aieq_a;
//    cout<<Aieq_a<<endl;
//    cout<<"Aieq_a.rows()"<<Aieq_a.rows()<<endl;
//    cout<<"Aieq_a.cols()"<<Aieq_a.cols()<<endl;
    return Aieq;
}

/**
 * 不等式约束(安全约束,动态性能约束)构造不等式bieq
 * @return
 */
Eigen::VectorXd BezierTrajGenerator::getbieq(int axis){
    int n_constraint_p = n_seg*(n_coeff-1)-1;   //  去掉首尾,首尾已有等式约束限制

    /**
     * Ac<b
     */
    Eigen::VectorXd bieq_p_plus = Eigen::VectorXd::Zero(n_constraint_p);
    for(int i = 0;i<n_seg;++i)
    {
        int first_coeff_index = (n_coeff-1)*i;
        /**
         *  对每段 2:n_coeff-1 个点进行限制
         *  这些点一定在该走廊的上下界
         */

        for(int k = 0;k<n_coeff-2;++k)
        {
            bieq_p_plus(first_coeff_index+k) = _corridor[i].max_pt(axis);
        }
        /**
         * 对每段第 n_coeff 个点进行限制(走廊的交接点)
         * 第n_coeff点是轨迹间共同控制点
         * 其上界要根据两走廊的最小上界决定
         */
        if(i!=n_seg-1)
        {
            if(_corridor[i+1].max_pt(axis)>_corridor[i].max_pt(axis))//    对比相关点当前走廊最大限制和下一走廊最小限制,取最大的作为限制
            {
                bieq_p_plus(first_coeff_index+6) = max(_corridor[i].max_pt(axis),_corridor[i+1].min_pt(axis));
            }
            else
            {
                bieq_p_plus(first_coeff_index+6) = max(_corridor[i].min_pt(axis),_corridor[i+1].max_pt(axis));
            }
        }
    }

    /**
     * -Ac<-a
     */
    Eigen::VectorXd bieq_p_minus = MatrixXd::Zero(n_constraint_p,1);
    for(int i = 0;i<n_seg;++i)
    {
        int first_coeff_index = (n_coeff-1)*i;
        /**
         *  对每段 2:n_coeff-1 个点进行限制
         */
        for(int k = 0;k<n_coeff-2;++k)
        {
            bieq_p_minus(first_coeff_index+k) = -_corridor[i].min_pt(axis);
        }
        /**
         * 对每段第 n_coeff 个点进行限制
         * 第n_coeff点是轨迹间共同控制点
         */
        if(i!=n_seg-1)
        {
            if(_corridor[i+1].max_pt(axis)>_corridor[i].max_pt(axis))//    对比相关点当前走廊最大限制和下一走廊最小限制,取最大的作为限制
            {
                bieq_p_minus(first_coeff_index+6) = -min(_corridor[i].max_pt(axis),_corridor[i+1].min_pt(axis));
            }
            else
            {
                bieq_p_minus(first_coeff_index+6) = -min(_corridor[i].min_pt(axis),_corridor[i+1].max_pt(axis));
            }
        }
    }
    Eigen::VectorXd bieq_p = Eigen::VectorXd::Zero(2*n_constraint_p);//   x2:+-约束
    bieq_p<<bieq_p_plus,
            bieq_p_minus;

    int n_constraint_v = n_seg*(n_coeff-1);
    Eigen::VectorXd bieq_v = Eigen::VectorXd::Constant(2*n_constraint_v,_Vel_max);

    int n_constraint_a = n_seg*(n_coeff-2);
    Eigen::VectorXd bieq_a = Eigen::VectorXd::Constant(2*n_constraint_a,_Acc_max);

    Eigen::VectorXd bieq(2*(n_constraint_p));//+n_constraint_v+n_constraint_a
    bieq<<bieq_p;
//            bieq_v;
//            bieq_a;
    cout<<bieq_p<<endl;
    cout<<"bieq_a.rows()"<<bieq_p.rows()<<endl;
    return bieq;
}

Eigen::MatrixXd BezierTrajGenerator::getAieq_plus() {
    int n_constraint_p = n_seg*(n_coeff-1)-1;   //  去掉首尾,首尾已有等式约束限制

    /**
     * Ac<b
     */
    MatrixXd Aieq_p_plus = MatrixXd::Zero(n_constraint_p, n_all_coeff);
    for(int i = 0;i<n_seg-1;++i)//  n_seg-1:最后一段单独设置(终点是等式约束)
    {
        /**
         * (n_coeff-1)*i:   因为不考虑初始点,同理往后顺延所以带入的是eye(7)
         * 1+n_coeff*i:   不含初始点(因为初始点是等式约束)
         */
        Aieq_p_plus.block((n_coeff-1)*i, 1+n_coeff*i, 7, 7) = MatrixXd::Identity(7, 7);
    }
    Aieq_p_plus.block(n_constraint_p - 6, n_all_coeff - 7, 6, 6) = MatrixXd::Identity(6, 6);//   因为每段轨迹间共用一个控制点,所以最后要留空给终点(等式约束)

    int n_constraint_v = n_seg*n_order;
    int d1 = n_order;
    int d2 = n_order*(n_order-1);

    MatrixXd derivate_matrix(7,8);
    derivate_matrix<<-d1, d1,  0,   0,   0,   0,   0,  0,
            0, -d1, d1,   0,   0,   0,   0,  0,
            0,  0, -d1,  d1,   0,   0,   0,  0,
            0,  0,   0, -d1,  d1,   0,   0,  0,
            0,  0,   0,   0, -d1,  d1,   0,  0,
            0,  0,   0,   0,   0, -d1,  d1,  0,
            0,  0,   0,   0,   0,   0, -d1, d1;
    MatrixXd Aieq_v_plus = MatrixXd::Zero(n_constraint_v,n_all_coeff);
    for(int i =0;i<n_seg;++i)
    {
        Aieq_v_plus.block((n_coeff-1)*i,n_coeff*i,7,8) = derivate_matrix;
    }

    int n_constraint_a = n_seg*(n_coeff-2);
    MatrixXd dderivate_matrix(6,8);
    dderivate_matrix<<d2, -2*d2,    d2,     0,     0,     0,     0,  0,
            0,    d2, -2*d2,    d2,     0,     0,     0,  0,
            0,     0,    d2, -2*d2,    d2,     0,     0,  0,
            0,     0,     0,    d2, -2*d2,    d2,     0,  0,
            0,     0,     0,     0,    d2, -2*d2,    d2,  0,
            0,     0,     0,     0,     0,    d2, -2*d2,  d2;
    MatrixXd Aieq_a_plus = MatrixXd::Zero(n_constraint_a,n_all_coeff);
    for(int i =0;i<n_seg;++i)
    {
        Aieq_a_plus.block((n_coeff-2)*i,n_coeff*i,6,8) =dderivate_matrix;
    }

    MatrixXd Aieq_plus((n_constraint_p),n_all_coeff);//+n_constraint_v+n_constraint_a
    Aieq_plus<<Aieq_p_plus;
//            Aieq_v;
//            Aieq_a;
//    cout<<Aieq_a<<endl;
//    cout<<"Aieq_a.rows()"<<Aieq_a.rows()<<endl;
//    cout<<"Aieq_a.cols()"<<Aieq_a.cols()<<endl;
    return Aieq_plus;
}

Eigen::VectorXd BezierTrajGenerator::getbieq_plus(int axis){
    int n_constraint_p = n_seg*(n_coeff-1)-1;   //  去掉首尾,首尾已有等式约束限制

    /**
     * Ac<b
     */
    Eigen::VectorXd bieq_p_plus = Eigen::VectorXd::Zero(n_constraint_p);
    for(int i = 0;i<n_seg;++i)
    {
        int first_coeff_index = (n_coeff-1)*i;
        /**
         *  对每段 2:n_coeff-1 个点进行限制
         *  这些点一定在该走廊的上下界
         */

        for(int k = 0;k<n_coeff-2;++k)
        {
            bieq_p_plus(first_coeff_index+k) = _corridor[i].max_pt(axis);
        }
        /**
         * 对每段第 n_coeff 个点进行限制(走廊的交接点)
         * 第n_coeff点是轨迹间共同控制点
         * 其上界要根据两走廊的最小上界决定
         */
        if(i!=n_seg-1)
        {
            if(_corridor[i+1].max_pt(axis)>_corridor[i].max_pt(axis))//    对比相关点当前走廊最大限制和下一走廊最小限制,取最大的作为限制
            {
                bieq_p_plus(first_coeff_index+6) = max(_corridor[i].max_pt(axis),_corridor[i+1].min_pt(axis));
            }
            else
            {
                bieq_p_plus(first_coeff_index+6) = max(_corridor[i].min_pt(axis),_corridor[i+1].max_pt(axis));
            }
        }
    }

    int n_constraint_v = n_seg*(n_coeff-1);
    Eigen::VectorXd bieq_v_plus = Eigen::VectorXd::Constant(n_constraint_v,_Vel_max);

    int n_constraint_a = n_seg*(n_coeff-2);
    Eigen::VectorXd bieq_a_plus = Eigen::VectorXd::Constant(n_constraint_a,_Acc_max);

    Eigen::VectorXd bieq_plus((n_constraint_p));//+n_constraint_v+n_constraint_a
    bieq_plus<<bieq_p_plus;
//            bieq_v;
//            bieq_a;
//    cout<<bieq_p<<endl;
//    cout<<"bieq_a.rows()"<<bieq_p.rows()<<endl;
    return bieq_plus;
}

Eigen::VectorXd BezierTrajGenerator::getbieq_minus(int axis){
    int n_constraint_p = n_seg*(n_coeff-1)-1;   //  去掉首尾,首尾已有等式约束限制

    /**
     * -Ac<-a
     */
    Eigen::VectorXd bieq_p_minus = MatrixXd::Zero(n_constraint_p,1);
    for(int i = 0;i<n_seg;++i)
    {
        int first_coeff_index = (n_coeff-1)*i;
        /**
         *  对每段 2:n_coeff-1 个点进行限制
         */
        for(int k = 0;k<n_coeff-2;++k)
        {
            bieq_p_minus(first_coeff_index+k) = _corridor[i].min_pt(axis);
        }
        /**
         * 对每段第 n_coeff 个点进行限制
         * 第n_coeff点是轨迹间共同控制点
         */
        if(i!=n_seg-1)
        {
            if(_corridor[i+1].max_pt(axis)>_corridor[i].max_pt(axis))//    对比相关点当前走廊最大限制和下一走廊最小限制,取最大的作为限制
            {
                bieq_p_minus(first_coeff_index+6) = min(_corridor[i].max_pt(axis),_corridor[i+1].min_pt(axis));
            }
            else
            {
                bieq_p_minus(first_coeff_index+6) = min(_corridor[i].min_pt(axis),_corridor[i+1].max_pt(axis));
            }
        }
    }

    int n_constraint_v = n_seg*(n_coeff-1);
    Eigen::VectorXd bieq_v = Eigen::VectorXd::Constant(n_constraint_v,-_Vel_max);

    int n_constraint_a = n_seg*(n_coeff-2);
    Eigen::VectorXd bieq_a = Eigen::VectorXd::Constant(n_constraint_a,-_Acc_max);

    Eigen::VectorXd bieq_minus((n_constraint_p));//+n_constraint_v+n_constraint_a
    bieq_minus<<bieq_p_minus;
//            bieq_v;
//            bieq_a;
//    cout<<bieq_p<<endl;
//    cout<<"bieq_a.rows()"<<bieq_p.rows()<<endl;
    return bieq_minus;
}

Vector3d BezierTrajGenerator::getPolyStates(int k, double t_seg, int order)
{
    Vector3d ret(0,0,0);

    double t = t_seg/_corridor[k].t;

    int prefix = 1;
    for(int i =0;i<order;++i)
    {
        prefix *= n_order-i;
    }

    auto temp_solution = _BezierSolution;
    int I_range = n_order-order;
    for(int i =0;i<order;++i)
    {
        for(int j =0;j<=I_range;++j)
        {
            temp_solution[0](j+k*n_coeff) = temp_solution[0](j+1+k*n_coeff) - temp_solution[0](j+k*n_coeff);
            temp_solution[1](j+k*n_coeff) = temp_solution[1](j+1+k*n_coeff) - temp_solution[1](j+k*n_coeff);
        }
    }

    for(int i = 0;i<I_range;++i)
    {
        int J_range = I_range - i;
        for(int j=0;j<J_range;++j)
        {
            temp_solution[0](j+k*n_coeff) = (1.0-t)*temp_solution[0](j+k*n_coeff) + t*temp_solution[0](j+1+k*n_coeff);
            temp_solution[1](j+k*n_coeff) = (1.0-t)*temp_solution[1](j+k*n_coeff) + t*temp_solution[1](j+1+k*n_coeff);
        }
    }
    ret(0)=temp_solution[0](0+k*n_coeff);
    ret(1)=temp_solution[1](0+k*n_coeff);

    ret = prefix*ret;

//    for(int i=0;i<=n_order-order;++i)
//    {
//        double basis_p = nchoosek(n_order-order,i)*pow(t,i)*pow(1-t,n_order-i);
//        ret(0)+=_BezierSolution[0](i+k*n_coeff)*basis_p;
//        ret(1)+=_BezierSolution[1](i+k*n_coeff)*basis_p;
//    }
    return ret;
}

//Vector3d BezierTrajGenerator::getPolyStates(int k, double t_seg, int order)
//{
//    Vector3d ret(0,0,0);
//
//    double t = t_seg/_corridor[k].t;
//
//    for(int i=0;i<=n_order;++i)
//    {
//        double basis_p = nchoosek(n_order,i)*pow(t,i)*pow(1-t,n_order-i);
//        ret(0)+=_BezierSolution[0](i+k*n_coeff)*basis_p;
//        ret(1)+=_BezierSolution[1](i+k*n_coeff)*basis_p;
//    }
//    return ret;
//}


Vector3d BezierTrajGenerator::getTrajectoryStates(double time_from_start, int order)
{
//    每一段轨迹的时间起始于0
//    t所在的时间段的初始时间
    double t_init = 0;
//    t在其对应时间段上的时间
    double t_seg = 0;

    //找出对应时间对应的第几段轨迹
    int seg_idx = 0;

    for (int i = 0; i < _polyTime.size(); i++)
    {
        if (time_from_start >= t_init + _polyTime(i))
        {
            t_init += _polyTime(i);
        } else
        {
            t_seg = time_from_start - t_init;
            seg_idx = i;
            break;
        }
    }

    Vector3d states = getPolyStates(seg_idx, t_seg, order);

    return states;
}


//Vector3d BezierTrajGenerator::getTrajectoryStates(double time_from_start, int order)
//{
////    每一段轨迹的时间起始于0
////    t所在的时间段的初始时间
//    double t_init = 0;
////    t在其对应时间段上的时间
//    double t_seg = 0;
//
//    //找出对应时间对应的第几段轨迹
//    int seg_idx = 0;
//
//    for (int i = 0; i < _polyTime.size(); i++)
//    {
//        if (time_from_start >= t_init + _polyTime(i))
//        {
//            t_init += _polyTime(i);
//        } else
//        {
//            t_seg = time_from_start - t_init;
//            seg_idx = i;
//            break;
//        }
//    }
//
//    Vector3d states = getPolyStates(seg_idx, t_seg, order);
//
//    return states;
//}


visualization_msgs::Marker BezierTrajGenerator::visBezierTraj()
{
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp = ros::Time::now();
    _traj_vis.header.frame_id = "map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = 0.2;
    _traj_vis.scale.y = 0.2;
    _traj_vis.scale.z = 0.2;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;


    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;

    for (int i = 0; i < _corridor.size(); i++)
    {
        for (double t = 0.0; t < _corridor[i].t; t += 0.01)
        {
            pos = getPolyStates(i, t, 0);
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            _traj_vis.points.push_back(pt);

        }
    }
    return _traj_vis;
}

visualization_msgs::Marker BezierTrajGenerator::visBezierPt()
{
    visualization_msgs::Marker _bezierPt_vis;

    _bezierPt_vis.header.stamp = ros::Time::now();
    _bezierPt_vis.header.frame_id = "map";

    _bezierPt_vis.ns = "_bezierPt_vis";
    _bezierPt_vis.id = 0;
    _bezierPt_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _bezierPt_vis.action = visualization_msgs::Marker::ADD;
    _bezierPt_vis.scale.x = 0.2;
    _bezierPt_vis.scale.y = 0.2;
    _bezierPt_vis.scale.z = 0.2;
    _bezierPt_vis.pose.orientation.x = 0.0;
    _bezierPt_vis.pose.orientation.y = 0.0;
    _bezierPt_vis.pose.orientation.z = 0.0;
    _bezierPt_vis.pose.orientation.w = 1.0;

    _bezierPt_vis.color.a = 1.0;
    _bezierPt_vis.color.r = 1.0;
    _bezierPt_vis.color.g = 1.0;
    _bezierPt_vis.color.b = 0.0;

    _bezierPt_vis.points.clear();

    geometry_msgs::Point pt;

    for (int k = 0;k<_BezierSolution[0].size();++k)
    {
        pt.x = _BezierSolution[0](k);
        pt.y = _BezierSolution[1](k);
        pt.z = 0.2;
        _bezierPt_vis.points.push_back(pt);
    }

    return _bezierPt_vis;
}