#include "nullspace_mpc/hqp.hpp"

namespace hqp
{

// constructor
HQP::HQP()
{
    // No Contents
}

// destructor
HQP::~HQP()
{
    // No Contents
}

// add task
void HQP::addTask(Task* task)
{
    task_list_.push_back(task);
}

// echo task list
void HQP::echoTaskList()
{
    for (int i = 0; i < task_list_.size(); i++)
    {
        std::cout << "*** [Task " << i+1 << "] ***" << std::endl;
        task_list_[i]->echo();
    }
}

// solve
Eigen::VectorXd HQP::solve()
{
    // solve task 1
    //// get nullspace of A matrix of task 1
    Eigen::MatrixXd Z_1 = calcNullspace(task_list_[0]->A_);
    //// calculate pseudo-inverse of A matrix of task 1
    Eigen::MatrixXd pinv_A_1 = task_list_[0]->A_.transpose() * (task_list_[0]->A_ * task_list_[0]->A_.transpose()).inverse();
    //// get x_1_star
    Eigen::VectorXd x_1_star = pinv_A_1 * task_list_[0]->b_;
    // std::cout << "x_1_star: (" << x_1_star.rows() << ", " << x_1_star.cols() << ")" << std::endl << x_1_star << std::endl;

    //// save variables
    x_star_ = x_1_star;
    x_star_list_.push_back(x_1_star);
    Z_prev_ = Z_1;
    Z_list_.push_back(Z_1);
    D_list_.push_back(task_list_[0]->D_);
    f_list_.push_back(task_list_[0]->f_);
    v_list_.push_back(Eigen::VectorXd::Zero(task_list_[0]->D_.rows()));

    // apply task p+1 (p = 0, ..., N-2)
    for (int p = 0; p < task_list_.size()-1; p++)
    {
        // calculate H for task p+1
        Eigen::MatrixXd H = calcHpp1(task_list_[p+1]);
        // std::cout << "H: (" << H.rows() << ", " << H.cols() << ")" << std::endl << H << std::endl;

        // calculate c
        Eigen::VectorXd c = calcCpp1(task_list_[p+1]);
        // std::cout << "c: (" << c.rows() << ", " << c.cols() << ")" << std::endl << c << std::endl;

        // calculate D_tilda
        Eigen::MatrixXd Dtilda = calcDtildapp1(task_list_[p+1]);
        // std::cout << "Dtilda: (" << Dtilda.rows() << ", " << Dtilda.cols() << ")" << std::endl << Dtilda << std::endl;

        // calculate f_tilda
        Eigen::VectorXd Ftilda = calcFtildapp1(task_list_[p+1]);
        // std::cout << "Ftilda: (" << Ftilda.rows() << ", " << Ftilda.cols() << ")" << std::endl << Ftilda << std::endl;

        // solve qp
        QpSolverCollection::QpCoeff qp_coeff;
        qp_coeff.setup(H.rows(), 0, Dtilda.rows());
        qp_coeff.obj_mat_ = H;
        qp_coeff.obj_vec_ = c;
        qp_coeff.ineq_mat_ = Dtilda;
        qp_coeff.ineq_vec_ = Ftilda;
        qp_coeff.x_min_.setConstant(-100.0);
        qp_coeff.x_max_.setConstant(+100.0);
        auto qp_solver = QpSolverCollection::allocateQpSolver(QpSolverCollection::QpSolverType::Any);
        Eigen::VectorXd solution = qp_solver->solve(qp_coeff);
        // std::cout << "solution: (" << solution.rows() << ", " << solution.cols() << ")" << std::endl << solution << std::endl;
        Eigen::VectorXd zpp1 = solution.segment(0, Z_prev_.cols());
        Eigen::VectorXd vpp1 = solution.segment(Z_prev_.cols(), solution.rows() - Z_prev_.cols());

        // save variables of task p+1
        x_star_ = x_star_ + Z_prev_ * zpp1;
        x_star_list_.push_back(x_star_);
        Z_prev_ = calcZpp1(task_list_[p+1]);
        Z_list_.push_back(Z_prev_);
        D_list_.push_back(task_list_[p+1]->D_);
        f_list_.push_back(task_list_[p+1]->f_);
        v_list_.push_back(vpp1);
    }
    return x_star_;
}

Eigen::MatrixXd HQP::calcZpp1(Task* task)
{
    Eigen::MatrixXd NApp1Zp = calcNullspace(task->A_ * Z_prev_);
    Eigen::MatrixXd Zpp1 = Z_prev_ * NApp1Zp;
    return Zpp1;
}

Eigen::MatrixXd HQP::calcHpp1(Task* task)
{
    Eigen::MatrixXd zaap = Z_prev_.transpose() * task->A_.transpose() * task->A_ * Z_prev_;
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(task->D_.rows(), task->D_.rows());
    Eigen::MatrixXd Hpp1 = Eigen::MatrixXd::Zero(zaap.rows() + identity.rows(), zaap.cols() + identity.cols());
    Hpp1.block(0, 0, zaap.rows(), zaap.cols()) = zaap;
    Hpp1.block(zaap.rows(), zaap.cols(), identity.rows(), identity.cols()) = identity;
    return Hpp1;
}

Eigen::VectorXd HQP::calcCpp1(Task* task)
{
    Eigen::MatrixXd zaaxb = Z_prev_.transpose() * task->A_.transpose() * (task->A_ * x_star_ - task->b_);
    Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(task->D_.rows(), 1);
    Eigen::VectorXd c = Eigen::VectorXd::Zero(zaaxb.rows() + zero.rows());
    c.segment(0, zaaxb.rows()) = zaaxb;
    c.segment(zaaxb.rows(), zero.rows()) = zero;
    return c;
}

Eigen::MatrixXd HQP::calcDtildapp1(Task* task)
{
    // get dimension of v_{p+1}
    int dim_vpp1 = task->D_.rows();

    // top part
    Eigen::MatrixXd Dpp1 = task->D_;
    Eigen::MatrixXd Dpp1Zp = Dpp1 * Z_prev_;
    Eigen::MatrixXd Dtildapp1 = Eigen::MatrixXd::Zero(Dpp1Zp.rows(), Dpp1Zp.cols() + dim_vpp1);
    Dtildapp1.block(0, 0, Dpp1Zp.rows(), Dpp1Zp.cols()) = Dpp1Zp;
    Dtildapp1.block(0, Dpp1Zp.cols(), dim_vpp1, dim_vpp1) = Eigen::MatrixXd::Identity(dim_vpp1, dim_vpp1) * -1.0;

    // middle part
    for (int p=D_list_.size()-1; p>=0; p--)
    {
        Eigen::MatrixXd Dp = D_list_[p];
        int dim_v_p = Dp.rows();
        Eigen::MatrixXd DpZprev = Dp * Z_prev_;
        Eigen::MatrixXd Dtildapp1_middle = Eigen::MatrixXd::Zero(DpZprev.rows(), DpZprev.cols() + dim_vpp1);
        Dtildapp1_middle.block(0, 0, DpZprev.rows(), DpZprev.cols()) = DpZprev;
        Dtildapp1_middle.block(0, DpZprev.cols(), dim_vpp1, dim_vpp1) = Eigen::MatrixXd::Zero(dim_v_p, dim_vpp1);
        Dtildapp1.conservativeResize(Dtildapp1.rows() + Dtildapp1_middle.rows(), Dtildapp1.cols());
        Dtildapp1.block(Dtildapp1.rows() - Dtildapp1_middle.rows(), 0, Dtildapp1_middle.rows(), Dtildapp1.cols()) = Dtildapp1_middle;
    }

    // bottom part
    Eigen::MatrixXd Dtildapp1_bottom  = Eigen::MatrixXd::Zero(dim_vpp1, Z_prev_.cols()+dim_vpp1);
    Dtildapp1_bottom.block(0, Z_prev_.cols(), dim_vpp1, dim_vpp1) = Eigen::MatrixXd::Identity(dim_vpp1, dim_vpp1) * -1.0;
    Dtildapp1.conservativeResize(Dtildapp1.rows() + Dtildapp1_bottom.rows(), Dtildapp1.cols());
    Dtildapp1.block(Dtildapp1.rows() - Dtildapp1_bottom.rows(), 0, Dtildapp1_bottom.rows(), Dtildapp1.cols()) = Dtildapp1_bottom;

    return Dtildapp1;
}

Eigen::VectorXd HQP::calcFtildapp1(Task* task)
{
    // get dimension of v_{p+1}
    int dim_vpp1 = task->D_.rows();

    // top part
    Eigen::VectorXd Ftildapp1 = task->f_ - task->D_ * x_star_;

    // middle part
    for (int p=D_list_.size()-1; p>=0; p--)
    {
        Eigen::VectorXd fp = f_list_[p];
        Eigen::VectorXd vp = v_list_[p];
        Eigen::VectorXd Ftildapp1_middle = fp - D_list_[p] * x_star_ + vp;
        Ftildapp1.conservativeResize(Ftildapp1.rows() + Ftildapp1_middle.rows());
        Ftildapp1.segment(Ftildapp1.rows() - Ftildapp1_middle.rows(), Ftildapp1_middle.rows()) = Ftildapp1_middle;
    }

    // bottom part
    Eigen::VectorXd Ftildapp1_bottom = Eigen::VectorXd::Zero(dim_vpp1);
    Ftildapp1.conservativeResize(Ftildapp1.rows() + Ftildapp1_bottom.rows());
    Ftildapp1.segment(Ftildapp1.rows() - Ftildapp1_bottom.rows(), Ftildapp1_bottom.rows()) = Ftildapp1_bottom;

    return Ftildapp1;
}

// calculate nullspace
Eigen::MatrixXd HQP::calcNullspace(Eigen::MatrixXd A)
{
    // reference: https://stackoverflow.com/questions/34662940/how-to-compute-basis-of-nullspace-with-eigen-library
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod;
    cod.compute(A);

    Eigen::MatrixXd V = cod.matrixZ().transpose();
    Eigen::MatrixXd Null_space = V.block(0, cod.rank(),V.rows(), V.cols() - cod.rank());
    Eigen::MatrixXd P = cod.colsPermutation();
    Null_space = P * Null_space; // Unpermute the columns

    return Null_space;
}

} // namespace hqp