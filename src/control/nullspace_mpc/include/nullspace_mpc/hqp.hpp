# pragma once
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <qp_solver_collection/QpSolverCollection.h>
#include "nullspace_mpc/task_template.hpp"

namespace hqp
{
    class HQP
    {
        public:
            HQP();
            ~HQP();
            void addTask(Task* task);
            void echoTaskList();
            Eigen::VectorXd solve(); // return x_star_
        private:
            std::vector<Task*> task_list_; // task list
            Eigen::MatrixXd calcNullspace(Eigen::MatrixXd A);
            Eigen::MatrixXd calcHpp1(Task* task);
            Eigen::VectorXd calcCpp1(Task* task);
            Eigen::MatrixXd calcDtildapp1(Task* task);
            Eigen::VectorXd calcFtildapp1(Task* task);
            Eigen::MatrixXd calcZpp1(Task* task);

            Eigen::VectorXd x_star_;
            std::vector<Eigen::VectorXd> x_star_list_;
            Eigen::MatrixXd Z_prev_;
            std::vector<Eigen::MatrixXd> Z_list_;
            std::vector<Eigen::MatrixXd> D_list_;
            std::vector<Eigen::MatrixXd> f_list_;
            std::vector<Eigen::VectorXd> v_list_;
    };
} // namespace hqp
