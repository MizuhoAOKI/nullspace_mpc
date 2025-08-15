#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <string>

namespace hqp
{
    class Task
    {
        public:
            Eigen::MatrixXd A_, b_; // equality constraint Ax = b
            Eigen::MatrixXd D_, f_; // inequality constraint Dx <= f
            std::string task_name_ = "undefined"; // task name
            void echo()  // print task_name, A, b, D, f
            {
                if (!is_constructed_)
                {
                    std::cout << "[ERROR] Task is not constructed yet" << std::endl;
                    return;
                }

                std::cout << "### Task Name: " << task_name_ << std::endl;
                std::cout << "A: (" << A_.rows() << ", " << A_.cols() << ")" << std::endl << A_ << std::endl;
                std::cout << "b: (" << b_.rows() << ", " << b_.cols() << ")" << std::endl << b_ << std::endl;
                std::cout << "D: (" << D_.rows() << ", " << D_.cols() << ")" << std::endl << D_ << std::endl;
                std::cout << "f: (" << f_.rows() << ", " << f_.cols() << ")" << std::endl << f_ << std::endl;
            }
            bool is_constructed_ = false; // flag for task construction
            bool isConstructed() { return is_constructed_; } // return is_constructed_
    };

} // namespace hqp
