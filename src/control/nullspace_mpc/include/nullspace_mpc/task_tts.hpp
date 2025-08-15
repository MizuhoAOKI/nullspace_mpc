#pragma once

#include "nullspace_mpc/task_template.hpp"

namespace hqp
{
    class TrackTargetState : public Task
    {
        public:
            // constructor
            TrackTargetState()
            {
                // set task name
                task_name_ = "Track Target State";
            }

            // destructor
            ~TrackTargetState()
            {
                // No Contents
            }

            void constructTask()
            {
                // set A, b
                constructAb();

                // set D, f
                constructDf();

                // set flag
                is_constructed_ = true;
            }

            // set prediction horizon
            void setPredictionHorizon(const int T){ T_ = T; }

            // set current state
            void setCurrentState(const double current_px, const double current_py, const double current_theta)
            {
                current_px_ = current_px;
                current_py_ = current_py;
                current_theta_ = current_theta;
            }

            // add tracking pose
            void addTrackingPose(const int target_time_step, const double tracking_px, const double tracking_py, const double tracking_theta)
            {
                // append target time step list
                tracking_pose_index_list_.push_back(target_time_step);

                // append target pose list
                Eigen::Vector3d tracking_pose;
                tracking_pose << tracking_px, tracking_py, tracking_theta;
                tracking_pose_list_.push_back(tracking_pose);
            }

            // add tracking velocity
            void addTrackingVelocity(const int target_time_step, const double tracking_vx, const double tracking_vy, const double tracking_omega)
            {
                // append target time step list
                tracking_vel_index_list_.push_back(target_time_step);

                // append target velocity list
                Eigen::Vector3d tracking_vel;
                tracking_vel << tracking_vx, tracking_vy, tracking_omega;
                tracking_vel_list_.push_back(tracking_vel);
            }

        private:
            // constant parameters
            const int Nx = 3; // number of state variables
            const int Nu = 3; // number of control variables
            const int NxNu = Nx + Nu; // number of state and control variables
            const double err_x_penalty_ = 1.0; // penalty for minimizing position error in x
            const double err_y_penalty_ = 1.0; // penalty for minimizing position error in y
            const double err_theta_penalty_ = 1.0; // penalty for minimizing orientation error in theta
            const double err_vx_penalty_ = 1.0; // penalty for minimizing velocity error in x
            const double err_vy_penalty_ = 1.0; // penalty for minimizing velocity error in y
            const double err_omega_penalty_ = 1.0; // penalty for minimizing angular velocity error in omega

            // variables
            int T_ = 0; // prediction horizon
            double current_px_, current_py_, current_theta_; // current pose

            // tracking pose index/vector list
            std::vector<int> tracking_pose_index_list_;
            std::vector<Eigen::Vector3d> tracking_pose_list_;

            // tracking velocity index/vector list
            std::vector<int> tracking_vel_index_list_;
            std::vector<Eigen::Vector3d> tracking_vel_list_;

            // construct matrices A_ and b_
            void constructAb()
            {
                // for each tracking pose
                for (int i = 0; i < tracking_pose_index_list_.size(); i++)
                {
                    // append A_
                    Eigen::MatrixXd At = Eigen::MatrixXd::Zero(Nx, Nx);
                    At << err_x_penalty_, 0.0, 0.0, 
                        0.0, err_y_penalty_, 0.0, 
                        0.0, 0.0, err_theta_penalty_;
                    A_.conservativeResizeLike(Eigen::MatrixXd::Zero(Nx*(i+1), NxNu*T_ + Nx));
                    A_.block(Nx*i, NxNu*tracking_pose_index_list_[i], Nx, Nx) = At;

                    // append b_
                    Eigen::MatrixXd bt = Eigen::MatrixXd::Zero(Nx, 1);
                    //// Note: all tracking yaw angles should be given in the range of [current_theta_ - pi, current_theta_ + pi]
                    bt << tracking_pose_list_[i](0, 0), tracking_pose_list_[i](1, 0), tracking_pose_list_[i](2, 0);
                    b_.conservativeResize(Nx*(i+1), 1);
                    b_.block(Nx*i, 0, Nx, 1) = bt;
                }

                // for each tracking velocity
                int row_bias = Nx*tracking_pose_index_list_.size(); // start row index from row_bias
                for (int i = 0; i < tracking_vel_index_list_.size(); i++)
                {
                    // append A_
                    Eigen::MatrixXd At = Eigen::MatrixXd::Zero(Nx, Nx);
                    At << err_vx_penalty_, 0.0, 0.0, 
                        0.0, err_vy_penalty_, 0.0, 
                        0.0, 0.0, err_omega_penalty_;
                    A_.conservativeResizeLike(Eigen::MatrixXd::Zero(row_bias + Nx*(i+1), NxNu*T_ + Nx));
                    A_.block(row_bias + Nx*i, Nx + NxNu*tracking_vel_index_list_[i], Nu, Nu) = At;

                    // append b_
                    Eigen::MatrixXd bt = Eigen::MatrixXd::Zero(Nx, 1);
                    bt << tracking_vel_list_[i](0, 0), tracking_vel_list_[i](1, 0), tracking_vel_list_[i](2, 0);
                    b_.conservativeResize(row_bias + Nx*(i+1), 1);
                    b_.block(row_bias + Nx*i, 0, Nx, 1) = bt;
                }
                // [FOR DEBUG] announce A and b for debugging
                // std::cout << "##### Track Target State #####" << std::endl;
                // std::cout << "A = " << std::endl << A_ << std::endl;
                // std::cout << "b = " << std::endl << b_ << std::endl;
                // std::cout << "shape of A = " << A_.rows() << " x " << A_.cols() << std::endl;
                // std::cout << "shape of b = " << b_.rows() << " x " << b_.cols() << std::endl;
                // std::cout << "#################################" << std::endl;
            }

            void constructDf()
            {
                // no inequality constraints
                D_ = Eigen::MatrixXd::Zero(1, NxNu*T_ + Nx);
                f_ = Eigen::MatrixXd::Zero(1, 1);
            }
    };
} // namespace hqp