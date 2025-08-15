#pragma once

#include "nullspace_mpc/task_template.hpp"

namespace hqp
{
    class MinimizeVelocityAndAcceleration : public Task
    {
        public:
            // constructor
            MinimizeVelocityAndAcceleration()
            {
                // set task name
                task_name_ = "Minimize Velocity and Acceleration";
            }

            // destructor
            ~MinimizeVelocityAndAcceleration()
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

            // set previous twist command
            void setPreviousTwistCommand(const double prev_vx, const double prev_vy, const double prev_omega)
            {
                prev_vx_ = prev_vx;
                prev_vy_ = prev_vy;
                prev_omega_ = prev_omega;
            }

        private:
            // constant parameters
            const int Nx = 3; // number of state variables
            const int Nu = 3; // number of control variables
            const int NxNu = Nx + Nu; // number of state and control variables
            const double cmd_change_penalty_ = 5.0; // penalty for minimizing twist command change
            const double acc_penalty_ = 0.6; // penalty for minimizing acceleration
            const double vel_penalty_ = 0.4; // penalty for minimizing velocity

            // variables
            int T_ = 0; // prediction horizon
            double prev_vx_ = 0.0; // previous vx
            double prev_vy_ = 0.0; // previous vy
            double prev_omega_ = 0.0; // previous omega

            // construct matrices A_ and b_
            void constructAb()
            {
                // step 1: (t = 1)
                Eigen::MatrixXd A0 = Eigen::MatrixXd::Zero(Nu, NxNu);
                // add penalty for twist command change from previous time step
                A0 << 0.0, 0.0, 0.0, cmd_change_penalty_, 0.0, 0.0, // vx_{0} = vx_prev
                      0.0, 0.0, 0.0, 0.0, cmd_change_penalty_, 0.0, // vy_{0} = vy_prev
                      0.0, 0.0, 0.0, 0.0, 0.0, cmd_change_penalty_; // omega_{0} = omega_prev
                A_.conservativeResizeLike(Eigen::MatrixXd::Zero(Nu, NxNu*T_ + Nx));
                A_.block(0, NxNu, Nu, NxNu) = A0;
                Eigen::MatrixXd b0 = Eigen::MatrixXd::Zero(Nu, 1);
                b0 << cmd_change_penalty_ * prev_vx_, cmd_change_penalty_ * prev_vy_, cmd_change_penalty_ * prev_omega_;
                b_.conservativeResize(Nu, 1);
                b_.block(0, 0, Nu, 1) = b0;

                // step t: (t = 1, 2, ..., T-1)
                for (int t = 1; t < T_; t++)
                {
                    // append A_
                    Eigen::MatrixXd At = Eigen::MatrixXd::Zero(Nu, Nu + Nx + Nu);
                    // minimize acceleration and velocity
                    At << -acc_penalty_, 0.0, 0.0, 0.0, 0.0, 0.0, acc_penalty_ + vel_penalty_, 0.0, 0.0, // vx_{t-1} - vx_{t} = 0
                        0.0, -acc_penalty_, 0.0, 0.0, 0.0, 0.0, 0.0, acc_penalty_ + vel_penalty_, 0.0, // vy_{t-1} - vy_{t} = 0
                        0.0, 0.0, -acc_penalty_, 0.0, 0.0, 0.0, 0.0, 0.0, acc_penalty_ + vel_penalty_; // omega_{t-1} - omega_{t} = 0
                    A_.conservativeResizeLike(Eigen::MatrixXd::Zero(Nu + Nu*t, NxNu*T_ + Nx));
                    A_.block(Nu + Nu*(t-1), Nx + NxNu*(t-1), Nu, Nu + Nx + Nu) = At;

                    // append b_
                    Eigen::MatrixXd bt = Eigen::MatrixXd::Zero(Nu, 1);
                    b_.conservativeResize(Nu + Nu*t, 1);
                    b_.block(Nu + Nu*(t-1), 0, Nu, 1) = bt;
                }
            }

            void constructDf()
            {
                // no inequality constraints
                D_ = Eigen::MatrixXd::Zero(1, NxNu*T_ + Nx);
                f_ = Eigen::MatrixXd::Zero(1, 1);
            }
    };
} // namespace hqp