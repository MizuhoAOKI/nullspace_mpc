#pragma once

#include "nullspace_mpc/task_template.hpp"

namespace hqp
{
    class SatisfyStateEquation : public Task
    {
        public:
            // constructor
            SatisfyStateEquation()
            {
                // set task name
                task_name_ = "Satisfy State Equation";
            }

            // destructor
            ~SatisfyStateEquation()
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

            // set time step length
            void setTimeStepLength(const double dt){ dt_ = dt; }

            // set initial states
            void setInitialStates(const double px0, const double py0, const double theta0)
            {
                px0_ = px0;
                py0_ = py0;
                theta0_ = theta0;
            }

            // set velocity limits
            void setVelocityLimits(const double vx_min, const double vx_max, const double vy_min, const double vy_max, const double omega_min, const double omega_max)
            {
                vx_min_ = vx_min;
                vx_max_ = vx_max;
                vy_min_ = vy_min;
                vy_max_ = vy_max;
                omega_min_ = omega_min;
                omega_max_ = omega_max;
            }

        private:
            // constant parameters
            const int Nx = 3; // number of state variables
            const int Nu = 3; // number of control variables
            const int NxNu = Nx + Nu; // number of state and control variables

            // variables
            int T_ = 0; // prediction horizon
            double dt_ = 0.1; // time step length
            double px0_, py0_, theta0_; // initial pose
            double vx_min_, vy_min_, omega_min_; // minimum velocity
            double vx_max_, vy_max_, omega_max_; // maximum velocity

            // construct matrices A_ and b_
            void constructAb()
            {
                // step 0: set initial pose and velocity
                Eigen::MatrixXd A0 = Eigen::MatrixXd::Identity(Nx, Nx);
                Eigen::MatrixXd Zero_A0_Right = Eigen::MatrixXd::Zero(Nx, Nu + NxNu*(T_-1)+Nx);
                A_.resize(Nx, NxNu*T_ + Nx);
                A_ << A0, Zero_A0_Right;

                Eigen::MatrixXd b0 = Eigen::MatrixXd::Zero(Nx, 1);
                b0 << px0_, py0_, theta0_; // set initial state as state at t = 0
                b_.resize(Nx, 1);
                b_ << b0;

                // step t: (t = 1, 2, ..., T)
                for (int t = 1; t <= T_; t++)
                {
                    // append A_
                    Eigen::MatrixXd At = Eigen::MatrixXd::Zero(Nx, NxNu+Nx);
                    At << -1.0, 0.0, 0.0, -dt_, 0.0, 0.0, 1.0, 0.0, 0.0, // px_{t-1} - dt*vx_{t-1} + px_{t} = 0
                          0.0, -1.0, 0.0, 0.0, -dt_, 0.0, 0.0, 1.0, 0.0, // py_{t-1} - dt*vy_{t-1} + py_{t} = 0
                          0.0, 0.0, -1.0, 0.0, 0.0, -dt_, 0.0, 0.0, 1.0; // theta_{t-1} - dt*omega_{t-1} + theta_{t} = 0
                    A_.conservativeResizeLike(Eigen::MatrixXd::Zero(Nx + Nx*t, NxNu*T_ + Nx));
                    A_.block(Nx + Nx*(t-1), NxNu*(t-1), Nx, NxNu + Nx) = At;

                    // append b_
                    Eigen::MatrixXd bt = Eigen::MatrixXd::Zero(Nx, 1);
                    b_.conservativeResize(Nx + Nx*t, 1);
                    b_.block(Nx + Nx*(t-1), 0, Nx, 1) = bt;
                }
                // [FOR DEBUG] announce A and b for debugging
                // std::cout << "##### Satify State Equation #####" << std::endl;
                // std::cout << "A = " << std::endl << A_ << std::endl;
                // std::cout << "b = " << std::endl << b_ << std::endl;
                // std::cout << "shape of A = " << A_.rows() << " x " << A_.cols() << std::endl;
                // std::cout << "shape of b = " << b_.rows() << " x " << b_.cols() << std::endl;
                // std::cout << "#################################" << std::endl;
            }

            void constructDf()
            {
                // step t: (t = 0, 1, ..., T-1)
                for (int t = 0; t <= T_-1; t++)
                {
                    // append D_
                    Eigen::MatrixXd Dt = Eigen::MatrixXd::Zero(Nu*2, Nu*2);
                    Dt << 0.0, 0.0, 0.0, +1.0, 0.0, 0.0, // vx_{t} <= vx_max
                          0.0, 0.0, 0.0, -1.0, 0.0, 0.0, // vx_{t} >= vx_min
                          0.0, 0.0, 0.0, 0.0, +1.0, 0.0, // vy_{t} <= vy_max
                          0.0, 0.0, 0.0, 0.0, -1.0, 0.0, // vy_{t} >= vy_min
                          0.0, 0.0, 0.0, 0.0, 0.0, +1.0, // omega_{t} <= omega_max
                          0.0, 0.0, 0.0, 0.0, 0.0, -1.0; // omega_{t} >= omega_min
                    D_.conservativeResizeLike(Eigen::MatrixXd::Zero(Nu*2*(t+1), NxNu*T_ + Nx));
                    D_.block(Nu*2*t, NxNu*t, Nu*2, Nu*2) = Dt;

                    // append f_
                    Eigen::MatrixXd ft = Eigen::MatrixXd::Zero(Nu*2, 1);
                    ft << vx_max_, -vx_min_, vy_max_, -vy_min_, omega_max_, -omega_min_;
                    f_.conservativeResize(Nu*2*(t+1), 1);
                    f_.block(Nu*2*t, 0, Nu*2, 1) = ft;
                }
                // [FOR DEBUG] announce D and f for debugging
                // std::cout << "##### Satify State Equation #####" << std::endl;
                // std::cout << "D = " << std::endl << D_ << std::endl;
                // std::cout << "f = " << std::endl << f_ << std::endl;
                // std::cout << "shape of D = " << D_.rows() << " x " << D_.cols() << std::endl;
                // std::cout << "shape of f = " << f_.rows() << " x " << f_.cols() << std::endl;
                // std::cout << "#################################" << std::endl;
            }
    };
} // namespace hqp