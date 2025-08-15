# [QpSolverCollection](https://github.com/isri-aist/QpSolverCollection)
Unified C++ interface for quadratic programming solvers

[![CI](https://github.com/isri-aist/QpSolverCollection/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/QpSolverCollection/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/QpSolverCollection/)

## Features
- Unified C++ interface to many QP solvers
- Can be built as a standalone package or ROS package
- High portability decoupled from each QP solver by [Pimpl technique](https://en.cppreference.com/w/cpp/language/pimpl)

## Supported QP solvers
- [QLD](https://github.com/jrl-umi3218/eigen-qld)
- [QuadProg](https://github.com/jrl-umi3218/eigen-quadprog)
- [JRLQP](https://github.com/jrl-umi3218/jrl-qp)
- [qpOASES](https://github.com/coin-or/qpOASES)
- [OSQP](https://osqp.org/)
- [NASOQ](https://nasoq.github.io/)
- [HPIPM](https://github.com/giaf/hpipm)
- [ProxQP](https://github.com/Simple-Robotics/proxsuite)
- [qpmad](https://github.com/asherikov/qpmad)
- [LSSOL](https://gite.lirmm.fr/multi-contact/eigen-lssol) (private)

## Installation

### Installation procedure
It is assumed that ROS is installed.

1. Install the QP solver you wish to use according to [this section](https://github.com/isri-aist/QpSolverCollection#qp-solver-installation). You can skip installing QP solvers that you do not use.

2. Setup catkin workspace.
```bash
$ mkdir -p ~/ros/ws_qp_solver_collection/src
$ cd ~/ros/ws_qp_solver_collection
$ wstool init src
$ wstool set -t src isri-aist/QpSolverCollection git@github.com:isri-aist/QpSolverCollection.git --git -y
$ wstool update -t src
```

3. Install dependent packages.
```bash
$ source /opt/ros/${ROS_DISTRO}/setup.bash
$ rosdep install -y -r --from-paths src --ignore-src
```

4. Build a package.
```bash
$ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo <qp-solver-flags> --catkin-make-args all tests
```
See [this section](https://github.com/isri-aist/QpSolverCollection#qp-solver-installation) for `<qp-solver-flags>`.

### QP solver installation
As all supported QP solvers are installed in [CI](https://github.com/isri-aist/QpSolverCollection/blob/master/.github/workflows/ci.yaml), please refer to the installation procedure.  
Please refer to the license specified in each QP solver when using it.

#### QLD
Install [eigen-qld](https://github.com/jrl-umi3218/eigen-qld).  
Add `-DENABLE_QLD=ON` to the catkin build command (i.e., `<qp-solver-flags>`).

#### QuadProg
Install [eigen-quadprog](https://github.com/jrl-umi3218/eigen-quadprog).  
Add `-DENABLE_QUADPROG=ON` to the catkin build command (i.e., `<qp-solver-flags>`).

#### JRLQP
Install master branch of [jrl-qp](https://github.com/jrl-umi3218/jrl-qp).
Add `-DENABLE_JRLQP=ON` to the catkin build command (i.e., `<qp-solver-flags>`).

#### qpOASES
Install master branch of [qpOASES](https://github.com/coin-or/qpOASES) with `-DBUILD_SHARED_LIBS=ON` cmake option.  
Add `-DENABLE_QPOASES=ON` to the catkin build command (i.e., `<qp-solver-flags>`).
Also, add `-DQPOASES_INCLUDE_DIR=<path to qpOASES.hpp>` and `-DQPOASES_LIBRARY_DIR=<path to libqpOASES.so>` to the catkin build command.

#### OSQP
Install master branch of [osqp](https://github.com/osqp/osqp) and [osqp-eigen](https://github.com/robotology/osqp-eigen).  
Add `-DENABLE_OSQP=ON` to the catkin build command (i.e., `<qp-solver-flags>`).

#### NASOQ
Install [cmake-install branch](https://github.com/mmurooka/nasoq/tree/cmake-install) of nasoq.  
Add `-DENABLE_NASOQ=ON` to the catkin build command (i.e., `<qp-solver-flags>`).

#### HPIPM
Install master branch of [blasfeo](https://github.com/giaf/blasfeo) and [hpipm](https://github.com/giaf/hpipm).  
Add `/opt/blasfeo/lib` and `/opt/hpipm/lib` to the environment variable `LD_LIBRARY_PATH`.  
Add `-DENABLE_HPIPM=ON` to the catkin build command (i.e., `<qp-solver-flags>`).

#### ProxQP
Install main branch of [proxsuite](https://github.com/Simple-Robotics/proxsuite).  
Add `-DENABLE_PROXQP=ON` to the catkin build command (i.e., `<qp-solver-flags>`).

#### qpmad
Install master branch of [qpmad](https://github.com/asherikov/qpmad).  
Add `-DENABLE_QPMAD=ON` to the catkin build command (i.e., `<qp-solver-flags>`).

#### LSSOL (private)
Install [eigen-lssol](https://gite.lirmm.fr/multi-contact/eigen-lssol).  
Add `-DENABLE_LSSOL=ON` to the catkin build command (i.e., `<qp-solver-flags>`).

## How to use
See [documentation](https://isri-aist.github.io/QpSolverCollection/doxygen/classQpSolverCollection_1_1QpSolver.html) and [test](https://github.com/isri-aist/QpSolverCollection/blob/master/tests/TestSampleQP.cpp) for examples of solving QP problems.

The following is a simple sample.
```cpp
// sample.cpp

#include <qp_solver_collection/QpSolverCollection.h>

int main()
{
  int dim_var = 2;
  int dim_eq = 1;
  int dim_ineq = 0;
  QpSolverCollection::QpCoeff qp_coeff;
  qp_coeff.setup(dim_var, dim_eq, dim_ineq);
  qp_coeff.obj_mat_ << 2.0, 0.5, 0.5, 1.0;
  qp_coeff.obj_vec_ << 1.0, 1.0;
  qp_coeff.eq_mat_ << 1.0, 1.0;
  qp_coeff.eq_vec_ << 1.0;
  qp_coeff.x_min_.setZero();
  qp_coeff.x_max_.setConstant(1000.0);

  auto qp_solver = QpSolverCollection::allocateQpSolver(QpSolverCollection::QpSolverType::Any);
  Eigen::VectorXd solution = qp_solver->solve(qp_coeff);
  std::cout << "solution: " << solution.transpose() << std::endl;

  return 0;
}
```

In addition to building a sample in a catkin package, you can also build it standalone as follows.
```bash
$ g++ sample.cpp `pkg-config --cflags qp_solver_collection` `pkg-config --libs qp_solver_collection`
```
