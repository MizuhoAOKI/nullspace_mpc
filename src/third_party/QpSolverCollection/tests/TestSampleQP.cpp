/* Author: Masaki Murooka */

#include <limits>

#include <gtest/gtest.h>

#include <qp_solver_collection/QpSolverCollection.h>

using QpSolverCollection::QpCoeff;
using QpSolverCollection::QpSolverType;

void solveOneQP(const QpCoeff & qp_coeff, const Eigen::VectorXd & x_gt)
{
  // clang-format off
  const std::vector<QpSolverType> qp_solver_type_list = {
      QpSolverType::QLD,
      QpSolverType::QuadProg,
      QpSolverType::LSSOL,
      QpSolverType::JRLQP,
      QpSolverType::qpOASES,
      QpSolverType::OSQP,
      QpSolverType::NASOQ,
      QpSolverType::HPIPM,
      QpSolverType::PROXQP,
      QpSolverType::QPMAD
  };
  // clang-format on
  for(const auto & qp_solver_type : qp_solver_type_list)
  {
    if(!QpSolverCollection::isQpSolverEnabled(qp_solver_type))
    {
      std::cout << "[solveOneQP] Skip QP solver " << std::to_string(qp_solver_type) << " because it is not enabled."
                << std::endl;
      continue;
    }

    auto qp_solver = QpSolverCollection::allocateQpSolver(qp_solver_type);
    EXPECT_TRUE(qp_solver) << "Instantiation of QP solver " << std::to_string(qp_solver_type) << " failed";
    if(!qp_solver)
    {
      continue;
    }

    QpCoeff qp_coeff_copied = qp_coeff;
    Eigen::VectorXd x_opt = qp_solver->solve(qp_coeff_copied);

    double thre = 1e-6;
    if(qp_solver_type == QpSolverType::OSQP)
    {
      thre = 1e-3;
    }
    else if(qp_solver_type == QpSolverType::PROXQP)
    {
      // Set proxqp_->settings.eps_abs to 1e-8 to satisfy thre of 1e-6
      thre = 1e-4;
    }
    EXPECT_LT((x_opt - x_gt).norm(), thre)
        << "QP solution of " << std::to_string(qp_solver_type) << " is incorrect:\n"
        << "  solution: " << x_opt.transpose() << "\n  ground truth: " << x_gt.transpose()
        << "\n  error: " << (x_opt - x_gt).norm() << std::endl;
  }
}

TEST(TestSampleQP, IdentityObj)
{
  // QP coefficients are copied from https://github.com/jrl-umi3218/eigen-qld/blob/master/tests/QPTest.cpp
  int dim_var = 6;
  int dim_eq = 3;
  int dim_ineq = 2;
  QpCoeff qp_coeff;
  qp_coeff.setup(dim_var, dim_eq, dim_ineq);
  qp_coeff.obj_mat_.setIdentity();
  qp_coeff.obj_vec_ << 1., 2., 3., 4., 5., 6.;
  qp_coeff.eq_mat_ << 1., -1., 1., 0., 3., 1., -1., 0., -3., -4., 5., 6., 2., 5., 3., 0., 1., 0.;
  qp_coeff.eq_vec_ << 1., 2., 3.;
  qp_coeff.ineq_mat_ << 0., 1., 0., 1., 2., -1., -1., 0., 2., 1., 1., 0.;
  qp_coeff.ineq_vec_ << -1., 2.5;
  qp_coeff.x_min_ << -1000., -10000., 0., -1000., -1000., -1000.;
  qp_coeff.x_max_ << 10000., 100., 1.5, 100., 100., 1000.;
  Eigen::VectorXd x_gt(dim_var);
  x_gt << 1.7975426, -0.3381487, 0.1633880, -4.9884023, 0.6054943, -3.1155623;

  solveOneQP(qp_coeff, x_gt);
}

TEST(TestSampleQP, Unconstrained)
{
  // QP coefficients are copied from
  // https://github.com/robotology/osqp-eigen/blob/83812bd0a56bbb656cac7016b307845e4a0ed11e/tests/QPTest.cpp#L13-L44
  int dim_var = 2;
  int dim_eq = 0;
  int dim_ineq = 0;
  QpCoeff qp_coeff;
  qp_coeff.setup(dim_var, dim_eq, dim_ineq);
  qp_coeff.obj_mat_ << 3, 2, 2, 4;
  qp_coeff.obj_vec_ << 3, 1;
  qp_coeff.x_min_.setConstant(-1 * std::numeric_limits<double>::infinity());
  qp_coeff.x_max_.setConstant(std::numeric_limits<double>::infinity());
  Eigen::VectorXd x_gt(dim_var);
  x_gt << -1.2500, 0.3750;

  solveOneQP(qp_coeff, x_gt);
}

TEST(TestSampleQP, OnlyEqConst)
{
  // QP coefficients are copied from https://cvxopt.org/examples/tutorial/qp.html
  int dim_var = 2;
  int dim_eq = 1;
  int dim_ineq = 0;
  QpCoeff qp_coeff;
  qp_coeff.setup(dim_var, dim_eq, dim_ineq);
  qp_coeff.obj_mat_ << 2, 0.5, 0.5, 1;
  qp_coeff.obj_vec_ << 1, 1;
  qp_coeff.eq_mat_ << 1, 1;
  qp_coeff.eq_vec_ << 1;
  qp_coeff.x_min_.setZero();
  qp_coeff.x_max_.setConstant(std::numeric_limits<double>::infinity());
  Eigen::VectorXd x_gt(dim_var);
  x_gt << 0.25, 0.75;

  solveOneQP(qp_coeff, x_gt);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
