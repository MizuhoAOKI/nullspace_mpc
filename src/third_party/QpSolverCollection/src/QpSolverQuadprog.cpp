/* Author: Masaki Murooka */

#include <qp_solver_collection/QpSolverOptions.h>

#if ENABLE_QUADPROG
#  include <qp_solver_collection/QpSolverCollection.h>

#  include <eigen-quadprog/QuadProg.h>

using namespace QpSolverCollection;

QpSolverQuadprog::QpSolverQuadprog()
{
  type_ = QpSolverType::QuadProg;
  quadprog_ = std::make_unique<Eigen::QuadProgDense>();
}

Eigen::VectorXd QpSolverQuadprog::solve(int dim_var,
                                        int dim_eq,
                                        int dim_ineq,
                                        Eigen::Ref<Eigen::MatrixXd> Q,
                                        const Eigen::Ref<const Eigen::VectorXd> & c,
                                        const Eigen::Ref<const Eigen::MatrixXd> & A,
                                        const Eigen::Ref<const Eigen::VectorXd> & b,
                                        const Eigen::Ref<const Eigen::MatrixXd> & C,
                                        const Eigen::Ref<const Eigen::VectorXd> & d,
                                        const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                        const Eigen::Ref<const Eigen::VectorXd> & x_max)
{
  int dim_ineq_with_bound = dim_ineq + 2 * dim_var;
  Eigen::MatrixXd C_with_bound(dim_ineq_with_bound, dim_var);
  Eigen::VectorXd d_with_bound(dim_ineq_with_bound);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_var, dim_var);
  C_with_bound << C, I, -I;
  d_with_bound << d, x_max, -x_min;

  quadprog_->problem(dim_var, dim_eq, dim_ineq_with_bound);
  quadprog_->solve(Q, c, A, b, C_with_bound, d_with_bound);

  if(quadprog_->fail() == 0)
  {
    solve_failed_ = false;
  }
  else
  {
    solve_failed_ = true;
    QSC_WARN_STREAM("[QpSolverQuadprog::solve] Failed to solve: " << quadprog_->fail());
  }

  return quadprog_->result();
}

namespace QpSolverCollection
{
std::shared_ptr<QpSolver> allocateQpSolverQuadprog()
{
  return std::make_shared<QpSolverQuadprog>();
}
} // namespace QpSolverCollection
#endif
