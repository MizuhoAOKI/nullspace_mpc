/* Author: Masaki Murooka */

#include <qp_solver_collection/QpSolverOptions.h>

#if ENABLE_PROXQP
#  include <qp_solver_collection/QpSolverCollection.h>

#  include <proxsuite/proxqp/dense/dense.hpp>

using namespace QpSolverCollection;

QpSolverProxqp::QpSolverProxqp()
{
  type_ = QpSolverType::PROXQP;
}

Eigen::VectorXd QpSolverProxqp::solve(int dim_var,
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
  int dim_ineq_with_bound = dim_ineq + dim_var;
  if(!(proxqp_ && proxqp_->model.dim == dim_var && proxqp_->model.n_eq == dim_eq
       && proxqp_->model.n_in == dim_ineq_with_bound))
  {
    proxqp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(dim_var, dim_eq, dim_ineq_with_bound);
  }

  Eigen::MatrixXd C_with_bound(dim_ineq_with_bound, dim_var);
  Eigen::VectorXd d_with_bound_min(dim_ineq_with_bound);
  Eigen::VectorXd d_with_bound_max(dim_ineq_with_bound);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_var, dim_var);
  C_with_bound << C, I;
  d_with_bound_min << Eigen::VectorXd::Constant(dim_ineq, -1 * std::numeric_limits<double>::infinity()), x_min;
  d_with_bound_max << d, x_max;

  proxqp_->update(Q, c, A, b, C_with_bound, d_with_bound_min, d_with_bound_max);
  proxqp_->solve();

  if(proxqp_->results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED)
  {
    solve_failed_ = false;
  }
  else
  {
    solve_failed_ = true;
    QSC_WARN_STREAM("[QpSolverProxqp::solve] Failed to solve: " << static_cast<int>(proxqp_->results.info.status));
  }

  return proxqp_->results.x;
}

namespace QpSolverCollection
{
std::shared_ptr<QpSolver> allocateQpSolverProxqp()
{
  return std::make_shared<QpSolverProxqp>();
}
} // namespace QpSolverCollection
#endif
