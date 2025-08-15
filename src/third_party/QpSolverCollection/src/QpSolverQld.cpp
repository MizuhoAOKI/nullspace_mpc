/* Author: Masaki Murooka */

#include <qp_solver_collection/QpSolverOptions.h>

#if ENABLE_QLD
#  include <qp_solver_collection/QpSolverCollection.h>

#  include <eigen-qld/QLD.h>

using namespace QpSolverCollection;

QpSolverQld::QpSolverQld()
{
  type_ = QpSolverType::QLD;
  qld_ = std::make_unique<Eigen::QLDDirect>();
}

Eigen::VectorXd QpSolverQld::solve(int dim_var,
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
  Eigen::MatrixXd AC(dim_eq + dim_ineq, dim_var);
  Eigen::VectorXd bd(dim_eq + dim_ineq);
  AC << -A, -C;
  bd << b, d;

  qld_->problem(dim_var, dim_eq, dim_ineq);
  qld_->solve(Q, c, AC, bd, x_min, x_max, dim_eq);

  if(qld_->fail() == 0)
  {
    solve_failed_ = false;
  }
  else
  {
    solve_failed_ = true;
    QSC_WARN_STREAM("[QpSolverQld::solve] Failed to solve: " << qld_->fail());
  }

  return qld_->result();
}

namespace QpSolverCollection
{
std::shared_ptr<QpSolver> allocateQpSolverQld()
{
  return std::make_shared<QpSolverQld>();
}
} // namespace QpSolverCollection
#endif
