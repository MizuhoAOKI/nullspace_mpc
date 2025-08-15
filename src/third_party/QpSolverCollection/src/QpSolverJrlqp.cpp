/* Author: Masaki Murooka */

#include <qp_solver_collection/QpSolverOptions.h>

#if ENABLE_JRLQP
#  include <limits>

#  include <qp_solver_collection/QpSolverCollection.h>

#  include <jrl-qp/GoldfarbIdnaniSolver.h>
#  include <jrl-qp/utils/enumsIO.h>

using namespace QpSolverCollection;

QpSolverJrlqp::QpSolverJrlqp()
{
  type_ = QpSolverType::JRLQP;
  jrlqp_ = std::make_unique<jrl::qp::GoldfarbIdnaniSolver>();
}

Eigen::VectorXd QpSolverJrlqp::solve(int dim_var,
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
  Eigen::VectorXd bd_min(dim_eq + dim_ineq);
  Eigen::VectorXd bd_max(dim_eq + dim_ineq);
  AC << A, C;
  bd_min << b, Eigen::VectorXd::Constant(dim_ineq, -1 * std::numeric_limits<double>::infinity());
  bd_max << b, d;

  jrlqp_->resize(dim_var, dim_eq + dim_ineq, true);

  if(solve_failed_)
  {
    jrlqp_->resetActiveSet();
  }
  else
  {
    jrl::qp::SolverOptions solver_option;
    solver_option.warmStart(true);
    jrlqp_->options(solver_option);
  }

  jrl::qp::TerminationStatus status = jrlqp_->solve(Q, c, AC.transpose(), bd_min, bd_max, x_min, x_max);

  if(status == jrl::qp::TerminationStatus::SUCCESS)
  {
    solve_failed_ = false;
  }
  else
  {
    solve_failed_ = true;
    QSC_WARN_STREAM("[QpSolverJrlqp::solve] Failed to solve: " << status);
  }

  return jrlqp_->solution();
}

namespace QpSolverCollection
{
std::shared_ptr<QpSolver> allocateQpSolverJrlqp()
{
  return std::make_shared<QpSolverJrlqp>();
}
} // namespace QpSolverCollection
#endif
