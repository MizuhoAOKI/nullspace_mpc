/* Author: Masaki Murooka */

#include <qp_solver_collection/QpSolverOptions.h>

#if ENABLE_QPMAD
#  include <qp_solver_collection/QpSolverCollection.h>

#  include <qpmad/solver.h>

using namespace QpSolverCollection;

QpSolverQpmad::QpSolverQpmad()
{
  type_ = QpSolverType::QPMAD;
  qpmad_ = std::make_unique<qpmad::Solver>();
}

Eigen::VectorXd QpSolverQpmad::solve(int dim_var,
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

  Eigen::VectorXd sol;
  qpmad::Solver::ReturnStatus status = qpmad_->solve(sol, Q, c, x_min, x_max, AC, bd_min, bd_max);

  if(status == qpmad::Solver::OK)
  {
    solve_failed_ = false;
  }
  else
  {
    solve_failed_ = true;
    QSC_WARN_STREAM("[QpSolverQpmad::solve] Failed to solve: " << static_cast<int>(status));
  }

  return sol;
}

namespace QpSolverCollection
{
std::shared_ptr<QpSolver> allocateQpSolverQpmad()
{
  return std::make_shared<QpSolverQpmad>();
}
} // namespace QpSolverCollection
#endif
