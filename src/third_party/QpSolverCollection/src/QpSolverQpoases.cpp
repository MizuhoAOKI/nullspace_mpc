/* Author: Masaki Murooka */

#include <qp_solver_collection/QpSolverOptions.h>

#if ENABLE_QPOASES
#  include <limits>

#  include <qp_solver_collection/QpSolverCollection.h>

#  include <qpOASES.hpp>

using namespace QpSolverCollection;

QpSolverQpoases::QpSolverQpoases()
{
  type_ = QpSolverType::qpOASES;
}

Eigen::VectorXd QpSolverQpoases::solve(int dim_var,
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
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> AC_row_major(dim_eq + dim_ineq, dim_var);
  Eigen::VectorXd bd_min(dim_eq + dim_ineq);
  Eigen::VectorXd bd_max(dim_eq + dim_ineq);
  AC_row_major << A, C;
  bd_min << b, Eigen::VectorXd::Constant(dim_ineq, -1 * std::numeric_limits<double>::infinity());
  bd_max << b, d;

  qpOASES::returnValue status = qpOASES::TERMINAL_LIST_ELEMENT;
  if(!solve_failed_ && !force_initialize_ && qpoases_ && qpoases_->getNV() == dim_var
     && qpoases_->getNC() == dim_eq + dim_ineq)
  {
    status = qpoases_->hotstart(
        // Since Q is a symmetric matrix, row/column-majors are interchangeable
        Q.data(), c.data(), AC_row_major.data(), x_min.data(), x_max.data(), bd_min.data(), bd_max.data(), n_wsr_);
  }
  if(status != qpOASES::SUCCESSFUL_RETURN)
  {
    qpoases_ = std::make_unique<qpOASES::SQProblem>(dim_var, dim_eq + dim_ineq);
    qpoases_->setPrintLevel(qpOASES::PL_LOW);
    status = qpoases_->init(
        // Since Q is a symmetric matrix, row/column-majors are interchangeable
        Q.data(), c.data(), AC_row_major.data(), x_min.data(), x_max.data(), bd_min.data(), bd_max.data(), n_wsr_);
  }

  if(status == qpOASES::SUCCESSFUL_RETURN)
  {
    solve_failed_ = false;
  }
  else
  {
    solve_failed_ = true;
    QSC_WARN_STREAM("[QpSolverQpoases::solve] Failed to solve: " << static_cast<int>(status));
  }

  Eigen::VectorXd sol(dim_var);
  qpoases_->getPrimalSolution(sol.data());
  return sol;
}

namespace QpSolverCollection
{
std::shared_ptr<QpSolver> allocateQpSolverQpoases()
{
  return std::make_shared<QpSolverQpoases>();
}
} // namespace QpSolverCollection
#endif
