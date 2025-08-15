/* Author: Masaki Murooka */

#include <qp_solver_collection/QpSolverOptions.h>

#if ENABLE_LSSOL
#  include <limits>
#  include <sstream>

#  include <qp_solver_collection/QpSolverCollection.h>

#  include <eigen-lssol/LSSOL_QP.h>

using namespace QpSolverCollection;

QpSolverLssol::QpSolverLssol()
{
  type_ = QpSolverType::LSSOL;
  lssol_ = std::make_unique<Eigen::LSSOL_QP>();
}

Eigen::VectorXd QpSolverLssol::solve(int dim_var,
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

  lssol_->resize(dim_var, dim_eq + dim_ineq, Eigen::lssol::QP2);

  lssol_->persistence(!solve_failed_);
  lssol_->warm(!solve_failed_);

  lssol_->solve(x_min, x_max, Q, c, AC, bd_min, bd_max);

  if(lssol_->inform() == Eigen::lssol::STRONG_MINIMUM)
  {
    solve_failed_ = false;
  }
  else
  {
    solve_failed_ = true;
    std::stringstream sstream;
    lssol_->inform(sstream);
    QSC_WARN_STREAM("[QpSolverLssol::solve] Failed to solve: " << sstream.str());
  }

  return lssol_->result();
}

namespace QpSolverCollection
{
std::shared_ptr<QpSolver> allocateQpSolverLssol()
{
  return std::make_shared<QpSolverLssol>();
}
} // namespace QpSolverCollection
#endif
