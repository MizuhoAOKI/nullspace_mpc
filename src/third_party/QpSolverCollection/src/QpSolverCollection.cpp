/* Author: Masaki Murooka */

#include <fstream>

#include <qp_solver_collection/QpSolverCollection.h>

using namespace QpSolverCollection;

QpSolverType QpSolverCollection::strToQpSolverType(const std::string & qp_solver_type)
{
  if(qp_solver_type == "Uninitialized")
  {
    return QpSolverType::Uninitialized;
  }
  else if(qp_solver_type == "QLD")
  {
    return QpSolverType::QLD;
  }
  else if(qp_solver_type == "QuadProg")
  {
    return QpSolverType::QuadProg;
  }
  else if(qp_solver_type == "LSSOL")
  {
    return QpSolverType::LSSOL;
  }
  else if(qp_solver_type == "JRLQP")
  {
    return QpSolverType::JRLQP;
  }
  else if(qp_solver_type == "qpOASES")
  {
    return QpSolverType::qpOASES;
  }
  else if(qp_solver_type == "OSQP")
  {
    return QpSolverType::OSQP;
  }
  else if(qp_solver_type == "NASOQ")
  {
    return QpSolverType::NASOQ;
  }
  else if(qp_solver_type == "HPIPM")
  {
    return QpSolverType::HPIPM;
  }
  else if(qp_solver_type == "PROXQP")
  {
    return QpSolverType::PROXQP;
  }
  else if(qp_solver_type == "QPMAD")
  {
    return QpSolverType::QPMAD;
  }
  else
  {
    throw std::runtime_error("[strToQpSolverType] Unsupported QpSolverType name: " + qp_solver_type);
  }
}

void QpCoeff::setup(int dim_var, int dim_eq, int dim_ineq)
{
  dim_var_ = dim_var;
  dim_eq_ = dim_eq;
  dim_ineq_ = dim_ineq;

  obj_mat_.setZero(dim_var, dim_var);
  obj_vec_.setZero(dim_var);
  eq_mat_.setZero(dim_eq, dim_var);
  eq_vec_.setZero(dim_eq);
  ineq_mat_.setZero(dim_ineq, dim_var);
  ineq_vec_.setZero(dim_ineq);
  x_min_.setConstant(dim_var, std::numeric_limits<double>::lowest());
  x_max_.setConstant(dim_var, std::numeric_limits<double>::max());
}

void QpCoeff::printInfo(bool, const std::string & header) const
{
  QSC_INFO_STREAM(header << "dim_var: " << dim_var_ << ", dim_eq: " << dim_eq_ << ", dim_ineq: " << dim_ineq_);
}

void QpCoeff::dump(std::ofstream & ofs) const
{
  ofs << "dim_var: " << dim_var_ << std::endl;
  ofs << "dim_eq: " << dim_eq_ << std::endl;
  ofs << "dim_ineq: " << dim_ineq_ << std::endl;
  ofs << "obj_mat:\n" << obj_mat_ << std::endl;
  ofs << "obj_vec:\n" << obj_vec_.transpose() << std::endl;
  ofs << "eq_mat:\n" << eq_mat_ << std::endl;
  ofs << "eq_vec:\n" << eq_vec_.transpose() << std::endl;
  ofs << "ineq_mat:\n" << ineq_mat_ << std::endl;
  ofs << "ineq_vec:\n" << ineq_vec_.transpose() << std::endl;
  ofs << "x_min:\n" << x_min_.transpose() << std::endl;
  ofs << "x_max:\n" << x_max_.transpose() << std::endl;
}

void QpSolver::printInfo(bool, const std::string & header) const
{
  QSC_INFO_STREAM(header << "QP solver: " << std::to_string(type_));
}

Eigen::VectorXd QpSolver::solve(QpCoeff & qp_coeff)
{
  return solve(qp_coeff.dim_var_, qp_coeff.dim_eq_, qp_coeff.dim_ineq_, qp_coeff.obj_mat_, qp_coeff.obj_vec_,
               qp_coeff.eq_mat_, qp_coeff.eq_vec_, qp_coeff.ineq_mat_, qp_coeff.ineq_vec_, qp_coeff.x_min_,
               qp_coeff.x_max_);
}

QpSolverType QpSolverCollection::getAnyQpSolverType()
{
  if(ENABLE_QLD)
  {
    return QpSolverType::QLD;
  }
  else if(ENABLE_QUADPROG)
  {
    return QpSolverType::QuadProg;
  }
  else if(ENABLE_LSSOL)
  {
    return QpSolverType::LSSOL;
  }
  else if(ENABLE_JRLQP)
  {
    return QpSolverType::JRLQP;
  }
  else if(ENABLE_QPOASES)
  {
    return QpSolverType::qpOASES;
  }
  else if(ENABLE_OSQP)
  {
    return QpSolverType::OSQP;
  }
  else if(ENABLE_NASOQ)
  {
    return QpSolverType::NASOQ;
  }
  else if(ENABLE_HPIPM)
  {
    return QpSolverType::HPIPM;
  }
  else if(ENABLE_PROXQP)
  {
    return QpSolverType::PROXQP;
  }
  else if(ENABLE_QPMAD)
  {
    return QpSolverType::QPMAD;
  }
  else
  {
    throw std::runtime_error("[getAnyQpSolverType] No QP solver is enabled.");
    return QpSolverType::Uninitialized;
  }
}

bool QpSolverCollection::isQpSolverEnabled(const QpSolverType & qp_solver_type)
{
  if(qp_solver_type == QpSolverType::Any)
  {
    return getAnyQpSolverType() != QpSolverType::Uninitialized;
  }
  else if(qp_solver_type == QpSolverType::QLD)
  {
    return ENABLE_QLD;
  }
  else if(qp_solver_type == QpSolverType::QuadProg)
  {
    return ENABLE_QUADPROG;
  }
  else if(qp_solver_type == QpSolverType::LSSOL)
  {
    return ENABLE_LSSOL;
  }
  else if(qp_solver_type == QpSolverType::JRLQP)
  {
    return ENABLE_JRLQP;
  }
  else if(qp_solver_type == QpSolverType::qpOASES)
  {
    return ENABLE_QPOASES;
  }
  else if(qp_solver_type == QpSolverType::OSQP)
  {
    return ENABLE_OSQP;
  }
  else if(qp_solver_type == QpSolverType::NASOQ)
  {
    return ENABLE_NASOQ;
  }
  else if(qp_solver_type == QpSolverType::HPIPM)
  {
    return ENABLE_HPIPM;
  }
  else if(qp_solver_type == QpSolverType::PROXQP)
  {
    return ENABLE_PROXQP;
  }
  else if(qp_solver_type == QpSolverType::QPMAD)
  {
    return ENABLE_QPMAD;
  }
  else
  {
    QSC_ERROR_STREAM("[isQpSolverEnabled] Unsupported QP solver: " << std::to_string(static_cast<int>(qp_solver_type)));
    return false;
  }
}

namespace QpSolverCollection
{
#if ENABLE_QLD
std::shared_ptr<QpSolver> allocateQpSolverQld();
#endif
#if ENABLE_QUADPROG
std::shared_ptr<QpSolver> allocateQpSolverQuadprog();
#endif
#if ENABLE_LSSOL
std::shared_ptr<QpSolver> allocateQpSolverLssol();
#endif
#if ENABLE_JRLQP
std::shared_ptr<QpSolver> allocateQpSolverJrlqp();
#endif
#if ENABLE_QPOASES
std::shared_ptr<QpSolver> allocateQpSolverQpoases();
#endif
#if ENABLE_OSQP
std::shared_ptr<QpSolver> allocateQpSolverOsqp();
#endif
#if ENABLE_NASOQ
std::shared_ptr<QpSolver> allocateQpSolverNasoq();
#endif
#if ENABLE_HPIPM
std::shared_ptr<QpSolver> allocateQpSolverHpipm();
#endif
#if ENABLE_PROXQP
std::shared_ptr<QpSolver> allocateQpSolverProxqp();
#endif
#if ENABLE_QPMAD
std::shared_ptr<QpSolver> allocateQpSolverQpmad();
#endif
} // namespace QpSolverCollection

std::shared_ptr<QpSolver> QpSolverCollection::allocateQpSolver(const QpSolverType & qp_solver_type)
{
  std::shared_ptr<QpSolver> qp;

  if(qp_solver_type == QpSolverType::Any)
  {
    return allocateQpSolver(getAnyQpSolverType());
  }
  else if(qp_solver_type == QpSolverType::QLD)
  {
#if ENABLE_QLD
    qp = allocateQpSolverQld();
#endif
  }
  else if(qp_solver_type == QpSolverType::QuadProg)
  {
#if ENABLE_QUADPROG
    qp = allocateQpSolverQuadprog();
#endif
  }
  else if(qp_solver_type == QpSolverType::LSSOL)
  {
#if ENABLE_LSSOL
    qp = allocateQpSolverLssol();
#endif
  }
  else if(qp_solver_type == QpSolverType::JRLQP)
  {
#if ENABLE_JRLQP
    qp = allocateQpSolverJrlqp();
#endif
  }
  else if(qp_solver_type == QpSolverType::qpOASES)
  {
#if ENABLE_QPOASES
    qp = allocateQpSolverQpoases();
#endif
  }
  else if(qp_solver_type == QpSolverType::OSQP)
  {
#if ENABLE_OSQP
    qp = allocateQpSolverOsqp();
#endif
  }
  else if(qp_solver_type == QpSolverType::NASOQ)
  {
#if ENABLE_NASOQ
    qp = allocateQpSolverNasoq();
#endif
  }
  else if(qp_solver_type == QpSolverType::HPIPM)
  {
#if ENABLE_HPIPM
    qp = allocateQpSolverHpipm();
#endif
  }
  else if(qp_solver_type == QpSolverType::PROXQP)
  {
#if ENABLE_PROXQP
    qp = allocateQpSolverProxqp();
#endif
  }
  else if(qp_solver_type == QpSolverType::QPMAD)
  {
#if ENABLE_QPMAD
    qp = allocateQpSolverQpmad();
#endif
  }
  else
  {
    QSC_ERROR_STREAM("[allocateQpSolver] Unsupported QP solver: " << std::to_string(static_cast<int>(qp_solver_type)));
  }

  if(!qp)
  {
    QSC_ERROR_STREAM("[allocateQpSolver] Failed to initialize QP solver: " << std::to_string(qp_solver_type));
  }

  return qp;
}
