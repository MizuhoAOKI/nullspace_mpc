/* Author: Masaki Murooka */

#pragma once

#include <chrono>
#include <fstream>
#include <memory>

#include <Eigen/SparseCore>

#include <qp_solver_collection/QpSolverOptions.h>

#ifdef QP_SOLVER_COLLECTION_STANDALONE
#  include <iostream>
#  define QSC_ERROR_STREAM(x) std::cerr << x << "\n"
#  define QSC_WARN_STREAM(x) std::cerr << x << "\n"
#  define QSC_INFO_STREAM(x) std::cout << x << "\n"
#else
#  include <ros/console.h>
#  define QSC_ERROR_STREAM ROS_ERROR_STREAM
#  define QSC_WARN_STREAM ROS_WARN_STREAM
#  define QSC_INFO_STREAM ROS_INFO_STREAM
#endif

namespace Eigen
{
class QLDDirect;
class QuadProgDense;
class LSSOL_QP;
} // namespace Eigen

namespace jrl
{
namespace qp
{
class GoldfarbIdnaniSolver;
} // namespace qp
} // namespace jrl

namespace qpOASES
{
class SQProblem;
} // namespace qpOASES

namespace OsqpEigen
{
class Solver;
} // namespace OsqpEigen

struct d_dense_qp_dim;
struct d_dense_qp;
struct d_dense_qp_sol;
struct d_dense_qp_ipm_arg;
struct d_dense_qp_ipm_ws;

namespace proxsuite
{
namespace proxqp
{
namespace dense
{
template<typename T>
class QP;
}
} // namespace proxqp
} // namespace proxsuite

namespace qpmad
{
template<typename t_Scalar, int... t_Parameters>
class SolverTemplate;
using Solver = SolverTemplate<double, Eigen::Dynamic, 1, Eigen::Dynamic>;
} // namespace qpmad

namespace QpSolverCollection
{
/** \brief QP solver type. */
enum class QpSolverType
{
  Any = -2,
  Uninitialized = -1,
  QLD = 0,
  QuadProg,
  LSSOL,
  JRLQP,
  qpOASES,
  OSQP,
  NASOQ,
  HPIPM,
  PROXQP,
  QPMAD
};

/*! \brief Convert std::string to QpSolverType. */
QpSolverType strToQpSolverType(const std::string & qp_solver_type);
} // namespace QpSolverCollection

namespace std
{
using QpSolverType = QpSolverCollection::QpSolverType;

inline string to_string(QpSolverType qp_solver_type)
{
  switch(qp_solver_type)
  {
    case QpSolverType::QLD:
      return "QpSolverType::QLD";
    case QpSolverType::QuadProg:
      return "QpSolverType::QuadProg";
    case QpSolverType::LSSOL:
      return "QpSolverType::LSSOL";
    case QpSolverType::JRLQP:
      return "QpSolverType::JRLQP";
    case QpSolverType::qpOASES:
      return "QpSolverType::qpOASES";
    case QpSolverType::OSQP:
      return "QpSolverType::OSQP";
    case QpSolverType::NASOQ:
      return "QpSolverType::NASOQ";
    case QpSolverType::HPIPM:
      return "QpSolverType::HPIPM";
    case QpSolverType::PROXQP:
      return "QpSolverType::PROXQP";
    case QpSolverType::QPMAD:
      return "QpSolverType::QPMAD";
    default:
      QSC_ERROR_STREAM("[QpSolverType] Unsupported value: " << std::to_string(static_cast<int>(qp_solver_type)));
  }

  return "";
}
} // namespace std

namespace QpSolverCollection
{
/** \brief Class of QP coefficient. */
class QpCoeff
{
public:
  /** \brief Constructor. */
  QpCoeff() {}

  /** \brief Setup the coefficients with filling zero.
      \param dim_var dimension of decision variable
      \param dim_eq dimension of equality constraint
      \param dim_ineq dimension of inequality constraint
  */
  void setup(int dim_var, int dim_eq, int dim_ineq);

  /** \brief Print information. */
  void printInfo(bool verbose = false, const std::string & header = "") const;

  /** \brief Dump coefficients. */
  void dump(std::ofstream & ofs) const;

public:
  //! Dimension of decision variable
  int dim_var_ = 0;

  //! Dimension of equality constraint
  int dim_eq_ = 0;

  //! Dimension of inequality constraint
  int dim_ineq_ = 0;

  //! Objective matrix (corresponding to \f$\boldsymbol{Q}\f$ in @ref QpSolver#solve "QpSolver::solve".)
  Eigen::MatrixXd obj_mat_;

  //! Objective vector (corresponding to \f$\boldsymbol{c}\f$ in @ref QpSolver#solve "QpSolver::solve".)
  Eigen::VectorXd obj_vec_;

  //! Equality constraint matrix (corresponding to \f$\boldsymbol{A}\f$ in @ref QpSolver#solve "QpSolver::solve".)
  Eigen::MatrixXd eq_mat_;

  //! Equality constraint vector (corresponding to \f$\boldsymbol{b}\f$ in @ref QpSolver#solve "QpSolver::solve".)
  Eigen::VectorXd eq_vec_;

  //! Inequality constraint matrix (corresponding to \f$\boldsymbol{C}\f$ in @ref QpSolver#solve "QpSolver::solve".)
  Eigen::MatrixXd ineq_mat_;

  //! Inequality constraint vector (corresponding to \f$\boldsymbol{d}\f$ in @ref QpSolver#solve "QpSolver::solve".)
  Eigen::VectorXd ineq_vec_;

  //! Lower bound (corresponding to \f$\boldsymbol{x}_{min}\f$ in @ref QpSolver#solve "QpSolver::solve".)
  Eigen::VectorXd x_min_;

  //! Upper bound (corresponding to \f$\boldsymbol{x}_{max}\f$ in @ref QpSolver#solve "QpSolver::solve".)
  Eigen::VectorXd x_max_;
};

/** \brief Virtual class of QP solver. */
class QpSolver
{
public:
  using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                          std::chrono::high_resolution_clock,
                                          std::chrono::steady_clock>::type;

public:
  /** \brief Constructor. */
  QpSolver() {}

  /** \brief Print information. */
  void printInfo(bool verbose = false, const std::string & header = "") const;

  /** \brief Solve QP.
      \param dim_var dimension of decision variable
      \param dim_eq dimension of equality constraint
      \param dim_ineq dimension of inequality constraint
      \param Q objective matrix (LSSOL requires non-const for Q)
      \param c objective vector
      \param A equality constraint matrix
      \param b equality constraint vector
      \param C inequality constraint matrix
      \param d inequality constraint vector
      \param x_min lower bound
      \param x_max upper bound

      QP is formulated as follows:
      \f{align*}{
      & min_{\boldsymbol{x}} \ \frac{1}{2}{\boldsymbol{x}^T \boldsymbol{Q} \boldsymbol{x}} + {\boldsymbol{c}^T
     \boldsymbol{x}} \\
      & s.t. \ \ \boldsymbol{A} \boldsymbol{x} = \boldsymbol{b} \nonumber \\
      & \phantom{s.t.} \ \ \boldsymbol{C} \boldsymbol{x} \leq \boldsymbol{d} \nonumber \\
      & \phantom{s.t.} \ \ \boldsymbol{x}_{min} \leq \boldsymbol{x} \leq \boldsymbol{x}_{max} \nonumber
      \f}

      \todo Support both-sided inequality constraints (i.e., \f$\boldsymbol{d}_{lower} \leq \boldsymbol{C}
     \boldsymbol{x} \leq \boldsymbol{d}_{upper}\f$). QLD, QuadProg, and NASOQ support only one-sided constraints, while
     LSSOL, JRLQP, QPOASES, OSQP, HPIPM, PROXQP, and QPMAD support both-sided constraints.
  */
  virtual Eigen::VectorXd solve(int dim_var,
                                int dim_eq,
                                int dim_ineq,
                                Eigen::Ref<Eigen::MatrixXd> Q,
                                const Eigen::Ref<const Eigen::VectorXd> & c,
                                const Eigen::Ref<const Eigen::MatrixXd> & A,
                                const Eigen::Ref<const Eigen::VectorXd> & b,
                                const Eigen::Ref<const Eigen::MatrixXd> & C,
                                const Eigen::Ref<const Eigen::VectorXd> & d,
                                const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                const Eigen::Ref<const Eigen::VectorXd> & x_max) = 0;

  /** \brief Solve QP.
      \param qp_coeff QP coefficient
  */
  virtual Eigen::VectorXd solve(QpCoeff & qp_coeff);

  /** \brief Get QP solver type. */
  inline QpSolverType type() const
  {
    return type_;
  }

  /** \brief Get whether it failed to solve the QP. */
  inline bool solveFailed() const
  {
    return solve_failed_;
  }

protected:
  /** \brief QP solver type. */
  QpSolverType type_ = QpSolverType::Uninitialized;

  /** \brief Whether it failed to solve the QP. */
  bool solve_failed_ = false;
};

#if ENABLE_QLD
/** \brief QP solver QLD. */
class QpSolverQld : public QpSolver
{
public:
  /** \brief Constructor. */
  QpSolverQld();

  /** \brief Solve QP. */
  virtual Eigen::VectorXd solve(int dim_var,
                                int dim_eq,
                                int dim_ineq,
                                Eigen::Ref<Eigen::MatrixXd> Q,
                                const Eigen::Ref<const Eigen::VectorXd> & c,
                                const Eigen::Ref<const Eigen::MatrixXd> & A,
                                const Eigen::Ref<const Eigen::VectorXd> & b,
                                const Eigen::Ref<const Eigen::MatrixXd> & C,
                                const Eigen::Ref<const Eigen::VectorXd> & d,
                                const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                const Eigen::Ref<const Eigen::VectorXd> & x_max) override;

protected:
  std::unique_ptr<Eigen::QLDDirect> qld_;
};
#endif

#if ENABLE_QUADPROG
/** \brief QP solver QuadProg. */
class QpSolverQuadprog : public QpSolver
{
public:
  /** \brief Constructor. */
  QpSolverQuadprog();

  /** \brief Solve QP. */
  virtual Eigen::VectorXd solve(int dim_var,
                                int dim_eq,
                                int dim_ineq,
                                Eigen::Ref<Eigen::MatrixXd> Q,
                                const Eigen::Ref<const Eigen::VectorXd> & c,
                                const Eigen::Ref<const Eigen::MatrixXd> & A,
                                const Eigen::Ref<const Eigen::VectorXd> & b,
                                const Eigen::Ref<const Eigen::MatrixXd> & C,
                                const Eigen::Ref<const Eigen::VectorXd> & d,
                                const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                const Eigen::Ref<const Eigen::VectorXd> & x_max) override;

protected:
  std::unique_ptr<Eigen::QuadProgDense> quadprog_;
};
#endif

#if ENABLE_LSSOL
/** \brief QP solver LSSOL. */
class QpSolverLssol : public QpSolver
{
public:
  /** \brief Constructor. */
  QpSolverLssol();

  /** \brief Solve QP. */
  virtual Eigen::VectorXd solve(int dim_var,
                                int dim_eq,
                                int dim_ineq,
                                Eigen::Ref<Eigen::MatrixXd> Q,
                                const Eigen::Ref<const Eigen::VectorXd> & c,
                                const Eigen::Ref<const Eigen::MatrixXd> & A,
                                const Eigen::Ref<const Eigen::VectorXd> & b,
                                const Eigen::Ref<const Eigen::MatrixXd> & C,
                                const Eigen::Ref<const Eigen::VectorXd> & d,
                                const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                const Eigen::Ref<const Eigen::VectorXd> & x_max) override;

protected:
  std::unique_ptr<Eigen::LSSOL_QP> lssol_;
};
#endif

#if ENABLE_JRLQP
/** \brief QP solver JRLQP. */
class QpSolverJrlqp : public QpSolver
{
public:
  /** \brief Constructor. */
  QpSolverJrlqp();

  /** \brief Solve QP. */
  virtual Eigen::VectorXd solve(int dim_var,
                                int dim_eq,
                                int dim_ineq,
                                Eigen::Ref<Eigen::MatrixXd> Q,
                                const Eigen::Ref<const Eigen::VectorXd> & c,
                                const Eigen::Ref<const Eigen::MatrixXd> & A,
                                const Eigen::Ref<const Eigen::VectorXd> & b,
                                const Eigen::Ref<const Eigen::MatrixXd> & C,
                                const Eigen::Ref<const Eigen::VectorXd> & d,
                                const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                const Eigen::Ref<const Eigen::VectorXd> & x_max) override;

protected:
  std::unique_ptr<jrl::qp::GoldfarbIdnaniSolver> jrlqp_;
};
#endif

#if ENABLE_QPOASES
/** \brief QP solver qpOASES.
    \todo Support an efficient interface (QProblemB) dedicated to QP with only box constraints.
*/
class QpSolverQpoases : public QpSolver
{
public:
  /** \brief Constructor. */
  QpSolverQpoases();

  /** \brief Solve QP. */
  virtual Eigen::VectorXd solve(int dim_var,
                                int dim_eq,
                                int dim_ineq,
                                Eigen::Ref<Eigen::MatrixXd> Q,
                                const Eigen::Ref<const Eigen::VectorXd> & c,
                                const Eigen::Ref<const Eigen::MatrixXd> & A,
                                const Eigen::Ref<const Eigen::VectorXd> & b,
                                const Eigen::Ref<const Eigen::MatrixXd> & C,
                                const Eigen::Ref<const Eigen::VectorXd> & d,
                                const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                const Eigen::Ref<const Eigen::VectorXd> & x_max) override;

public:
  int n_wsr_ = 10000;

  /** \brief Whether to initialize each time instead of doing a warm start.

      \note Warm start did not give good results.
  */
  bool force_initialize_ = true;

protected:
  std::unique_ptr<qpOASES::SQProblem> qpoases_;
};
#endif

#if ENABLE_OSQP
/** \brief QP solver OSQP.
    \todo Set without going through a dense matrix.
 */
class QpSolverOsqp : public QpSolver
{
public:
  /** \brief Constructor. */
  QpSolverOsqp();

  /** \brief Solve QP. */
  virtual Eigen::VectorXd solve(int dim_var,
                                int dim_eq,
                                int dim_ineq,
                                Eigen::Ref<Eigen::MatrixXd> Q,
                                const Eigen::Ref<const Eigen::VectorXd> & c,
                                const Eigen::Ref<const Eigen::MatrixXd> & A,
                                const Eigen::Ref<const Eigen::VectorXd> & b,
                                const Eigen::Ref<const Eigen::MatrixXd> & C,
                                const Eigen::Ref<const Eigen::VectorXd> & d,
                                const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                const Eigen::Ref<const Eigen::VectorXd> & x_max) override;

public:
  /** \brief Whether to initialize each time instead of doing a warm start.

      \note Warm start did not give good results.
  */
  bool force_initialize_ = true;

protected:
  std::unique_ptr<OsqpEigen::Solver> osqp_;

  Eigen::SparseMatrix<double> Q_sparse_;
  Eigen::VectorXd c_;
  Eigen::SparseMatrix<double> AC_with_bound_sparse_;
  Eigen::VectorXd bd_with_bound_min_;
  Eigen::VectorXd bd_with_bound_max_;

  double sparse_duration_ = 0; // [ms]
};
#endif

#if ENABLE_NASOQ
/** \brief QP solver NASOQ.
    \todo Set without going through a dense matrix.
*/
class QpSolverNasoq : public QpSolver
{
public:
  /** \brief Constructor. */
  QpSolverNasoq();

  /** \brief Solve QP. */
  virtual Eigen::VectorXd solve(int dim_var,
                                int dim_eq,
                                int dim_ineq,
                                Eigen::Ref<Eigen::MatrixXd> Q,
                                const Eigen::Ref<const Eigen::VectorXd> & c,
                                const Eigen::Ref<const Eigen::MatrixXd> & A,
                                const Eigen::Ref<const Eigen::VectorXd> & b,
                                const Eigen::Ref<const Eigen::MatrixXd> & C,
                                const Eigen::Ref<const Eigen::VectorXd> & d,
                                const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                const Eigen::Ref<const Eigen::VectorXd> & x_max) override;

protected:
  Eigen::SparseMatrix<double, Eigen::ColMajor, int> Q_sparse_;
  Eigen::SparseMatrix<double, Eigen::ColMajor, int> A_sparse_;
  Eigen::SparseMatrix<double, Eigen::ColMajor, int> C_with_bound_sparse_;

  double sparse_duration_ = 0; // [ms]
};
#endif

#if ENABLE_HPIPM
/** \brief QP solver HPIPM. */
class QpSolverHpipm : public QpSolver
{
public:
  /** \brief Constructor. */
  QpSolverHpipm();

  /** \brief Destructor. */
  ~QpSolverHpipm();

  /** \brief Solve QP. */
  virtual Eigen::VectorXd solve(int dim_var,
                                int dim_eq,
                                int dim_ineq,
                                Eigen::Ref<Eigen::MatrixXd> Q,
                                const Eigen::Ref<const Eigen::VectorXd> & c,
                                const Eigen::Ref<const Eigen::MatrixXd> & A,
                                const Eigen::Ref<const Eigen::VectorXd> & b,
                                const Eigen::Ref<const Eigen::MatrixXd> & C,
                                const Eigen::Ref<const Eigen::VectorXd> & d,
                                const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                const Eigen::Ref<const Eigen::VectorXd> & x_max) override;

public:
  /** \brief Maximum limits of inequality bounds.

      \note Setting very large values for inequality bounds will result in NaN or incorrect solutions.
     std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max(), and even 1e20 have this problem.
  */
  double bound_limit_ = 1e10;

protected:
  std::unique_ptr<struct d_dense_qp_dim> qp_dim_;
  std::unique_ptr<struct d_dense_qp> qp_;
  std::unique_ptr<struct d_dense_qp_sol> qp_sol_;
  std::unique_ptr<struct d_dense_qp_ipm_arg> ipm_arg_;
  std::unique_ptr<struct d_dense_qp_ipm_ws> ipm_ws_;

  void * qp_dim_mem_ = nullptr;
  void * qp_mem_ = nullptr;
  void * qp_sol_mem_ = nullptr;
  void * ipm_arg_mem_ = nullptr;
  void * ipm_ws_mem_ = nullptr;

  double * opt_x_mem_;
};
#endif

#if ENABLE_PROXQP
/** \brief QP solver PROXQP. */
class QpSolverProxqp : public QpSolver
{
public:
  /** \brief Constructor. */
  QpSolverProxqp();

  /** \brief Solve QP. */
  virtual Eigen::VectorXd solve(int dim_var,
                                int dim_eq,
                                int dim_ineq,
                                Eigen::Ref<Eigen::MatrixXd> Q,
                                const Eigen::Ref<const Eigen::VectorXd> & c,
                                const Eigen::Ref<const Eigen::MatrixXd> & A,
                                const Eigen::Ref<const Eigen::VectorXd> & b,
                                const Eigen::Ref<const Eigen::MatrixXd> & C,
                                const Eigen::Ref<const Eigen::VectorXd> & d,
                                const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                const Eigen::Ref<const Eigen::VectorXd> & x_max) override;

protected:
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> proxqp_;
};
#endif

#if ENABLE_QPMAD
/** \brief QP solver QPMAD. */
class QpSolverQpmad : public QpSolver
{
public:
  /** \brief Constructor. */
  QpSolverQpmad();

  /** \brief Solve QP. */
  virtual Eigen::VectorXd solve(int dim_var,
                                int dim_eq,
                                int dim_ineq,
                                Eigen::Ref<Eigen::MatrixXd> Q,
                                const Eigen::Ref<const Eigen::VectorXd> & c,
                                const Eigen::Ref<const Eigen::MatrixXd> & A,
                                const Eigen::Ref<const Eigen::VectorXd> & b,
                                const Eigen::Ref<const Eigen::MatrixXd> & C,
                                const Eigen::Ref<const Eigen::VectorXd> & d,
                                const Eigen::Ref<const Eigen::VectorXd> & x_min,
                                const Eigen::Ref<const Eigen::VectorXd> & x_max) override;

protected:
  std::unique_ptr<qpmad::Solver> qpmad_;
};
#endif

/** \brief Get one QP solver type that is enabled.

    Checks whether each QP solver is enabled in the order of definition in QpSolverType and returns the first one that
   is enabled.
 */
QpSolverType getAnyQpSolverType();

/** \brief Check whether QP solver is enabled.
    \param qp_solver_type QP solver type
 */
bool isQpSolverEnabled(const QpSolverType & qp_solver_type);

/** \brief Allocate the specified QP solver.
    \param qp_solver_type QP solver type
 */
std::shared_ptr<QpSolver> allocateQpSolver(const QpSolverType & qp_solver_type);
} // namespace QpSolverCollection
