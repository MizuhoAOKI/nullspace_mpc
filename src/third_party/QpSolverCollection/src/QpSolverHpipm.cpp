/* Author: Masaki Murooka */

#include <qp_solver_collection/QpSolverOptions.h>

#if ENABLE_HPIPM
#  include <numeric>

#  include <qp_solver_collection/QpSolverCollection.h>

#  include <hpipm_d_dense_qp_ipm.h>

using namespace QpSolverCollection;

QpSolverHpipm::QpSolverHpipm()
{
  type_ = QpSolverType::HPIPM;
  qp_dim_ = std::make_unique<struct d_dense_qp_dim>();
  qp_ = std::make_unique<struct d_dense_qp>();
  qp_sol_ = std::make_unique<struct d_dense_qp_sol>();
  ipm_arg_ = std::make_unique<struct d_dense_qp_ipm_arg>();
  ipm_ws_ = std::make_unique<struct d_dense_qp_ipm_ws>();
}

QpSolverHpipm::~QpSolverHpipm()
{
  if(qp_dim_mem_ != nullptr)
  {
    free(qp_dim_mem_);
  }
  if(qp_mem_ != nullptr)
  {
    free(qp_mem_);
  }
  if(qp_sol_mem_ != nullptr)
  {
    free(qp_sol_mem_);
  }
  if(ipm_arg_mem_ != nullptr)
  {
    free(ipm_arg_mem_);
  }
  if(ipm_ws_mem_ != nullptr)
  {
    free(ipm_ws_mem_);
  }
  if(opt_x_mem_ != nullptr)
  {
    free(opt_x_mem_);
  }
}

Eigen::VectorXd QpSolverHpipm::solve(int dim_var,
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
  // Allocate memory
  if(!(qp_dim_->nv == dim_var && qp_dim_->ne == dim_eq && qp_dim_->ng == dim_ineq))
  {
    int qp_dim_size = d_dense_qp_dim_memsize();
    qp_dim_mem_ = malloc(qp_dim_size);
    d_dense_qp_dim_create(qp_dim_.get(), qp_dim_mem_);
    d_dense_qp_dim_set_all(dim_var, dim_eq, dim_var, dim_ineq, 0, 0, qp_dim_.get());

    int qp_size = d_dense_qp_memsize(qp_dim_.get());
    qp_mem_ = malloc(qp_size);
    d_dense_qp_create(qp_dim_.get(), qp_.get(), qp_mem_);

    int qp_sol_size = d_dense_qp_sol_memsize(qp_dim_.get());
    qp_sol_mem_ = malloc(qp_sol_size);
    d_dense_qp_sol_create(qp_dim_.get(), qp_sol_.get(), qp_sol_mem_);

    int ipm_arg_size = d_dense_qp_ipm_arg_memsize(qp_dim_.get());
    ipm_arg_mem_ = malloc(ipm_arg_size);
    d_dense_qp_ipm_arg_create(qp_dim_.get(), ipm_arg_.get(), ipm_arg_mem_);
    enum hpipm_mode mode = SPEED; // SPEED_ABS, SPEED, BALANCE, ROBUST
    d_dense_qp_ipm_arg_set_default(mode, ipm_arg_.get());

    int ipm_ws_size = d_dense_qp_ipm_ws_memsize(qp_dim_.get(), ipm_arg_.get());
    ipm_ws_mem_ = malloc(ipm_ws_size);
    d_dense_qp_ipm_ws_create(qp_dim_.get(), ipm_arg_.get(), ipm_ws_.get(), ipm_ws_mem_);

    opt_x_mem_ = static_cast<double *>(malloc(dim_var * sizeof(double)));
  }

  // Set QP coefficients
  {
    d_dense_qp_set_H(Q.data(), qp_.get());
    d_dense_qp_set_g(const_cast<double *>(c.data()), qp_.get());
    d_dense_qp_set_A(const_cast<double *>(A.data()), qp_.get());
    d_dense_qp_set_b(const_cast<double *>(b.data()), qp_.get());
    d_dense_qp_set_C(const_cast<double *>(C.data()), qp_.get());
    std::vector<double> lg(dim_ineq, -1 * bound_limit_);
    d_dense_qp_set_lg(lg.data(), qp_.get());
    d_dense_qp_set_ug(const_cast<double *>(d.cwiseMin(bound_limit_).eval().data()), qp_.get());
    std::vector<int> idxb(dim_var);
    std::iota(idxb.begin(), idxb.end(), 0);
    d_dense_qp_set_idxb(idxb.data(), qp_.get());
    d_dense_qp_set_lb(const_cast<double *>(x_min.cwiseMax(-1 * bound_limit_).eval().data()), qp_.get());
    d_dense_qp_set_ub(const_cast<double *>(x_max.cwiseMin(bound_limit_).eval().data()), qp_.get());
  }

  // Solve QP
  {
    d_dense_qp_ipm_solve(qp_.get(), qp_sol_.get(), ipm_arg_.get(), ipm_ws_.get());
    d_dense_qp_sol_get_v(qp_sol_.get(), opt_x_mem_);

    int status;
    d_dense_qp_ipm_get_status(ipm_ws_.get(), &status);
    if(status == SUCCESS || status == MAX_ITER) // enum hpipm_status
    {
      solve_failed_ = false;
    }
    else
    {
      solve_failed_ = true;
      QSC_WARN_STREAM("[QpSolverHpipm::solve] Failed to solve: " << status);
    }
  }

  return Eigen::Map<Eigen::VectorXd>(opt_x_mem_, dim_var);
}

namespace QpSolverCollection
{
std::shared_ptr<QpSolver> allocateQpSolverHpipm()
{
  return std::make_shared<QpSolverHpipm>();
}
} // namespace QpSolverCollection
#endif
