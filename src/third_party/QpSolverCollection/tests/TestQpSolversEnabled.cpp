/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <qp_solver_collection/QpSolverCollection.h>

using QpSolverCollection::QpSolverType;

void checkOneSolver(QpSolverType qp_solver_type)
{
  EXPECT_TRUE(QpSolverCollection::isQpSolverEnabled(qp_solver_type));
  EXPECT_TRUE(QpSolverCollection::allocateQpSolver(qp_solver_type));
}

TEST(TestQpSolversEnabled, Any)
{
  checkOneSolver(QpSolverType::Any);
}
#if ENABLE_QLD || defined FORCE_ALL_SOLVER_TEST
TEST(TestQpSolversEnabled, QLD)
{
  checkOneSolver(QpSolverType::QLD);
}
#endif

#if ENABLE_QUADPROG || defined FORCE_ALL_SOLVER_TEST
TEST(TestQpSolversEnabled, QuadProg)
{
  checkOneSolver(QpSolverType::QuadProg);
}
#endif

#if ENABLE_LSSOL || (defined FORCE_ALL_SOLVER_TEST && !defined SKIP_PRIVATE_SOLVER_TEST)
TEST(TestQpSolversEnabled, LSSOL)
{
  checkOneSolver(QpSolverType::LSSOL);
}
#endif

#if ENABLE_JRLQP || defined FORCE_ALL_SOLVER_TEST
TEST(TestQpSolversEnabled, JRLQP)
{
  checkOneSolver(QpSolverType::JRLQP);
}
#endif

#if ENABLE_QPOASES || defined FORCE_ALL_SOLVER_TEST
TEST(TestQpSolversEnabled, qpOASES)
{
  checkOneSolver(QpSolverType::qpOASES);
}
#endif

#if ENABLE_OSQP || defined FORCE_ALL_SOLVER_TEST
TEST(TestQpSolversEnabled, OSQP)
{
  checkOneSolver(QpSolverType::OSQP);
}
#endif

#if ENABLE_NASOQ || defined FORCE_ALL_SOLVER_TEST
TEST(TestQpSolversEnabled, NASOQ)
{
  checkOneSolver(QpSolverType::NASOQ);
}
#endif

#if ENABLE_HPIPM || defined FORCE_ALL_SOLVER_TEST
TEST(TestQpSolversEnabled, HPIPM)
{
  checkOneSolver(QpSolverType::HPIPM);
}
#endif

#if ENABLE_PROXQP || defined FORCE_ALL_SOLVER_TEST
TEST(TestQpSolversEnabled, PROXQP)
{
  checkOneSolver(QpSolverType::PROXQP);
}
#endif

#if ENABLE_QPMAD || defined FORCE_ALL_SOLVER_TEST
TEST(TestQpSolversEnabled, QPMAD)
{
  checkOneSolver(QpSolverType::QPMAD);
}
#endif

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
