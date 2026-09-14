#include <OsqpEigen/OsqpEigen.h>
#include <gtest/gtest.h>

// Exercise storage allocated by the shared OsqpEigen library and destroyed by
// the caller's implicit Solver destructor, independently of T-DT and costmaps.
TEST(SolverBoundary, SolutionStorageSurvivesLibraryAndCallerLifetime)
{
  for (const int n : {17, 18, 33}) {
    Eigen::SparseMatrix<c_float> hessian(n, n), constraints(n, n);
    Eigen::Matrix<c_float, -1, 1> target(n);
    for (int i = 0; i < n; ++i) {
      hessian.insert(i, i) = 2.0;
      constraints.insert(i, i) = 1.0;
      target[i] = 0.1 * (i + 1);
    }
    Eigen::Matrix<c_float, -1, 1> gradient = -2.0 * target;
    Eigen::Matrix<c_float, -1, 1> lower = target.array() - 1.0;
    Eigen::Matrix<c_float, -1, 1> upper = target.array() + 1.0;
    Eigen::Matrix<c_float, -1, 1> copied;
    {
      OsqpEigen::Solver solver;
      solver.settings()->setVerbosity(false);
      solver.settings()->setAbsoluteTolerance(1e-7);
      solver.settings()->setRelativeTolerance(1e-7);
      solver.data()->setNumberOfVariables(n);
      solver.data()->setNumberOfConstraints(n);
      ASSERT_TRUE(solver.data()->setHessianMatrix(hessian));
      ASSERT_TRUE(solver.data()->setGradient(gradient));
      ASSERT_TRUE(solver.data()->setLinearConstraintsMatrix(constraints));
      ASSERT_TRUE(solver.data()->setLowerBound(lower));
      ASSERT_TRUE(solver.data()->setUpperBound(upper));
      ASSERT_TRUE(solver.initSolver());
      ASSERT_EQ(solver.solveProblem(), OsqpEigen::ErrorExitFlag::NoError);
      ASSERT_EQ(solver.getStatus(), OsqpEigen::Status::Solved);
      const auto & solution = solver.getSolution();
      ASSERT_EQ(solution.size(), n);
      ASSERT_TRUE(solution.allFinite());
      EXPECT_LT((solution - target).lpNorm<Eigen::Infinity>(), 1e-5);
      const auto & dual = solver.getDualSolution();
      ASSERT_EQ(dual.size(), n);
      ASSERT_TRUE(dual.allFinite());
      copied = solution;
    }  // Both library-owned dynamic vectors are freed here under ASan.
    ASSERT_TRUE(copied.allFinite());
    EXPECT_LT((copied - target).lpNorm<Eigen::Infinity>(), 1e-5);
  }
}
