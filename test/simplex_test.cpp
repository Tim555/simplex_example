#include "lp/solver.hpp"

#include <gtest/gtest.h>
#include <stdexcept>

TEST(SimplexTest, Solution) {
    const Eigen::MatrixXd inequalities{{1.0, 2.0}, {2.0, 1.0}};
    Eigen::VectorXd inequalities_rhs(2);
    inequalities_rhs << 6.0, 8.0;
    Eigen::VectorXd maximization_function(2);
    maximization_function << 3.0, 2.0;

    NonNegativeLinearProgram lp(inequalities, inequalities_rhs, maximization_function,
                               LinearProgram::Sense::Maximize,
                               LinearProgram::Domain::NonNegative);
    SimplexSolver solver;
    Eigen::VectorXd solution = solver.solve(lp);

    EXPECT_NEAR(solution[0], 10.0 / 3.0, 1e-9);
    EXPECT_NEAR(solution[1], 4.0 / 3.0, 1e-9);
}

TEST(SimplexTest, Solution2) {
    const Eigen::MatrixXd inequalities{{1.0, 0.0}, {0.0, 2.0}, {3.0, 2.0}};
    Eigen::VectorXd inequalities_rhs(3);
    inequalities_rhs << 4.0, 12.0, 18.0;
    Eigen::VectorXd maximization_function(2);
    maximization_function << 3.0, 5.0;

    NonNegativeLinearProgram lp(inequalities, inequalities_rhs, maximization_function,
                               LinearProgram::Sense::Maximize,
                               LinearProgram::Domain::NonNegative);
    SimplexSolver solver;
    Eigen::VectorXd solution = solver.solve(lp);

    EXPECT_NEAR(solution[0], 2.0, 1e-9);
    EXPECT_NEAR(solution[1], 6.0, 1e-9);
}

TEST(SimplexTest, Infeasible) {
    const Eigen::MatrixXd inequalities{{1.0, 1.0}, {-1.0, -1.0}};
    Eigen::VectorXd inequalities_rhs(2);
    inequalities_rhs << 1.0, -3.0;
    Eigen::VectorXd maximization_function(2);
    maximization_function << 1.0, 1.0;

    NonNegativeLinearProgram lp(inequalities, inequalities_rhs, maximization_function,
                               LinearProgram::Sense::Maximize,
                               LinearProgram::Domain::NonNegative);
    SimplexSolver solver;
    EXPECT_THROW(solver.solve(lp), InvalidModelError);
}

TEST(SimplexTest, Unbounded) {
    const Eigen::MatrixXd inequalities{{1.0, -1.0}};
    Eigen::VectorXd inequalities_rhs(1);
    inequalities_rhs << 1.0;
    Eigen::VectorXd maximization_function(2);
    maximization_function << 1.0, 1.0;

    NonNegativeLinearProgram lp(inequalities, inequalities_rhs, maximization_function,
                               LinearProgram::Sense::Maximize,
                               LinearProgram::Domain::NonNegative);
    SimplexSolver solver;
    EXPECT_THROW(solver.solve(lp), UnboundedProblemError);
}

TEST(SimplexTest, Minimize) {
    const Eigen::MatrixXd inequalities{{1.0, 2.0}, {2.0, 1.0}};
    Eigen::VectorXd inequalities_rhs(2);
    inequalities_rhs << 6.0, 8.0;
    Eigen::VectorXd maximization_function(2);
    maximization_function << 3.0, 2.0;

    NonNegativeLinearProgram lp(inequalities, inequalities_rhs, maximization_function,
                               LinearProgram::Sense::Minimize,
                               LinearProgram::Domain::NonNegative);
    SimplexSolver solver;
    Eigen::VectorXd solution = solver.solve(lp);

    EXPECT_NEAR(solution[0], 0.0, 1e-9);
    EXPECT_NEAR(solution[1], 0.0, 1e-9);
}