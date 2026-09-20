#include "lp/solver.hpp"

#include <gtest/gtest.h>
#include <stdexcept>

TEST(SimplexTest, RejectsInvalidLinearProgramConstruction) {
    const Eigen::MatrixXd inequalities{{1.0, 2.0}, {3.0, 4.0}};
    Eigen::VectorXd inequalities_rhs(1);
    inequalities_rhs << 7.0;
    Eigen::VectorXd maximization_function(2);
    maximization_function << 5.0, 6.0;

    EXPECT_THROW(NaturalLinearProgram(
        inequalities, inequalities_rhs, maximization_function,
        NaturalLinearProgram::Sense::Maximize,
        NaturalLinearProgram::Domain::Natural),
                 std::invalid_argument);
}
