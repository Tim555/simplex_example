#include "lp/linear_program.hpp"

#include <gtest/gtest.h>

TEST(LinearProgramTest, AcceptsValidLinearProgram) {
    const Eigen::MatrixXd inequalities{{1.0, 2.0}, {3.0, 4.0}};
    Eigen::VectorXd inequalities_rhs(2);
    inequalities_rhs << 7.0, 11.0;
    Eigen::VectorXd maximization_function(2);
    maximization_function << 5.0, 6.0;

    EXPECT_NO_THROW({
        const NaturalLinearProgram linear_program(
            inequalities, inequalities_rhs, maximization_function,
            NaturalLinearProgram::Sense::Maximize, NaturalLinearProgram::Domain::Natural);
        EXPECT_EQ(linear_program.inequalities(), inequalities);
        EXPECT_EQ(linear_program.inequalities_rhs(), inequalities_rhs);
        EXPECT_EQ(linear_program.maximization_function(), maximization_function);
    });
}

TEST(LinearProgramTest, RejectsInvalidLinearProgramConstruction) {
    const Eigen::MatrixXd inequalities{{1.0, 2.0}, {3.0, 4.0}};
    Eigen::VectorXd inequalities_rhs(1);
    inequalities_rhs << 7.0;
    Eigen::VectorXd maximization_function(2);
    maximization_function << 5.0, 6.0;

    EXPECT_THROW(NaturalLinearProgram(inequalities, inequalities_rhs, maximization_function,
                                      NaturalLinearProgram::Sense::Maximize,
                                      NaturalLinearProgram::Domain::Natural),
                 std::invalid_argument);
}
