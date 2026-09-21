#include "lp/solver.hpp"
#include <cmath>
#include <iostream>
#include <limits>

Eigen::VectorXd SimplexSolver::solve(const LinearProgram &linear_program) const {
    auto standard_form = initialize(linear_program);
    if (!standard_form) {
        throw std::runtime_error("Failed to initialize standard form.");
    }

    StepResult result = StepResult::Continue;
    while (result == StepResult::Continue) {
        result = step(standard_form);
    }
    if (result == StepResult::Unbounded) {
        return unbounded_solution(linear_program);
    }

    Eigen::VectorXd ret = Eigen::VectorXd::Zero(linear_program.maximization_function().size());
    if (!check_unit_variable(standard_form, linear_program)) {
        return ret;
    }
    for (Eigen::Index col = 0; col < linear_program.maximization_function().size(); ++col) {
        for (Eigen::Index row = 0; row < linear_program.inequalities().rows(); ++row) {
            // You can access the solution for the original variables here if needed
            auto value = (*standard_form)(row, col);
            if (value == 1){
                ret[col] = (*standard_form)(row, standard_form->cols() - 1);
            }
        }
    }
    if (check_feasible(standard_form) == false) {
        throw std::runtime_error("Solution is not feasible.");
    }
    return ret;
}

Eigen::VectorXd SimplexSolver::unbounded_solution(const LinearProgram &linear_program) const {
    double fill = linear_program.sense() == LinearProgram::Sense::Maximize
        ? std::numeric_limits<double>::infinity()
        : -std::numeric_limits<double>::infinity();
    return Eigen::VectorXd::Constant(linear_program.maximization_function().size(), fill);
}

std::shared_ptr<Eigen::MatrixXd> SimplexSolver::initialize(const LinearProgram &linear_program) const {
    auto num_variables = linear_program.inequalities().cols();
    auto num_inequalities = linear_program.inequalities().rows();
    auto num_slack_variables = num_inequalities;
    auto total_variables = num_variables + num_slack_variables + 1;

    auto standard_form = std::make_shared<Eigen::MatrixXd>(
        Eigen::MatrixXd::Zero(num_inequalities + 1, total_variables));
    standard_form->block(0, 0, num_inequalities, num_variables) = linear_program.inequalities();
    standard_form->block(0, num_variables, num_inequalities, num_slack_variables) =
        Eigen::MatrixXd::Identity(num_inequalities, num_slack_variables);
    standard_form->block(0, total_variables - 1, num_inequalities, 1) = linear_program.inequalities_rhs();
    standard_form->block(num_inequalities, 0, 1, num_variables) =
        -linear_program.maximization_function().transpose();

    return standard_form;
}

Eigen::Index SimplexSolver::leaving_variable(std::shared_ptr<Eigen::MatrixXd> standard_form, Eigen::Index entering) const {
    if (entering < 0) {
        return -1;
    }
    Eigen::VectorXd result = standard_form->col(standard_form->cols() - 1).array() / standard_form->col(entering).array();
    result.conservativeResize(result.size() - 1); // Remove the last element (bottom row)
    Eigen::Index leaving = -1;
    result.minCoeff(&leaving);
    return leaving;
}

Eigen::Index SimplexSolver::entering_variable(std::shared_ptr<Eigen::MatrixXd> standard_form) const {
    auto bottomRow = standard_form->bottomRows(1);
    Eigen::Index minColIdx;
    Eigen::Index minRowIdx;
    double minVal = bottomRow.minCoeff(&minColIdx, &minRowIdx); // Single scan pass
    if (minVal < 0) {
        return minRowIdx;
    }
    return -1;
}

std::tuple<Eigen::Index, Eigen::Index> SimplexSolver::compute_pivot(std::shared_ptr<Eigen::MatrixXd> standard_form) const {
    auto entering = entering_variable(standard_form);
    auto leaving = leaving_variable(standard_form, entering);
    return std::make_tuple(leaving, entering);
}

SimplexSolver::StepResult SimplexSolver::step(std::shared_ptr<Eigen::MatrixXd> standard_form) const {
    auto entering = entering_variable(standard_form);
    if (entering < 0) {
        return StepResult::Optimal;
    }
    if (check_unbounded(standard_form, entering)) {
        return StepResult::Unbounded;
    }
    auto leaving = leaving_variable(standard_form, entering);

    // first divide the pivot row by the pivot element to make it 1
    double pivot_value = (*standard_form)(leaving, entering);
    if (pivot_value == 0) {
        throw std::runtime_error("Pivot element is zero.");
    }
    standard_form->row(leaving) /= pivot_value;

    // now eliminate the entering column in all other rows
    for (Eigen::Index i = 0; i < standard_form->rows(); ++i) {
        if (i != leaving) {
            double factor = (*standard_form)(i, entering);
            standard_form->row(i) -= factor * standard_form->row(leaving);
        }
    }
    
    return StepResult::Continue;
}

bool SimplexSolver::check_unbounded(std::shared_ptr<Eigen::MatrixXd> standard_form, Eigen::Index entering) const {
    for (Eigen::Index i = 0; i < standard_form->rows() - 1; ++i) {
        if ((*standard_form)(i, entering) > 0) {
            return false;
        }
    }
    return true;
}

bool SimplexSolver::check_feasible(std::shared_ptr<Eigen::MatrixXd> standard_form) const {
    for (Eigen::Index i = 0; i < standard_form->rows() - 1; ++i) {
        if ((*standard_form)(i, standard_form->cols() - 1) < 0) {
            return false;
        }
    }
    return true;
}

bool SimplexSolver::check_unit_variable(std::shared_ptr<Eigen::MatrixXd> standard_form, const LinearProgram &linear_program) const {
    constexpr double kTolerance = 1e-9;
    auto solution_block = standard_form->topLeftCorner(linear_program.inequalities_rhs().size(), linear_program.maximization_function().size());

    for (Eigen::Index col = 0; col < solution_block.cols(); ++col) {
        Eigen::Index ones_in_col = 0;
        for (Eigen::Index row = 0; row < solution_block.rows(); ++row) {
            if (std::abs(solution_block(row, col) - 1.0) < kTolerance) {
                ++ones_in_col;
            } else if (std::abs(solution_block(row, col)) >= kTolerance) {
                return false;
            }
        }
        if (ones_in_col != 1) {
            return false;
        }
    }

    return true;
}
    