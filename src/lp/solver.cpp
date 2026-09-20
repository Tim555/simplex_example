#include "lp/solver.hpp"
#include <iostream>

SimplexSolver::SimplexSolver(const LinearProgram &linear_program)
    : linear_program_(linear_program) {}

const Eigen::MatrixXd &SimplexSolver::inequalities() const noexcept {
    return linear_program_.inequalities();
}

const Eigen::VectorXd &SimplexSolver::inequalities_rhs() const noexcept {
    return linear_program_.inequalities_rhs();
}

const Eigen::VectorXd &SimplexSolver::maximization_function() const noexcept {
    return linear_program_.maximization_function();
}

Eigen::VectorXd SimplexSolver::solve() const { 
    auto standard_form = initialize();
    if (!standard_form) {
        throw std::runtime_error("Failed to initialize standard form.");
    }

    while (!step(standard_form)) {}

    std::cout << *standard_form << std::endl;
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(linear_program_.maximization_function().size());
    for (Eigen::Index col = 0; col < linear_program_.maximization_function().size(); ++col) {
        for (Eigen::Index row = 0; row < linear_program_.maximization_function().size(); ++row) {
            // You can access the solution for the original variables here if needed
            auto value = (*standard_form)(row, col);
            if (value == 1){
                ret[col] = (*standard_form)(row, standard_form->cols() - 1);
            }
        }
    }
    return ret;
}

std::shared_ptr<Eigen::MatrixXd> SimplexSolver::initialize() const {
    auto num_variables = inequalities().cols();
    auto num_inequalities = inequalities().rows();
    auto num_slack_variables = num_inequalities;
    auto total_variables = num_variables + num_slack_variables + 1;

    auto standard_form = std::make_shared<Eigen::MatrixXd>(
        Eigen::MatrixXd::Zero(num_inequalities + 1, total_variables));
    standard_form->block(0, 0, num_inequalities, num_variables) = inequalities();
    standard_form->block(0, num_variables, num_inequalities, num_slack_variables) =
        Eigen::MatrixXd::Identity(num_inequalities, num_slack_variables);
    standard_form->block(0, total_variables - 1, num_inequalities, 1) = inequalities_rhs();
    standard_form->block(num_inequalities, 0, 1, num_variables) =
        -maximization_function().transpose();

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

bool SimplexSolver::step(std::shared_ptr<Eigen::MatrixXd> standard_form) const {
    auto [leaving, entering] = compute_pivot(standard_form);
    if (leaving < 0 || entering < 0) {
        return true; // indicates that the algorithm should terminate
    }

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
    
    return false; // indicates that the algorithm should continue
}