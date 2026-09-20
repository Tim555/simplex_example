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
    std::cout << *standard_form << std::endl;

    auto [leaving, entering] = compute_pivot(standard_form);

    return maximization_function(); 
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
    Eigen::VectorXd result = standard_form->col(standard_form->cols() - 1).array() / standard_form->col(entering).array();
    result.conservativeResize(result.size() - 1); // Remove the last element (bottom row)
    std::cout << "Leaving variable ratios: " << result.transpose() << std::endl;
    Eigen::Index leaving = -1;
    return result.minCoeff(&leaving);
}

Eigen::Index SimplexSolver::entering_variable(std::shared_ptr<Eigen::MatrixXd> standard_form) const {
    auto bottomRow = standard_form->bottomRows(1);
    Eigen::Index minColIdx;
    Eigen::Index minRowIdx;
    double minVal = bottomRow.minCoeff(&minColIdx, &minRowIdx); // Single scan pass
    if (minVal < 0) {
        // You instantly have your index and validation in one go
        std::cout << "Negative value found at column: " << minColIdx << std::endl;
        return minColIdx;
    }
    return -1;
}

std::tuple<Eigen::Index, Eigen::Index> SimplexSolver::compute_pivot(std::shared_ptr<Eigen::MatrixXd> standard_form) const {
    auto entering = entering_variable(standard_form);
    auto leaving = leaving_variable(standard_form, entering);
    std::cout << "Pivot: leaving = " << leaving << ", entering = " << entering << std::endl;
    
    return std::make_tuple(leaving, entering);
}