#include "lp/solver.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <thread>

namespace {
constexpr double kTolerance = 1e-9;
constexpr int kMaxIterations = 10000;
// Below this many rows, thread spawn overhead outweighs the parallelism benefit.
constexpr Eigen::Index kParallelRowThreshold = 64;
} // namespace

Eigen::VectorXd SimplexSolver::solve(const LinearProgram &linear_program) const {
    if (linear_program.inequalities_rhs().minCoeff() < -kTolerance) {
        throw std::runtime_error(
            "Negative right-hand side requires a phase-1 method, which is not supported.");
    }

    auto standard_form = initialize(linear_program);

    auto num_variables = linear_program.maximization_function().size();
    auto num_inequalities = linear_program.inequalities().rows();
    // Slack variables start basic, one per constraint row.
    std::vector<Eigen::Index> basis(num_inequalities);
    for (Eigen::Index i = 0; i < num_inequalities; ++i) {
        basis[i] = num_variables + i;
    }

    StepResult result = StepResult::Continue;
    int iterations = 0;
    while (result == StepResult::Continue) {
        if (++iterations > kMaxIterations) {
            throw std::runtime_error("Simplex exceeded maximum iterations; possible cycling.");
        }
        result = step(standard_form, basis);
    }
    if (result == StepResult::Unbounded) {
        return unbounded_solution(linear_program);
    }

    Eigen::VectorXd ret = Eigen::VectorXd::Zero(num_variables);
    for (Eigen::Index row = 0; row < num_inequalities; ++row) {
        if (basis[row] < num_variables) {
            ret[basis[row]] = standard_form(row, standard_form.cols() - 1);
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

Eigen::MatrixXd SimplexSolver::initialize(const LinearProgram &linear_program) const {
    auto num_variables = linear_program.inequalities().cols();
    auto num_inequalities = linear_program.inequalities().rows();
    auto num_slack_variables = num_inequalities;
    auto total_variables = num_variables + num_slack_variables + 1;

    Eigen::MatrixXd standard_form = Eigen::MatrixXd::Zero(num_inequalities + 1, total_variables);
    standard_form.block(0, 0, num_inequalities, num_variables) = linear_program.inequalities();
    standard_form.block(0, num_variables, num_inequalities, num_slack_variables) =
        Eigen::MatrixXd::Identity(num_inequalities, num_slack_variables);
    standard_form.block(0, total_variables - 1, num_inequalities, 1) =
        linear_program.inequalities_rhs();
    standard_form.block(num_inequalities, 0, 1, num_variables) =
        -linear_program.maximization_function().transpose();

    return standard_form;
}

Eigen::Index SimplexSolver::leaving_variable(const Eigen::MatrixXd &standard_form,
                                             Eigen::Index entering) const {
    if (entering < 0) {
        return -1;
    }
    Eigen::Index leaving = -1;
    double best_ratio = std::numeric_limits<double>::infinity();
    for (Eigen::Index row = 0; row < standard_form.rows() - 1; ++row) {
        double coeff = standard_form(row, entering);
        if (coeff <= kTolerance) {
            continue; // only rows with a positive entering-column coefficient are valid
        }
        double ratio = standard_form(row, standard_form.cols() - 1) / coeff;
        if (ratio < best_ratio) {
            best_ratio = ratio;
            leaving = row;
        }
    }
    return leaving;
}

Eigen::Index SimplexSolver::entering_variable(const Eigen::MatrixXd &standard_form) const {
    auto bottom_row = standard_form.bottomRows(1);
    Eigen::Index row_idx;
    Eigen::Index col_idx;
    double min_val = bottom_row.minCoeff(&row_idx, &col_idx); // Single scan pass
    if (min_val < -kTolerance) {
        return col_idx;
    }
    return -1;
}

SimplexSolver::StepResult SimplexSolver::step(Eigen::MatrixXd &standard_form,
                                              std::vector<Eigen::Index> &basis) const {
    auto entering = entering_variable(standard_form);
    if (entering < 0) {
        return StepResult::Optimal;
    }
    if (check_unbounded(standard_form, entering)) {
        return StepResult::Unbounded;
    }
    auto leaving = leaving_variable(standard_form, entering);

    // first divide the pivot row by the pivot element to make it 1
    double pivot_value = standard_form(leaving, entering);
    if (std::abs(pivot_value) < kTolerance) {
        throw std::runtime_error("Pivot element is zero.");
    }
    standard_form.row(leaving) /= pivot_value;

    // now eliminate the entering column in all other rows; each row update is independent.
    auto eliminate = [&](Eigen::Index begin, Eigen::Index end) {
        for (Eigen::Index i = begin; i < end; ++i) {
            if (i != leaving) {
                double factor = standard_form(i, entering);
                standard_form.row(i) -= factor * standard_form.row(leaving);
            }
        }
    };

    Eigen::Index total_rows = standard_form.rows();
    if (total_rows < kParallelRowThreshold) {
        eliminate(0, total_rows);
    } else {
        unsigned int thread_count = std::max(1u, std::thread::hardware_concurrency());
        Eigen::Index chunk_size = (total_rows + thread_count - 1) / thread_count;
        std::vector<std::jthread> workers;
        for (unsigned int t = 0; t < thread_count; ++t) {
            Eigen::Index begin = t * chunk_size;
            Eigen::Index end = std::min(total_rows, begin + chunk_size);
            if (begin >= end) {
                break;
            }
            workers.emplace_back(eliminate, begin, end);
        }
        // std::jthread joins automatically when workers goes out of scope.
    }

    basis[leaving] = entering;
    return StepResult::Continue;
}

bool SimplexSolver::check_unbounded(const Eigen::MatrixXd &standard_form,
                                    Eigen::Index entering) const {
    for (Eigen::Index i = 0; i < standard_form.rows() - 1; ++i) {
        if (standard_form(i, entering) > 0) {
            return false;
        }
    }
    return true;
}

bool SimplexSolver::check_feasible(const Eigen::MatrixXd &standard_form) const {
    for (Eigen::Index i = 0; i < standard_form.rows() - 1; ++i) {
        if (standard_form(i, standard_form.cols() - 1) < 0) {
            return false;
        }
    }
    return true;
}
