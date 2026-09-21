#pragma once

#include "lp/linear_program.hpp"
#include <Eigen/Dense>
#include <memory>

/// Abstract solver that produces a solution vector for a LinearProgram.
class Solver {
  public:
    virtual ~Solver() = default;

    /// Solves the given program and returns the resulting decision variable values.
    virtual Eigen::VectorXd solve(const LinearProgram &linear_program) const = 0;
};

/// Solves a LinearProgram using the tableau-based simplex method.
class SimplexSolver : public Solver {
  public:
    /// Outcome of a single simplex pivot step.
    enum class StepResult { Continue, Optimal, Unbounded };

    /// Runs the simplex algorithm to completion; throws std::runtime_error if no feasible solution
    /// is found.
    Eigen::VectorXd solve(const LinearProgram &linear_program) const override;

  private:
    /// Builds the initial simplex tableau (standard form) from the given program.
    std::shared_ptr<Eigen::MatrixXd> initialize(const LinearProgram &linear_program) const;
    /// Determines the (row, column) pivot position for the next simplex step.
    std::tuple<Eigen::Index, Eigen::Index>
    compute_pivot(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    /// Selects the entering variable column using the objective row.
    Eigen::Index entering_variable(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    /// Selects the leaving variable row via the minimum ratio test for the given entering column.
    Eigen::Index leaving_variable(std::shared_ptr<Eigen::MatrixXd> standard_form,
                                  Eigen::Index entering) const;

    /// Returns true if the entering column indicates an unbounded objective.
    bool check_unbounded(std::shared_ptr<Eigen::MatrixXd> standard_form,
                         Eigen::Index entering) const;
    /// Returns true if the current tableau represents a feasible solution.
    bool check_feasible(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    /// Returns true if the given column is a unit (basic) column.
    bool check_unit_variable(std::shared_ptr<Eigen::MatrixXd> standard_form,
                             const LinearProgram &linear_program) const;
    /// Performs one pivot step and reports whether to continue, stop optimally, or stop as
    /// unbounded.
    StepResult step(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    /// Builds the sentinel solution vector returned when the program is unbounded.
    Eigen::VectorXd unbounded_solution(const LinearProgram &linear_program) const;
};
