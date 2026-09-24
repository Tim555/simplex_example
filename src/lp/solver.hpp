#pragma once

#include "lp/linear_program.hpp"
#include <Eigen/Dense>
#include <stdexcept>
#include <vector>

/// Base class for solver failures.
class SolverError : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};

/// The input LP violates the assumptions required by this solver.
class InvalidModelError : public SolverError {
  public:
    using SolverError::SolverError;
};

/// The LP has no feasible solution.
class InfeasibleProblemError : public SolverError {
  public:
    using SolverError::SolverError;
};

/// The LP objective can be made arbitrarily good.
class UnboundedProblemError : public SolverError {
  public:
    using SolverError::SolverError;
};

/// The simplex algorithm encountered a numerical or cycling failure.
class NumericalFailureError : public SolverError {
  public:
    using SolverError::SolverError;
};

/// Abstract solver that produces a solution vector for a LinearProgram.
class Solver {
  public:
    virtual ~Solver() = default;

    /// Solves the given program and returns the resulting decision variable values.
    virtual Eigen::VectorXd solve(const LinearProgram &linear_program) const = 0;
};

/// Solves a LinearProgram using the tableau-based simplex method.
///
/// Deliberate limitations for this learning implementation:
/// - The solver currently assumes all RHS values are non-negative.
/// - The solver assumes standard form with slack variables.
/// - The solver does not implement phase-1 feasibility recovery.
/// These assumptions are intentional for this project, but they should be treated as explicit
/// limitations of the current solver.
class SimplexSolver : public Solver {
  public:
    /// Outcome of a single simplex pivot step.
    enum class StepResult { Continue, Optimal, Unbounded };

    /// Runs the simplex algorithm to completion; throws std::runtime_error if no feasible solution
    /// is found.
    Eigen::VectorXd solve(const LinearProgram &linear_program) const override;

  private:
    /// Builds the initial simplex tableau (standard form) from the given program.
    Eigen::MatrixXd initialize(const LinearProgram &linear_program) const;
    /// Selects the entering variable column using the objective row.
    Eigen::Index entering_variable(const Eigen::MatrixXd &standard_form) const;
    /// Selects the leaving variable row via the minimum ratio test for the given entering column.
    Eigen::Index leaving_variable(const Eigen::MatrixXd &standard_form,
                                  Eigen::Index entering) const;

    /// Returns true if the entering column indicates an unbounded objective.
    bool check_unbounded(const Eigen::MatrixXd &standard_form, Eigen::Index entering) const;
    /// Returns true if the current tableau represents a feasible solution.
    bool check_feasible(const Eigen::MatrixXd &standard_form) const;
    /// Performs one pivot step, recording the variable now basic in the pivot row, and reports
    /// whether to continue, stop optimally, or stop as unbounded.
    StepResult step(Eigen::MatrixXd &standard_form, std::vector<Eigen::Index> &basis) const;
    /// Builds the sentinel solution vector returned when the program is unbounded.
    Eigen::VectorXd unbounded_solution(const LinearProgram &linear_program) const;
};
