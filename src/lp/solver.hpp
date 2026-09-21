#pragma once

#include "lp/linear_program.hpp"
#include <Eigen/Dense>
#include <memory>

/// Abstract solver that produces a solution vector for a LinearProgram.
class Solver {
  public:
    virtual ~Solver() = default;

    /// Solves the program and returns the resulting decision variable values.
    virtual Eigen::VectorXd solve() const = 0;
};

/// Solves a LinearProgram using the tableau-based simplex method.
class SimplexSolver : public Solver {
  public:
    /// Outcome of a single simplex pivot step.
    enum class StepResult {
        Continue,
        Optimal,
        Unbounded
    };

    /// Binds the solver to a program; linear_program must outlive this solver.
    explicit SimplexSolver(const LinearProgram &linear_program);

    /// Coefficient matrix A of the bound program's inequality constraints.
    const Eigen::MatrixXd &inequalities() const noexcept;
    /// Right-hand side vector b of the bound program's inequality constraints.
    const Eigen::VectorXd &inequalities_rhs() const noexcept;
    /// Objective coefficients of the bound program, oriented for maximization.
    const Eigen::VectorXd &maximization_function() const noexcept;

    /// Runs the simplex algorithm to completion; throws std::runtime_error if no feasible solution is found.
    Eigen::VectorXd solve() const override;

  private:
    const LinearProgram &linear_program_;

    /// Builds the initial simplex tableau (standard form) from the bound program.
    std::shared_ptr<Eigen::MatrixXd> initialize() const;
    /// Determines the (row, column) pivot position for the next simplex step.
    std::tuple<Eigen::Index, Eigen::Index> compute_pivot(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    /// Selects the entering variable column using the objective row.
    Eigen::Index entering_variable(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    /// Selects the leaving variable row via the minimum ratio test for the given entering column.
    Eigen::Index leaving_variable(std::shared_ptr<Eigen::MatrixXd> standard_form, Eigen::Index entering) const;

    /// Returns true if the entering column indicates an unbounded objective.
    bool check_unbounded(std::shared_ptr<Eigen::MatrixXd> standard_form, Eigen::Index entering) const;
    /// Returns true if the current tableau represents a feasible solution.
    bool check_feasible(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    /// Returns true if the given column is a unit (basic) column.
    bool check_unit_variable(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    /// Performs one pivot step and reports whether to continue, stop optimally, or stop as unbounded.
    StepResult step(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    /// Builds the sentinel solution vector returned when the program is unbounded.
    Eigen::VectorXd unbounded_solution() const;
};
