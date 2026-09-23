#pragma once

#include <Eigen/Dense>

/// Abstract representation of a linear program of the form Ax <= b, optimizing c^T x.
class LinearProgram {
  public:
    /// Optimization direction of the objective function.
    enum class Sense {
        Minimize,
        Maximize
    };

    /// Sign restriction applied to the decision variables.
    enum class Domain {
        NonNegative,
        Unrestricted
    };
    virtual ~LinearProgram() = default;

    /// Coefficient matrix A of the inequality constraints Ax <= b.
    virtual const Eigen::MatrixXd &inequalities() const noexcept = 0;
    /// Right-hand side vector b of the inequality constraints Ax <= b.
    virtual const Eigen::VectorXd &inequalities_rhs() const noexcept = 0;
    /// Objective coefficients, already oriented for maximization regardless of sense().
    virtual const Eigen::VectorXd &maximization_function() const noexcept = 0;

    /// Whether the original objective is to minimize or maximize.
    virtual LinearProgram::Sense sense() const noexcept = 0;
    /// Whether the decision variables are constrained to x >= 0 or remain unrestricted.
    virtual LinearProgram::Domain domain() const noexcept = 0;
};

/// A LinearProgram backed by explicit, in-memory constraint and objective data.
class NonNegativeLinearProgram : public LinearProgram {
  public:
    /// Constructs a program from raw constraint/objective data; throws std::invalid_argument on dimension mismatch.
    NonNegativeLinearProgram(const Eigen::MatrixXd &inequalities,
                             const Eigen::VectorXd &inequalities_rhs,
                             const Eigen::VectorXd &maximization_function,
                             LinearProgram::Sense sense,
                             LinearProgram::Domain domain);

    const Eigen::MatrixXd &inequalities() const noexcept override;
    const Eigen::VectorXd &inequalities_rhs() const noexcept override;
    const Eigen::VectorXd &maximization_function() const noexcept override;
    LinearProgram::Sense sense() const noexcept override;
    LinearProgram::Domain domain() const noexcept override;

  private:
    Eigen::MatrixXd inequalities_;
    Eigen::VectorXd inequalities_rhs_;
    Eigen::VectorXd maximization_function_;
    // Cached maximization_function_ negated when sense_ is Minimize, since maximization_function() returns a reference.
    Eigen::VectorXd effective_maximization_function_;
    LinearProgram::Sense sense_;
    LinearProgram::Domain domain_;
};