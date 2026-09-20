#pragma once

#include "lp/linear_program.hpp"
#include <Eigen/Dense>
#include <memory>

class Solver {
  public:
    virtual ~Solver() = default;

    virtual Eigen::VectorXd solve() const = 0;
};

class SimplexSolver : public Solver {
  public:
    enum class StepResult {
        Continue,
        Optimal,
        Unbounded
    };

    explicit SimplexSolver(const LinearProgram &linear_program);

    const Eigen::MatrixXd &inequalities() const noexcept;
    const Eigen::VectorXd &inequalities_rhs() const noexcept;
    const Eigen::VectorXd &maximization_function() const noexcept;

    Eigen::VectorXd solve() const override;

  private:
    const LinearProgram &linear_program_;

    std::shared_ptr<Eigen::MatrixXd> initialize() const;
    std::tuple<Eigen::Index, Eigen::Index> compute_pivot(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    Eigen::Index entering_variable(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    Eigen::Index leaving_variable(std::shared_ptr<Eigen::MatrixXd> standard_form, Eigen::Index entering) const;

    bool check_unbounded(std::shared_ptr<Eigen::MatrixXd> standard_form, Eigen::Index entering) const;
    bool check_feasible(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    StepResult step(std::shared_ptr<Eigen::MatrixXd> standard_form) const;
    Eigen::VectorXd unbounded_solution() const;
};
