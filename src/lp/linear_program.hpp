#pragma once

#include <Eigen/Dense>

class LinearProgram {
  public:
    virtual ~LinearProgram() = default;

    virtual const Eigen::MatrixXd &inequalities() const noexcept = 0;
    virtual const Eigen::VectorXd &inequalities_rhs() const noexcept = 0;
    virtual const Eigen::VectorXd &maximization_function() const noexcept = 0;
};

class NaturalLinearProgram : public LinearProgram {
  public:
    NaturalLinearProgram(const Eigen::MatrixXd &inequalities, const Eigen::VectorXd &inequalities_rhs,
                        const Eigen::VectorXd &maximization_function);

    const Eigen::MatrixXd &inequalities() const noexcept override;
    const Eigen::VectorXd &inequalities_rhs() const noexcept override;
    const Eigen::VectorXd &maximization_function() const noexcept override;

  private:
    Eigen::MatrixXd inequalities_;
    Eigen::VectorXd inequalities_rhs_;
    Eigen::VectorXd maximization_function_;
};