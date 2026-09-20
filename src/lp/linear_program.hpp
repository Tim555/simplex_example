#pragma once

#include <Eigen/Dense>




class LinearProgram {
  public:
    enum class Sense {
        Minimize,
        Maximize
    };

    enum class Domain {
        Integer,
        Natural
    };
    virtual ~LinearProgram() = default;

    virtual const Eigen::MatrixXd &inequalities() const noexcept = 0;
    virtual const Eigen::VectorXd &inequalities_rhs() const noexcept = 0;
    virtual const Eigen::VectorXd &maximization_function() const noexcept = 0;

    virtual LinearProgram::Sense sense() const noexcept = 0;
    virtual LinearProgram::Domain domain() const noexcept = 0;
};

class NaturalLinearProgram : public LinearProgram {
  public:
    NaturalLinearProgram(const Eigen::MatrixXd &inequalities, const Eigen::VectorXd &inequalities_rhs,
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
    LinearProgram::Sense sense_;
    LinearProgram::Domain domain_;
};