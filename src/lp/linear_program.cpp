#include "lp/linear_program.hpp"
#include <stdexcept>

NonNegativeLinearProgram::NonNegativeLinearProgram(const Eigen::MatrixXd &inequalities,
                                                 const Eigen::VectorXd &inequalities_rhs,
                                                 const Eigen::VectorXd &maximization_function,
                                                 LinearProgram::Sense sense,
                                                 LinearProgram::Domain domain)
    : inequalities_(inequalities), inequalities_rhs_(inequalities_rhs),
      maximization_function_(maximization_function), sense_(sense), domain_(domain) {
    if (inequalities_.rows() != inequalities_rhs_.size()) {
        throw std::invalid_argument("inequalities_rhs must have one value per inequality row");
    }

    if (inequalities_.cols() != maximization_function_.size()) {
        throw std::invalid_argument(
            "maximization_function must have one value per inequality column");
    }

    effective_maximization_function_ =
        sense_ == LinearProgram::Sense::Minimize ? -maximization_function_ : maximization_function_;
}

const Eigen::MatrixXd &NonNegativeLinearProgram::inequalities() const noexcept {
    return inequalities_;
}

const Eigen::VectorXd &NonNegativeLinearProgram::inequalities_rhs() const noexcept {
    return inequalities_rhs_;
}

const Eigen::VectorXd &NonNegativeLinearProgram::maximization_function() const noexcept {
    return effective_maximization_function_;
}

LinearProgram::Sense NonNegativeLinearProgram::sense() const noexcept { return sense_; }

LinearProgram::Domain NonNegativeLinearProgram::domain() const noexcept { return domain_; }
