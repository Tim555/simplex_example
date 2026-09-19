#include "lp/linear_program.hpp"

#include <stdexcept>

NaturalLinearProgram::NaturalLinearProgram(const Eigen::MatrixXd &inequalities,
                                         const Eigen::VectorXd &inequalities_rhs,
                                         const Eigen::VectorXd &maximization_function)
    : inequalities_(inequalities), inequalities_rhs_(inequalities_rhs), maximization_function_(maximization_function) {
    if (inequalities_.rows() != inequalities_rhs_.size()) {
        throw std::invalid_argument("inequalities_rhs must have one value per inequality row");
    }

    if (inequalities_.cols() != maximization_function_.size()) {
        throw std::invalid_argument("maximization_function must have one value per inequality column");
    }
}

const Eigen::MatrixXd &NaturalLinearProgram::inequalities() const noexcept {
    return inequalities_;
}

const Eigen::VectorXd &NaturalLinearProgram::inequalities_rhs() const noexcept {
    return inequalities_rhs_;
}

const Eigen::VectorXd &NaturalLinearProgram::maximization_function() const noexcept {
    return maximization_function_;
}
