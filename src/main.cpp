#include "lp/solver.hpp"
#include <Eigen/Dense>
#include <iostream>

int main() {

    const Eigen::MatrixXd inequalities{{1.0, 2.0}, {2.0, 1.0}};
    Eigen::VectorXd inequalities_rhs(2);
    inequalities_rhs << 6.0, 8.0;
    Eigen::VectorXd maximization_function(2);
    maximization_function << 3.0, 2.0;

    NaturalLinearProgram linear_program(inequalities, inequalities_rhs, maximization_function);
    SimplexSolver simplex(linear_program);

    std::cout << simplex.solve() << std::endl;

    return 0;
}
