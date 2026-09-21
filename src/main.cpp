#include "lp/lp_parser.hpp"
#include "lp/solver.hpp"
#include <Eigen/Dense>
#include <iostream>

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " <lp-file.yaml>" << std::endl;
        return 1;
    }

    try {
        const std::shared_ptr<LinearProgram> linear_program = LPParser::parse(argv[1]);
        SimplexSolver simplex(*linear_program);
        std::cout << simplex.solve() << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
