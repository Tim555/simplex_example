#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "lp/linear_program.hpp"


class LPParser {
public:
    static std::shared_ptr<LinearProgram> parse(const std::string& filename);

private:
    static void validateRoot(const YAML::Node& root); 

    static Eigen::VectorXd parseVector(const YAML::Node& node); 

    static Eigen::MatrixXd parseMatrix(const YAML::Node& node);

    static LinearProgram::Sense parseSense(const YAML::Node& node);

    static LinearProgram::Domain parseDomain(const YAML::Node& node);

    static void validateDimensions(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::VectorXd& c);
};
