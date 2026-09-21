#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "lp/linear_program.hpp"

/// Parses a LinearProgram from a YAML file.
class LPParser {
public:
    /// Loads and validates a YAML file, returning the resulting LinearProgram; throws std::runtime_error on invalid input.
    static std::shared_ptr<LinearProgram> parse(const std::string& filename);

private:
    /// Validates that the root node contains the required LP fields.
    static void validateRoot(const YAML::Node& root); 

    /// Parses a YAML sequence node into an Eigen vector.
    static Eigen::VectorXd parseVector(const YAML::Node& node); 

    /// Parses a YAML sequence-of-sequences node into an Eigen matrix.
    static Eigen::MatrixXd parseMatrix(const YAML::Node& node);

    /// Parses the optimization sense (minimize/maximize) field.
    static LinearProgram::Sense parseSense(const YAML::Node& node);

    /// Parses the variable domain (integer/natural) field.
    static LinearProgram::Domain parseDomain(const YAML::Node& node);

    /// Validates that constraint matrix/vector dimensions are consistent with each other.
    static void validateDimensions(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::VectorXd& c);
};
