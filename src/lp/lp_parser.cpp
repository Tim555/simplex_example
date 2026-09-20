#include "lp_parser.hpp"

std::shared_ptr<LinearProgram> LPParser::parse(const std::string& filename) {
    YAML::Node root = YAML::LoadFile(filename);

        validateRoot(root);

        auto A = parseMatrix(root["data"]["A"]);
        auto b = parseVector(root["data"]["b"]);
        auto c = parseVector(root["data"]["c"]);

        auto sense = parseSense(root["problem"]["sense"]);
        auto domain = parseDomain(root["problem"]["domain"]);

        validateDimensions(A, b, c);

        if (domain == LinearProgram::Domain::Natural) {
            return std::make_shared<NaturalLinearProgram>(A, b, c, sense, domain);
        }
        throw std::runtime_error("Unsupported domain");
}
        
void LPParser::validateRoot(const YAML::Node& root) {
    if (!root["data"])
        throw std::runtime_error("Missing 'data' section");

    if (!root["problem"])
        throw std::runtime_error("Missing 'problem' section");

    for (const char* key : {"A", "b", "c"}) {
        if (!root["data"][key])
            throw std::runtime_error(
                std::string("Missing data field '") + key + "'"
            );
    }

    if (!root["problem"]["sense"])
        throw std::runtime_error("Missing 'problem.sense'");

    if (!root["problem"]["domain"])
        throw std::runtime_error("Missing 'problem.domain'");
}

Eigen::VectorXd LPParser::parseVector(const YAML::Node& node) {
    if (!node.IsSequence())
        throw std::runtime_error("Expected a vector");

    const Eigen::Index n = node.size();
    Eigen::VectorXd result(n);

    for (Eigen::Index i = 0; i < n; ++i) {
        if (!node[i].IsScalar())
            throw std::runtime_error(
                "Vector contains non-scalar value"
            );

        result(i) = node[i].as<double>();
    }

    return result;
}

Eigen::MatrixXd LPParser::parseMatrix(const YAML::Node& node) {
    if (!node.IsSequence())
        throw std::runtime_error("Expected a matrix");

    const Eigen::Index rows = node.size();

    if (rows == 0)
        return Eigen::MatrixXd(0, 0);

    if (!node[0].IsSequence())
        throw std::runtime_error("Expected matrix rows");

    const Eigen::Index cols = node[0].size();

    Eigen::MatrixXd result(rows, cols);

    for (Eigen::Index i = 0; i < rows; ++i) {
        if (!node[i].IsSequence())
            throw std::runtime_error(
                "Matrix contains non-row value"
            );

        if (node[i].size() != cols)
            throw std::runtime_error(
                "Matrix rows have different sizes"
            );

        for (Eigen::Index j = 0; j < cols; ++j) {
            if (!node[i][j].IsScalar())
                throw std::runtime_error(
                    "Matrix contains non-scalar value"
                );

            result(i, j) = node[i][j].as<double>();
        }
    }

    return result;
}

LinearProgram::Sense LPParser::parseSense(const YAML::Node& node) {
    const std::string value = node.as<std::string>();

    if (value == "minimize")
        return LinearProgram::Sense::Minimize;

    if (value == "maximize")
        return LinearProgram::Sense::Maximize;

    throw std::runtime_error(
        "Invalid sense: " + value
    );
}

LinearProgram::Domain LPParser::parseDomain(const YAML::Node& node) {
    const std::string value = node.as<std::string>();

    if (value == "integer")
        return LinearProgram::Domain::Integer;

    if (value == "natural")
        return LinearProgram::Domain::Natural;

    throw std::runtime_error(
        "Invalid domain: " + value
    );
}

void LPParser::validateDimensions(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::VectorXd& c) {
    if (A.rows() != b.size()) {
        throw std::runtime_error(
            "Dimension mismatch: A has " +
            std::to_string(A.rows()) +
            " rows, but b has " +
            std::to_string(b.size()) +
            " elements"
        );
    }

    if (A.cols() != c.size()) {
        throw std::runtime_error(
            "Dimension mismatch: A has " +
            std::to_string(A.cols()) +
            " columns, but c has " +
            std::to_string(c.size()) +
            " elements"
        );
    }
}
