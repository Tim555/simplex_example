#include "lp/lp_parser.hpp"
#include <gtest/gtest.h>

TEST(LPParserTest, ParseValidFile) {
    const std::string filename = "test/test_assets/lpparsertest_parsevalidfile.yaml";
    EXPECT_NO_THROW({
        auto model = LPParser::parse(filename);
        EXPECT_EQ(model->inequalities().rows(), model->inequalities_rhs().size());
        EXPECT_EQ(model->inequalities().cols(), model->maximization_function().size());
    });
}
