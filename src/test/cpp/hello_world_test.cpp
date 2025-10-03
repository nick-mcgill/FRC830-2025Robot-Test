#include <gtest/gtest.h>
#include <iostream>

TEST(HelloWorldTest, BasicAssertions) {
    EXPECT_EQ(1 + 1, 2);
}