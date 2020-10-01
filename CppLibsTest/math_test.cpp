#include "pch.h"

#include "Eigen/Eigen"

TEST(MathTest, Vector) {
    Eigen::Vector3d v1(1, 2, 3);
    EXPECT_DOUBLE_EQ(v1.x(), 1);
    EXPECT_DOUBLE_EQ(v1.y(), 2);
    EXPECT_DOUBLE_EQ(v1.z(), 3);

    Eigen::Vector3d v2 = { 1, 2, 3 };
    EXPECT_EQ(v1, v2);
}
