#include "rr_object_tracker/lap.h"
#include <gtest/gtest.h>
#include <iostream>

TEST(linear_assginment_problem, solve)
{
    rr_perception::LAPJVAlgorithm sol;
    Eigen::MatrixXf mat(3, 4);
    mat << -1, -4, -7, -2,
        -5, -2, -4, -9,
        -4, -6, -2, -1;

    EXPECT_TRUE(sol.Solve(mat));
    EXPECT_EQ(sol.Solution().size(), 3);

    for (auto &s : sol.Solution())
    {
        EXPECT_TRUE((s.row == 0 && s.col == 2) ||
                    (s.row == 1 && s.col == 3) ||
                    (s.row == 2 && s.col == 1));
    }
}