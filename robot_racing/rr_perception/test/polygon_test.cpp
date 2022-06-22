#include "rr_object_tracker/polygon.h"
#include <gtest/gtest.h>

TEST(verify_self, verify_self)
{
    EXPECT_TRUE(true);
    EXPECT_FALSE(false);
    EXPECT_EQ(0, 0);
    EXPECT_NE(1, 0);
    EXPECT_NEAR(1.0, 1.009, 0.01);
}

TEST(area, area)
{
    rr_perception::Polygon poly;
    int n = 1000;
    for (int i = 0; i < n; ++i)
    {
        float th = 2 * M_PI / n * i;
        Eigen::Vector2f x(cos(th), sin(th));
        poly.push_back(x);
    }
    EXPECT_NEAR(rr_perception::Area(poly), M_PI, 0.00001);
}

TEST(area, area0)
{
    rr_perception::Polygon b0;
    b0.push_back(Eigen::Vector2f(0, 0));
    b0.push_back(Eigen::Vector2f(1, 0));
    b0.push_back(Eigen::Vector2f(1, 1));
    b0.push_back(Eigen::Vector2f(0, 1));
    EXPECT_NEAR(rr_perception::Area(b0), 1, 0.00001);
}

TEST(intersection, area)
{
    rr_perception::Polygon b0;
    rr_perception::Polygon b1;
    rr_perception::Polygon inter;

    b0.push_back(Eigen::Vector2f(0, 0));
    b0.push_back(Eigen::Vector2f(1, 0));
    b0.push_back(Eigen::Vector2f(1, 1));
    b0.push_back(Eigen::Vector2f(0, 1));

    b1.push_back(Eigen::Vector2f(0.5, 0.5));
    b1.push_back(Eigen::Vector2f(1.5, 0.5));
    b1.push_back(Eigen::Vector2f(1.5, 1.5));
    b1.push_back(Eigen::Vector2f(0.5, 1.5));
    for (int i = 0; i < 1000; ++i)
    {
        EXPECT_TRUE(rr_perception::Intersection(b0, b1, inter));
        EXPECT_NEAR(rr_perception::Area(inter), 0.25, 0.00001);
    }
}