#include <gtest/gtest.h>
#include "interface/polynomial_3d_trajectory.hpp"

TEST(polynomial_3d_trajectory, sampling)
{
    // sample 100000 point for a 3rd order polynomial randomly generated
    Eigen::MatrixXd coeff = Eigen::MatrixXd::Random(3, 4);
    Polynomial3dTrajectory poly3d(coeff);

    EXPECT_EQ(poly3d.dimension(), 3);
    EXPECT_EQ(poly3d.order(), 3);

    // Check the sample function
    double t = 1.0;
    Eigen::VectorXd sample = poly3d.sample(t);
    Eigen::VectorXd sample2 = coeff.col(0) + coeff.col(1) * t + coeff.col(2) * t * t + coeff.col(3) * t * t * t;
    EXPECT_NEAR(sample(0), sample2(0), 1e-6);
    EXPECT_NEAR(sample(1), sample2(1), 1e-6);
    EXPECT_NEAR(sample(2), sample2(2), 1e-6);

    // for Benchmark
    for (int i = 0; i < 100000; i++)
    {
        double t = (double)rand() / RAND_MAX;
        Eigen::VectorXd sample = poly3d.sample(t);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}