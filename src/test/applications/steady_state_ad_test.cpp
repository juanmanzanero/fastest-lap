#include "gtest/gtest.h"
#include "src/core/applications/steady_state.h"
#include "src/core/vehicles/lot2016kart.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"

extern bool is_valgrind;

class Steady_state_ad_test_kart : public ::testing::Test
{
 protected:
    Steady_state_ad_test_kart() { car.get_chassis().get_rear_axle().enable_direct_torque(); }
    Xml_document database = {"./database/vehicles/kart/roberto-lot-kart-2016.xml", true};
    Xml_document results  = {"./data/steady_state.xml", true};
    lot2016kart<CppAD::AD<scalar>>::cartesian car = { database };
};

TEST_F(Steady_state_ad_test_kart, gg_diagram_50)
{
    constexpr size_t n = 10;
    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved);
        EXPECT_TRUE(sol_min[i].solved);
    }
}

TEST_F(Steady_state_ad_test_kart, gg_diagram_60)
{
    if ( is_valgrind ) GTEST_SKIP();

    constexpr size_t n = 20;
    Steady_state ss(car);
    const scalar v = 60.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved);
        EXPECT_TRUE(sol_min[i].solved);
    }

}

TEST_F(Steady_state_ad_test_kart, gg_diagram_70)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 50;
    Steady_state ss(car);
    const scalar v = 70.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved);
        EXPECT_TRUE(sol_min[i].solved);
    }

}

TEST_F(Steady_state_ad_test_kart, gg_diagram_80)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 80;
    Steady_state ss(car);
    const scalar v = 80.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved);
        EXPECT_TRUE(sol_min[i].solved);
    }

}

TEST_F(Steady_state_ad_test_kart, gg_diagram_90)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 110;
    Steady_state ss(car);
    const scalar v = 90.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved);
        EXPECT_TRUE(sol_min[i].solved);
    }

}

TEST_F(Steady_state_ad_test_kart, gg_diagram_100)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 150;
    Steady_state ss(car);
    const scalar v = 100.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved);
        EXPECT_TRUE(sol_min[i].solved);
    }

}

TEST_F(Steady_state_ad_test_kart, gg_diagram_110)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 180;
    Steady_state ss(car);
    const scalar v = 110.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved);
        EXPECT_TRUE(sol_min[i].solved);
    }

}

TEST_F(Steady_state_ad_test_kart, gg_diagram_120)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 200;
    Steady_state ss(car);
    const scalar v = 120.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved);
        EXPECT_TRUE(sol_min[i].solved);
    }

}


TEST_F(Steady_state_ad_test_kart, _0g_0g_50kmh)
{
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_50/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_50/zero_g/ax").get_value(double());

    const double v = 50.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_lateral_accel_50kmh)
{
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_50/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lat_acc/ax").get_value(double());

    const scalar v = 50.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay2_50kmh)
{
    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_2/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay2_50kmh)
{
    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_2/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay5_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_5/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay5_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_5/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay8_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_8/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay8_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_8/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay10_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_10/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay10_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_10/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay12_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_12/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay12_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_12/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay14_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_14/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay14_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_14/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, _0g_0g_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_60/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_60/zero_g/ax").get_value(double());

    const double v = 60.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_lateral_accel_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_60/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lat_acc/ax").get_value(double());

    const scalar v = 60.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay2_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_2/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay2_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_2/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay5_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_5/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay5_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_5/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay8_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_8/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay8_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_8/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay10_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_10/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay10_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_10/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay12_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_12/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay12_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_12/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay14_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_14/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay14_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_14/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, _0g_0g_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_70/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_70/zero_g/ax").get_value(double());

    const double v = 70.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_lateral_accel_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_70/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lat_acc/ax").get_value(double());

    const scalar v = 70.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay2_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_2/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay2_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_2/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay5_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_5/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay5_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_5/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay8_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_8/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay8_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_8/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay10_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_10/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay10_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_10/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay12_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_12/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay12_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_12/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay14_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_14/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay14_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_14/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, _0g_0g_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_80/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_80/zero_g/ax").get_value(double());

    const double v = 80.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_lateral_accel_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_80/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lat_acc/ax").get_value(double());

    const scalar v = 80.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay2_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_2/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay2_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_2/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay5_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_5/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay5_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_5/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay8_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_8/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay8_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_8/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay10_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_10/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay10_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_10/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay12_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_12/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay12_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_12/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay14_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_14/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay14_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_14/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, _0g_0g_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_90/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_90/zero_g/ax").get_value(double());

    const double v = 90.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_lateral_accel_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_90/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lat_acc/ax").get_value(double());

    const scalar v = 90.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay2_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_2/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay2_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_2/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay5_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_5/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay5_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_5/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay8_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_8/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay8_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_8/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay10_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_10/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay10_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_10/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay12_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_12/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay12_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_12/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay14_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_14/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay14_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_14/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, _0g_0g_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_100/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_100/zero_g/ax").get_value(double());

    const double v = 100.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_lateral_accel_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_100/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lat_acc/ax").get_value(double());

    const scalar v = 100.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay2_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_2/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay2_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_2/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay5_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_5/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay5_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_5/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay8_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_8/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay8_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_8/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay10_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_10/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay10_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_10/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay12_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_12/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay12_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_12/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay14_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_14/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay14_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_14/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, _0g_0g_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_110/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_110/zero_g/ax").get_value(double());

    const double v = 110.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_lateral_accel_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_110/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lat_acc/ax").get_value(double());

    const scalar v = 110.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay2_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_2/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay2_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_2/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay5_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_5/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay5_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_5/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay8_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_8/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay8_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_8/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay10_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_10/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay10_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_10/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay12_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_12/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay12_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_12/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay14_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_14/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay14_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_14/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, _0g_0g_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_120/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_120/zero_g/ax").get_value(double());

    const double v = 120.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_lateral_accel_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_120/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lat_acc/ax").get_value(double());

    const scalar v = 120.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay2_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_2/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay2_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_2/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay5_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_5/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay5_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_5/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay8_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_8/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay8_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_8/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay10_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_10/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay10_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_10/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay12_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_12/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay12_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_12/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_ad_test_kart, max_longitudinal_accel_ay14_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_14/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_max.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_ad_test_kart, min_longitudinal_accel_ay14_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> inputs_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_14/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_inputs; ++i)
        EXPECT_NEAR(solution_min.inputs[i], inputs_saved[i], 2.0e-4*std::max(1.0,inputs_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::number_of_controls; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;
}
