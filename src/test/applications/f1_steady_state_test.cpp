#include "gtest/gtest.h"
#include "src/core/applications/steady_state.h"

#include "lion/frame/frame.h"
#include "src/core/vehicles/limebeer2014f1.h"
#include "lion/math/optimise.h"

extern bool is_valgrind;

class Steady_state_test_f1 : public ::testing::Test
{
 protected:
    Steady_state_test_f1() {}
    Xml_document database = {"./database/limebeer-2014-f1.xml", true};
    limebeer2014f1<CppAD::AD<scalar>>::cartesian car = { database };
    limebeer2014f1<scalar>::cartesian car_sc = { database };
};

TEST_F(Steady_state_test_f1, _0g_0g_300kmh)
{
    const double v = 300.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, 0.0, 1.0e-4);
    EXPECT_NEAR(solution.ay, 0.0, 1.0e-4);
    EXPECT_NEAR(solution.u[0], 0.0, 1.0e-4);
}

TEST_F(Steady_state_test_f1, max_longitudinal_acceleration)
{
    double ay = 0.0;

    const scalar v = 150.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_TRUE(solution_min.solved);

    PRINTVARIABLE(JMT, solution_max.ax);
    PRINTVARIABLE(JMT, solution_min.ax);

    // Check the minimum acceleration solution
    car_sc(solution_min.q, solution_min.qa, solution_min.u, 0.0);

    PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa());
    PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa());
    PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa());
    PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa());
}

TEST_F(Steady_state_test_f1, max_longitudinal_acceleration_several_speeds)
{
    double ay = 0.0;

    size_t n = 25;

    double ax_previous = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 100.0*KMH + 200.0*KMH*i/(n-1);
        Steady_state ss(car);
        auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);
    
        std::cout << v/KMH << std::endl;
        EXPECT_TRUE(solution_max.solved);
        EXPECT_TRUE(solution_min.solved);
        PRINTVARIABLE(JMT, solution_max.ax);
        PRINTVARIABLE(JMT, solution_min.ax);
        std::cout << "Maximum acc: " << std::endl;
        car_sc(solution_max.q, solution_max.qa, solution_max.u, 0.0);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_road().get_psi()*RAD);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD);

        std::cout << "Minimum acc: " << std::endl;
        car_sc(solution_min.q, solution_min.qa, solution_min.u, 0.0);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_road().get_psi()*RAD);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD);

        // For now, just test that the longitudinal acceleration decreases
        EXPECT_TRUE(ax_previous > solution_min.ax);
        ax_previous = solution_min.ax;

    }
}

TEST_F(Steady_state_test_f1, max_longitudinal_acceleration_several_speeds_25percent_lat)
{
    size_t n = 25;

    double ax_previous = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 100.0*KMH + 200.0*KMH*i/(n-1);
        Steady_state ss(car);

        auto solution = ss.solve_max_lat_acc(v);
        double ay = solution.ay*0.25;
        auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);
    
        std::cout << v/KMH << std::endl;
        EXPECT_TRUE(solution_max.solved);
        EXPECT_TRUE(solution_min.solved);
        PRINTVARIABLE(JMT, solution_max.ax);
        PRINTVARIABLE(JMT, solution_min.ax);
        std::cout << "Maximum acc: " << std::endl;
        car_sc(solution_max.q, solution_max.qa, solution_max.u, 0.0);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_road().get_psi()*RAD);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD);

        std::cout << "Minimum acc: " << std::endl;
        car_sc(solution_min.q, solution_min.qa, solution_min.u, 0.0);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_road().get_psi()*RAD);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD);

        // For now, just test that the longitudinal acceleration decreases
        EXPECT_TRUE(ax_previous > solution_min.ax);
        ax_previous = solution_min.ax;

    }
}


TEST_F(Steady_state_test_f1, max_longitudinal_acceleration_several_speeds_50percent_lat)
{
    size_t n = 25;

    double ax_previous = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 100.0*KMH + 200.0*KMH*i/(n-1);
        Steady_state ss(car);

        auto solution = ss.solve_max_lat_acc(v);
        double ay = solution.ay*0.50;
        auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);
    
        std::cout << v/KMH << std::endl;
        EXPECT_TRUE(solution_max.solved);
        EXPECT_TRUE(solution_min.solved);
        PRINTVARIABLE(JMT, solution_max.ax);
        PRINTVARIABLE(JMT, solution_min.ax);
        std::cout << "Maximum acc: " << std::endl;
        car_sc(solution_max.q, solution_max.qa, solution_max.u, 0.0);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_road().get_psi()*RAD);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD);

        std::cout << "Minimum acc: " << std::endl;
        car_sc(solution_min.q, solution_min.qa, solution_min.u, 0.0);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_road().get_psi()*RAD);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD);

        // For now, just test that the longitudinal acceleration decreases
        EXPECT_TRUE(ax_previous > solution_min.ax);
        ax_previous = solution_min.ax;

    }
}

TEST_F(Steady_state_test_f1, max_longitudinal_acceleration_several_speeds_75percent_lat)
{
    size_t n = 25;

    double ax_previous = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 100.0*KMH + 200.0*KMH*i/(n-1);
        Steady_state ss(car);

        auto solution = ss.solve_max_lat_acc(v);
        double ay = solution.ay*0.75;
        auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);
    
        std::cout << v/KMH << std::endl;
        EXPECT_TRUE(solution_max.solved);
        EXPECT_TRUE(solution_min.solved);
        PRINTVARIABLE(JMT, solution_max.ax);
        PRINTVARIABLE(JMT, solution_min.ax);
        std::cout << "Maximum acc: " << std::endl;
        car_sc(solution_max.q, solution_max.qa, solution_max.u, 0.0);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_road().get_psi()*RAD);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD);

        std::cout << "Minimum acc: " << std::endl;
        car_sc(solution_min.q, solution_min.qa, solution_min.u, 0.0);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_road().get_psi()*RAD);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD);

        // For now, just test that the longitudinal acceleration decreases
        EXPECT_TRUE(ax_previous > solution_min.ax);
        ax_previous = solution_min.ax;

    }
}

TEST_F(Steady_state_test_f1, max_longitudinal_acceleration_several_speeds_95percent_lat)
{
    size_t n = 25;

    double ax_previous = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 100.0*KMH + 200.0*KMH*i/(n-1);
        Steady_state ss(car);

        auto solution = ss.solve_max_lat_acc(v);
        double ay = solution.ay*0.95;
        auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);
    
        std::cout << v/KMH << std::endl;
        EXPECT_TRUE(solution_max.solved);
        EXPECT_TRUE(solution_min.solved);
        PRINTVARIABLE(JMT, solution_max.ax);
        PRINTVARIABLE(JMT, solution_min.ax);
        std::cout << "Maximum acc: " << std::endl;
        car_sc(solution_max.q, solution_max.qa, solution_max.u, 0.0);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_road().get_psi()*RAD);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD);

        std::cout << "Minimum acc: " << std::endl;
        car_sc(solution_min.q, solution_min.qa, solution_min.u, 0.0);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa());
        PRINTVARIABLE(JMT, car_sc.get_road().get_psi()*RAD);
        PRINTVARIABLE(JMT, car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD);

        // For now, just test that the longitudinal acceleration decreases
        EXPECT_TRUE(ax_previous > solution_min.ax);
        ax_previous = solution_min.ax;

    }
}







TEST_F(Steady_state_test_f1, max_lateral_accel_300kmh)
{
    const scalar v = 300.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car_sc(solution.q,solution.qa,solution.u,0.0);
    
    EXPECT_TRUE(solution.solved);
}

TEST_F(Steady_state_test_f1, max_lateral_accel_several_speeds)
{
    const size_t n = 100;
    std::array<double,n> vel, acc;
    double ay_prev = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 250.0*KMH*i/(n-1) + 100.0*KMH;

        std::cout << "vel: " << v/KMH << std::endl;
        Steady_state ss(car);
        auto solution = ss.solve_max_lat_acc(v);
        std::cout << "throttle: " << solution.u[1] << std::endl;

        car_sc(solution.q,solution.qa,solution.u,0.0);
    
        EXPECT_TRUE(solution.solved);
        vel[i] = v;
        acc[i] = solution.ay; 

        // For now, just test that the lateral acceleration increases
        if ( v < 80.0 )
        {
            EXPECT_TRUE(solution.ay > ay_prev);
        }

        ay_prev = solution.ay;
    }

    std::cout << vel << std::endl;
    std::cout << acc << std::endl;
}


/*
TEST_F(Steady_state_test_f1, gg_diagram_60)
{
    if ( is_valgrind ) GTEST_SKIP();

    constexpr size_t n = 20;
    Steady_state ss(car);
    const scalar v = 60.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_f1, gg_diagram_70)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 50;
    Steady_state ss(car);
    const scalar v = 70.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_f1, gg_diagram_80)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 80;
    Steady_state ss(car);
    const scalar v = 80.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_f1, gg_diagram_90)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 110;
    Steady_state ss(car);
    const scalar v = 90.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_f1, gg_diagram_100)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 150;
    Steady_state ss(car);
    const scalar v = 100.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_f1, gg_diagram_110)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 180;
    Steady_state ss(car);
    const scalar v = 110.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_f1, gg_diagram_120)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 200;
    Steady_state ss(car);
    const scalar v = 120.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}


TEST_F(Steady_state_test_f1, _0g_0g_50kmh)
{
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/zero_g/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_50/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_50/zero_g/ax").get_value(double());

    const double v = 50.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.q,solution.u,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_lateral_accel_50kmh)
{
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_50/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lat_acc/ax").get_value(double());

    const scalar v = 50.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.q,solution.u,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay2_50kmh)
{
    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_2/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay2_50kmh)
{
    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_2/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay5_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_5/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay5_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_5/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay8_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_8/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay8_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_8/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay10_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_10/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay10_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_10/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay12_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_12/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay12_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_12/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay14_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/max_lon_ay_14/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay14_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_50/min_lon_ay_14/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, _0g_0g_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/zero_g/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_60/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_60/zero_g/ax").get_value(double());

    const double v = 60.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.q,solution.u,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_lateral_accel_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_60/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lat_acc/ax").get_value(double());

    const scalar v = 60.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.q,solution.u,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay2_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_2/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay2_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_2/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay5_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_5/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay5_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_5/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay8_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_8/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay8_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_8/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay10_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_10/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay10_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_10/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay12_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_12/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay12_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_12/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay14_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/max_lon_ay_14/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay14_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_60/min_lon_ay_14/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, _0g_0g_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/zero_g/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_70/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_70/zero_g/ax").get_value(double());

    const double v = 70.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.q,solution.u,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_lateral_accel_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_70/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lat_acc/ax").get_value(double());

    const scalar v = 70.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.q,solution.u,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay2_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_2/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay2_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_2/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay5_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_5/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay5_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_5/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay8_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_8/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay8_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_8/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay10_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_10/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay10_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_10/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay12_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_12/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay12_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_12/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay14_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/max_lon_ay_14/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay14_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_70/min_lon_ay_14/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, _0g_0g_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/zero_g/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_80/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_80/zero_g/ax").get_value(double());

    const double v = 80.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.q,solution.u,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_lateral_accel_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_80/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lat_acc/ax").get_value(double());

    const scalar v = 80.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.q,solution.u,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay2_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_2/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay2_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_2/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay5_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_5/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay5_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_5/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay8_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_8/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay8_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_8/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay10_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_10/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay10_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_10/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay12_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_12/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay12_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_12/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay14_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/max_lon_ay_14/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay14_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_80/min_lon_ay_14/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, _0g_0g_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/zero_g/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_90/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_90/zero_g/ax").get_value(double());

    const double v = 90.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.q,solution.u,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_lateral_accel_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_90/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lat_acc/ax").get_value(double());

    const scalar v = 90.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.q,solution.u,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay2_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_2/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay2_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_2/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay5_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_5/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay5_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_5/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay8_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_8/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay8_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_8/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay10_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_10/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay10_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_10/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay12_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_12/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay12_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_12/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay14_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/max_lon_ay_14/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay14_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_90/min_lon_ay_14/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, _0g_0g_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/zero_g/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_100/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_100/zero_g/ax").get_value(double());

    const double v = 100.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.q,solution.u,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_lateral_accel_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_100/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lat_acc/ax").get_value(double());

    const scalar v = 100.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.q,solution.u,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay2_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_2/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay2_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_2/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay5_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_5/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay5_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_5/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay8_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_8/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay8_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_8/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay10_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_10/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay10_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_10/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay12_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_12/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay12_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_12/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay14_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/max_lon_ay_14/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay14_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_100/min_lon_ay_14/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, _0g_0g_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/zero_g/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_110/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_110/zero_g/ax").get_value(double());

    const double v = 110.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.q,solution.u,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_lateral_accel_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_110/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lat_acc/ax").get_value(double());

    const scalar v = 110.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.q,solution.u,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay2_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_2/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay2_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_2/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay5_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_5/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay5_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_5/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay8_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_8/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay8_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_8/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay10_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_10/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay10_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_10/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay12_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_12/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay12_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_12/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay14_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/max_lon_ay_14/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay14_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_110/min_lon_ay_14/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, _0g_0g_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/zero_g/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/zero_g/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_120/zero_g/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_120/zero_g/ax").get_value(double());

    const double v = 120.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.q,solution.u,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-07) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_lateral_accel_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = results.get_root_element().get_child("velocity_120/max_lat_acc/ay").get_value(double());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lat_acc/ax").get_value(double());

    const scalar v = 120.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.q,solution.u,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}

TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay2_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_2/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay2_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 2.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_2/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay5_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_5/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay5_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_5/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay8_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_8/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay8_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_8/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay10_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_10/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay10_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_10/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}




TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay12_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_12/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay12_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_12/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}





TEST_F(Steady_state_test_f1, max_longitudinal_accel_ay14_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/max_lon_ay_14/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}


TEST_F(Steady_state_test_f1, min_longitudinal_accel_ay14_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> q_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> u_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = results.get_root_element().get_child("velocity_120/min_lon_ay_14/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.q[i], q_saved[i], 2.0e-4*std::max(1.0,q_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.u[i], u_saved[i], 2.0e-4) << "with i = " << i;
}
*/
