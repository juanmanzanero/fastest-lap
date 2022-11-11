#include "gtest/gtest.h"
#include "src/core/applications/steady_state.h"

#include "lion/frame/frame.h"
#include "src/core/vehicles/lot2016kart.h"
#include "lion/propagators/rk4.h"
#include "lion/math/optimise.h"

extern bool is_valgrind;

class Steady_state_test_kart : public ::testing::Test
{
 protected:
    Steady_state_test_kart() { car.get_chassis().get_rear_axle().enable_direct_torque(); }
    Xml_document database = {"./database/vehicles/kart/roberto-lot-kart-2016.xml", true};
    lot2016kart<scalar>::cartesian car = { database };
    Xml_document references  = {"./data/steady_state.xml", true};
};

template<typename T>
static std::string vec2str(const T& vec)
{
    std::ostringstream s_out; s_out << std::setprecision(17);

    if ( vec.size() == 0 ) return {};

    for (auto it = vec.cbegin() ; it != vec.cend() - 1; ++it)
        s_out << *it << ", " ;

    s_out << vec.back();

    return s_out.str();
}

template<>
std::string vec2str<double>(const double& vec)
{
    std::ostringstream s_out; s_out << std::setprecision(17);
    s_out << vec;
    return s_out.str();
}



struct Steady_state_test_kart_new_reference
{
    Steady_state_test_kart_new_reference(bool save_) : save(save_) { doc.create_root_element("steady_state_test_kart"); }
    ~Steady_state_test_kart_new_reference() { if (save) doc.save("steady_state_test_kart_new_reference.xml"); }
    Xml_document doc;
    bool save;
};

static bool save_xml = true;
static Steady_state_test_kart_new_reference new_reference(save_xml);


TEST_F(Steady_state_test_kart, gg_diagram_50)
{
    constexpr size_t n = 10;
    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_kart, gg_diagram_60)
{
    if ( is_valgrind ) GTEST_SKIP();

    constexpr size_t n = 20;
    Steady_state ss(car);
    const scalar v = 60.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_kart, gg_diagram_70)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 50;
    Steady_state ss(car);
    const scalar v = 70.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_kart, gg_diagram_80)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 80;
    Steady_state ss(car);
    const scalar v = 80.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_kart, gg_diagram_90)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 110;
    Steady_state ss(car);
    const scalar v = 90.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_kart, gg_diagram_100)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 150;
    Steady_state ss(car);
    const scalar v = 100.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_kart, gg_diagram_110)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 180;
    Steady_state ss(car);
    const scalar v = 110.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}

TEST_F(Steady_state_test_kart, gg_diagram_120)
{
    if ( is_valgrind ) GTEST_SKIP();
    constexpr size_t n = 200;
    Steady_state ss(car);
    const scalar v = 120.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);
}


TEST_F(Steady_state_test_kart, _0g_0g_50kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/zero_g/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_50/zero_g/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_50/zero_g/ax").get_value(double());

    const double v = 50.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.input_states,solution.controls,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/zero_g/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/zero_g/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/zero_g/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/zero_g/ax").set_value(vec2str(solution.ax));
}


TEST_F(Steady_state_test_kart, max_lateral_accel_50kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_50/max_lat_acc/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_50/max_lat_acc/ax").get_value(double());

    const scalar v = 50.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.input_states,solution.controls,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;


    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lat_acc/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lat_acc/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lat_acc/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lat_acc/ax").set_value(vec2str(solution.ax));
}

TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay2_50kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_2/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_2/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_2/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_2/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay2_50kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_2/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_2/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_2/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_2/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay5_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_5/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_5/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_5/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_5/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay5_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_5/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_5/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_5/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_5/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay8_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_8/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_8/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_8/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_8/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay8_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_8/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_8/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_8/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_8/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay10_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_10/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_10/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_10/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_10/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay10_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_10/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_10/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_10/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_10/ax").set_value(vec2str(solution_min.ax));
}




TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay12_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_12/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_12/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_12/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_12/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay12_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_12/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_12/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_12/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_12/ax").set_value(vec2str(solution_min.ax));
}





TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay14_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/max_lon_ay_14/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_14/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_14/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/max_lon_ay_14/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay14_50kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_50/min_lon_ay_14/ax").get_value(double());

    const scalar v = 50.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_14/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_14/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_50/min_lon_ay_14/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, _0g_0g_60kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/zero_g/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_60/zero_g/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_60/zero_g/ax").get_value(double());

    const double v = 60.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.input_states,solution.controls,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/zero_g/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/zero_g/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/zero_g/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/zero_g/ax").set_value(vec2str(solution.ax));
}


TEST_F(Steady_state_test_kart, max_lateral_accel_60kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_60/max_lat_acc/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_60/max_lat_acc/ax").get_value(double());

    const scalar v = 60.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.input_states,solution.controls,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;


    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lat_acc/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lat_acc/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lat_acc/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lat_acc/ax").set_value(vec2str(solution.ax));
}

TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay2_60kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_2/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_2/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_2/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_2/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay2_60kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_2/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_2/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_2/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_2/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay5_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_5/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_5/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_5/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_5/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay5_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_5/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_5/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_5/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_5/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay8_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_8/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_8/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_8/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_8/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay8_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_8/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_8/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_8/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_8/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay10_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_10/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_10/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_10/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_10/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay10_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_10/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_10/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_10/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_10/ax").set_value(vec2str(solution_min.ax));
}




TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay12_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_12/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_12/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_12/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_12/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay12_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_12/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_12/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_12/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_12/ax").set_value(vec2str(solution_min.ax));
}





TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay14_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/max_lon_ay_14/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_14/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_14/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/max_lon_ay_14/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay14_60kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_60/min_lon_ay_14/ax").get_value(double());

    const scalar v = 60.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_14/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_14/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_60/min_lon_ay_14/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, _0g_0g_70kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/zero_g/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_70/zero_g/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_70/zero_g/ax").get_value(double());

    const double v = 70.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.input_states,solution.controls,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/zero_g/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/zero_g/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/zero_g/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/zero_g/ax").set_value(vec2str(solution.ax));
}


TEST_F(Steady_state_test_kart, max_lateral_accel_70kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_70/max_lat_acc/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_70/max_lat_acc/ax").get_value(double());

    const scalar v = 70.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.input_states,solution.controls,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;


    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lat_acc/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lat_acc/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lat_acc/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lat_acc/ax").set_value(vec2str(solution.ax));
}

TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay2_70kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_2/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_2/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_2/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_2/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay2_70kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_2/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_2/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_2/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_2/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay5_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_5/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_5/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_5/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_5/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay5_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_5/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_5/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_5/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_5/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay8_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_8/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_8/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_8/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_8/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay8_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_8/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_8/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_8/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_8/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay10_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_10/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_10/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_10/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_10/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay10_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_10/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_10/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_10/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_10/ax").set_value(vec2str(solution_min.ax));
}




TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay12_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_12/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_12/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_12/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_12/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay12_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_12/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_12/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_12/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_12/ax").set_value(vec2str(solution_min.ax));
}





TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay14_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/max_lon_ay_14/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_14/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_14/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/max_lon_ay_14/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay14_70kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_70/min_lon_ay_14/ax").get_value(double());

    const scalar v = 70.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_14/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_14/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_70/min_lon_ay_14/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, _0g_0g_80kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/zero_g/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_80/zero_g/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_80/zero_g/ax").get_value(double());

    const double v = 80.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.input_states,solution.controls,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/zero_g/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/zero_g/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/zero_g/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/zero_g/ax").set_value(vec2str(solution.ax));
}


TEST_F(Steady_state_test_kart, max_lateral_accel_80kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_80/max_lat_acc/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_80/max_lat_acc/ax").get_value(double());

    const scalar v = 80.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.input_states,solution.controls,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;


    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lat_acc/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lat_acc/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lat_acc/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lat_acc/ax").set_value(vec2str(solution.ax));
}

TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay2_80kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_2/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_2/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_2/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_2/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay2_80kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_2/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_2/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_2/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_2/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay5_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_5/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_5/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_5/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_5/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay5_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_5/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_5/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_5/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_5/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay8_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_8/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_8/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_8/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_8/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay8_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_8/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_8/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_8/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_8/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay10_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_10/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_10/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_10/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_10/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay10_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_10/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_10/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_10/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_10/ax").set_value(vec2str(solution_min.ax));
}




TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay12_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_12/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_12/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_12/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_12/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay12_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_12/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_12/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_12/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_12/ax").set_value(vec2str(solution_min.ax));
}





TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay14_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/max_lon_ay_14/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_14/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_14/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/max_lon_ay_14/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay14_80kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_80/min_lon_ay_14/ax").get_value(double());

    const scalar v = 80.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_14/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_14/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_80/min_lon_ay_14/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, _0g_0g_90kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/zero_g/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_90/zero_g/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_90/zero_g/ax").get_value(double());

    const double v = 90.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.input_states,solution.controls,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/zero_g/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/zero_g/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/zero_g/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/zero_g/ax").set_value(vec2str(solution.ax));
}


TEST_F(Steady_state_test_kart, max_lateral_accel_90kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_90/max_lat_acc/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_90/max_lat_acc/ax").get_value(double());

    const scalar v = 90.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.input_states,solution.controls,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;


    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lat_acc/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lat_acc/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lat_acc/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lat_acc/ax").set_value(vec2str(solution.ax));
}

TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay2_90kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_2/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_2/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_2/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_2/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay2_90kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_2/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_2/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_2/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_2/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay5_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_5/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_5/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_5/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_5/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay5_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_5/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_5/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_5/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_5/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay8_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_8/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_8/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_8/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_8/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay8_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_8/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_8/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_8/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_8/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay10_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_10/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_10/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_10/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_10/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay10_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_10/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_10/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_10/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_10/ax").set_value(vec2str(solution_min.ax));
}




TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay12_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_12/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_12/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_12/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_12/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay12_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_12/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_12/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_12/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_12/ax").set_value(vec2str(solution_min.ax));
}





TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay14_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/max_lon_ay_14/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_14/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_14/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/max_lon_ay_14/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay14_90kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_90/min_lon_ay_14/ax").get_value(double());

    const scalar v = 90.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_14/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_14/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_90/min_lon_ay_14/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, _0g_0g_100kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/zero_g/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_100/zero_g/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_100/zero_g/ax").get_value(double());

    const double v = 100.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.input_states,solution.controls,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/zero_g/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/zero_g/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/zero_g/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/zero_g/ax").set_value(vec2str(solution.ax));
}


TEST_F(Steady_state_test_kart, max_lateral_accel_100kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_100/max_lat_acc/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_100/max_lat_acc/ax").get_value(double());

    const scalar v = 100.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.input_states,solution.controls,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;


    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lat_acc/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lat_acc/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lat_acc/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lat_acc/ax").set_value(vec2str(solution.ax));
}

TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay2_100kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_2/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_2/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_2/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_2/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay2_100kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_2/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_2/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_2/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_2/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay5_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_5/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_5/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_5/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_5/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay5_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_5/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_5/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_5/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_5/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay8_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_8/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_8/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_8/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_8/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay8_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_8/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_8/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_8/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_8/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay10_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_10/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_10/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_10/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_10/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay10_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_10/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_10/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_10/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_10/ax").set_value(vec2str(solution_min.ax));
}




TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay12_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_12/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_12/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_12/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_12/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay12_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_12/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_12/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_12/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_12/ax").set_value(vec2str(solution_min.ax));
}





TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay14_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/max_lon_ay_14/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_14/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_14/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/max_lon_ay_14/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay14_100kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_100/min_lon_ay_14/ax").get_value(double());

    const scalar v = 100.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_14/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_14/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_100/min_lon_ay_14/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, _0g_0g_110kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/zero_g/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_110/zero_g/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_110/zero_g/ax").get_value(double());

    const double v = 110.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.input_states,solution.controls,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/zero_g/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/zero_g/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/zero_g/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/zero_g/ax").set_value(vec2str(solution.ax));
}


TEST_F(Steady_state_test_kart, max_lateral_accel_110kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_110/max_lat_acc/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_110/max_lat_acc/ax").get_value(double());

    const scalar v = 110.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.input_states,solution.controls,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;


    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lat_acc/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lat_acc/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lat_acc/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lat_acc/ax").set_value(vec2str(solution.ax));
}

TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay2_110kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_2/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_2/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_2/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_2/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay2_110kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_2/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_2/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_2/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_2/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay5_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_5/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_5/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_5/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_5/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay5_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_5/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_5/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_5/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_5/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay8_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_8/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_8/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_8/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_8/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay8_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_8/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_8/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_8/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_8/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay10_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_10/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_10/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_10/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_10/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay10_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_10/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_10/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_10/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_10/ax").set_value(vec2str(solution_min.ax));
}




TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay12_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_12/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_12/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_12/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_12/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay12_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_12/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_12/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_12/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_12/ax").set_value(vec2str(solution_min.ax));
}





TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay14_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/max_lon_ay_14/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_14/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_14/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/max_lon_ay_14/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay14_110kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_110/min_lon_ay_14/ax").get_value(double());

    const scalar v = 110.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_14/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_14/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_110/min_lon_ay_14/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, _0g_0g_120kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/zero_g/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/zero_g/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_120/zero_g/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_120/zero_g/ax").get_value(double());

    const double v = 120.0*KMH;
    const double ax = 0;
    const double ay = 0;

    Steady_state ss(car);
    auto solution = ss.solve(v,ax,ay);

    car(solution.input_states,solution.controls,0.0);

    EXPECT_TRUE(solution.solved);
    EXPECT_NEAR(solution.ax, ax_saved, 1.0e-7);
    EXPECT_NEAR(solution.ay, ay_saved, 1.0e-7);
    

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-07) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-07) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/zero_g/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/zero_g/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/zero_g/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/zero_g/ax").set_value(vec2str(solution.ax));
}


TEST_F(Steady_state_test_kart, max_lateral_accel_120kmh)
{
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/max_lat_acc/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/max_lat_acc/u").get_value(std::vector<double>());
    double ay_saved = references.get_root_element().get_child("velocity_120/max_lat_acc/ay").get_value(double());
    double ax_saved = references.get_root_element().get_child("velocity_120/max_lat_acc/ax").get_value(double());

    const scalar v = 120.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car(solution.input_states,solution.controls,0.0);
    
    EXPECT_TRUE(solution.solved);

    EXPECT_NEAR(solution.ax, ax_saved, 2.0e-4);
    EXPECT_NEAR(solution.ay, ay_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;


    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lat_acc/q").set_value(vec2str(solution.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lat_acc/u").set_value(vec2str(solution.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lat_acc/ay").set_value(vec2str(solution.ay));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lat_acc/ax").set_value(vec2str(solution.ax));
}

TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay2_120kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_2/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_2/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_2/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_2/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay2_120kmh)
{
    double ay = 2.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_2/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_2/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_2/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_2/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_2/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_2/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay5_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_5/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_5/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_5/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_5/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay5_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 5.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_5/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_5/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_5/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_5/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_5/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_5/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay8_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_8/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_8/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_8/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_8/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay8_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 8.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_8/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_8/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_8/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_8/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_8/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_8/ax").set_value(vec2str(solution_min.ax));
}


TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay10_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_10/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_10/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_10/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_10/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay10_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 10.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_10/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_10/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_10/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_10/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_10/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_10/ax").set_value(vec2str(solution_min.ax));
}




TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay12_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_12/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_12/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_12/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_12/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay12_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 12.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_12/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_12/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_12/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_12/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_12/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_12/ax").set_value(vec2str(solution_min.ax));
}





TEST_F(Steady_state_test_kart, max_longitudinal_accel_ay14_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/max_lon_ay_14/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_NEAR(solution_max.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_max.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_max.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_max.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_14/q").set_value(vec2str(solution_max.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_14/u").set_value(vec2str(solution_max.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/max_lon_ay_14/ax").set_value(vec2str(solution_max.ax));
}


TEST_F(Steady_state_test_kart, min_longitudinal_accel_ay14_120kmh)
{
    if ( is_valgrind ) GTEST_SKIP();

    double ay = 14.0;
    std::vector<double> input_states_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_14/q").get_value(std::vector<double>());
    std::vector<double> controls_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_14/u").get_value(std::vector<double>());
    double ax_saved = references.get_root_element().get_child("velocity_120/min_lon_ay_14/ax").get_value(double());

    const scalar v = 120.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_min.solved);
    EXPECT_NEAR(solution_min.ay, ay, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, ax_saved, 2.0e-4);

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NSTATE; ++i)
        EXPECT_NEAR(solution_min.input_states[i], input_states_saved[i], 2.0e-4*std::max(1.0,input_states_saved[i])) << "with i = " << i;

    for (size_t i = 0; i < lot2016kart<scalar>::cartesian::NCONTROL; ++i)
        EXPECT_NEAR(solution_min.controls[i], controls_saved[i], 2.0e-4) << "with i = " << i;

    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_14/q").set_value(vec2str(solution_min.input_states));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_14/u").set_value(vec2str(solution_min.controls));
    new_reference.doc.add_element("steady_state_test_kart/velocity_120/min_lon_ay_14/ax").set_value(vec2str(solution_min.ax));
}

