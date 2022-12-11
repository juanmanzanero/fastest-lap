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
    Xml_document database = {"./database/vehicles/f1/limebeer-2014-f1.xml", true};
    limebeer2014f1<CppAD::AD<scalar>>::cartesian car = { database };
    limebeer2014f1<scalar>::cartesian car_sc = { database };
    Xml_document references = {"./data/f1_steady_state_test.xml", true};
};

static std::string vec2str(const std::vector<double>& vec)
{
    std::ostringstream s_out; s_out << std::setprecision(17);

    if ( vec.size() == 0 ) return {};

    for (auto it = vec.cbegin() ; it != vec.cend() - 1; ++it)
        s_out << *it << ", " ;

    s_out << vec.back();

    return s_out.str();
}


struct Steady_state_new_reference
{
    Steady_state_new_reference(bool save_) : save(save_) { doc.create_root_element("f1_steady_state_test"); }
    ~Steady_state_new_reference() { if (save) doc.save("steady_state_f1_new_reference.xml"); }
    Xml_document doc;
    bool save;
};

static bool save_xml = false;
static Steady_state_new_reference new_reference(save_xml);

TEST_F(Steady_state_test_f1, max_lateral_accel_several_speeds)
{
    const size_t n = 100;
    std::vector<double> vel(n), acc(n), ax(n);
    double ay_prev = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 250.0*KMH*i/(n-1) + 100.0*KMH;

        Steady_state ss(car);
        auto solution = ss.solve_max_lat_acc(v);

        car_sc(solution.inputs,solution.controls,0.0);
    
        EXPECT_TRUE(solution.solved);
        vel[i] = v;
        acc[i] = solution.ay; 
        ax[i]  = solution.ax;

        // For now, just test that the lateral acceleration increases
        EXPECT_TRUE(solution.ay > ay_prev);

        ay_prev = solution.ay;
    }

    auto velocity_reference = references.get_element("f1_steady_state_test/max_lateral_acceleration/velocity").get_value(std::vector<scalar>());
    auto ay_reference = references.get_element("f1_steady_state_test/max_lateral_acceleration/acceleration-y").get_value(std::vector<scalar>());
    auto ax_reference = references.get_element("f1_steady_state_test/max_lateral_acceleration/acceleration-x").get_value(std::vector<scalar>());

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(vel[i], velocity_reference[i], 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(acc[i], ay_reference[i], 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(ax[i],  ax_reference[i], 2.0e-4) << ", with i = " << i;
    }

    new_reference.doc.add_element("f1_steady_state_test/max_lateral_acceleration/velocity").set_value(vec2str(vel));
    new_reference.doc.add_element("f1_steady_state_test/max_lateral_acceleration/acceleration-y").set_value(vec2str(acc));
    new_reference.doc.add_element("f1_steady_state_test/max_lateral_acceleration/acceleration-x").set_value(vec2str(ax));
}


TEST_F(Steady_state_test_f1, gg_diagram_100)
{
    if ( is_valgrind ) GTEST_SKIP();

    constexpr size_t n = 49;
    Steady_state ss(car);
    const scalar v = 100.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved) << "with i = " << i;
        EXPECT_TRUE(sol_min[i].solved) << "with i = " << i;
    }

    auto max_ax_reference = references.get_element("f1_steady_state_test/gg_diagram_100/maximum_x_acceleration").get_value(std::vector<scalar>());
    auto min_ax_reference = references.get_element("f1_steady_state_test/gg_diagram_100/minimum_x_acceleration").get_value(std::vector<scalar>());
    auto ay_reference = references.get_element("f1_steady_state_test/gg_diagram_100/y_acceleration").get_value(std::vector<scalar>());

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(sol_max[i].ax, max_ax_reference[i], 2.0e-4);
        EXPECT_NEAR(sol_min[i].ax, min_ax_reference[i], 2.0e-4);
        EXPECT_NEAR(sol_min[i].ay, ay_reference[i], 2.0e-4);
    }

    std::vector<double> ax_max(sol_max.size()), ax_min(sol_min.size()), ay(sol_min.size());

    std::transform(sol_max.cbegin(), sol_max.cend(), ax_max.begin(), [](const auto& sol_max_i) -> auto { return sol_max_i.ax; });
    std::transform(sol_min.cbegin(), sol_min.cend(), ax_min.begin(), [](const auto& sol_min_i) -> auto { return sol_min_i.ax; });
    std::transform(sol_min.cbegin(), sol_min.cend(), ay.begin(), [](const auto& sol_min_i) -> auto { return sol_min_i.ay; });

    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_100/maximum_x_acceleration").set_value(vec2str(ax_max));
    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_100/minimum_x_acceleration").set_value(vec2str(ax_min));
    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_100/y_acceleration").set_value(vec2str(ay));
}



TEST_F(Steady_state_test_f1, gg_diagram_150)
{
    if ( is_valgrind ) GTEST_SKIP();

    constexpr size_t n = 49;
    Steady_state ss(car);
    const scalar v = 150.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved) << "with i = " << i;
        EXPECT_TRUE(sol_min[i].solved) << "with i = " << i;
    }

    auto max_ax_reference = references.get_element("f1_steady_state_test/gg_diagram_150/maximum_x_acceleration").get_value(std::vector<scalar>());
    auto min_ax_reference = references.get_element("f1_steady_state_test/gg_diagram_150/minimum_x_acceleration").get_value(std::vector<scalar>());
    auto ay_reference = references.get_element("f1_steady_state_test/gg_diagram_150/y_acceleration").get_value(std::vector<scalar>());

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(sol_max[i].ax, max_ax_reference[i], 2.0e-4);
        EXPECT_NEAR(sol_min[i].ax, min_ax_reference[i], 2.0e-4);
        EXPECT_NEAR(sol_min[i].ay, ay_reference[i], 2.0e-4);
    }

    std::vector<double> ax_max(sol_max.size()), ax_min(sol_min.size()), ay(sol_min.size());

    std::transform(sol_max.cbegin(), sol_max.cend(), ax_max.begin(), [](const auto& sol_max_i) -> auto { return sol_max_i.ax; });
    std::transform(sol_min.cbegin(), sol_min.cend(), ax_min.begin(), [](const auto& sol_min_i) -> auto { return sol_min_i.ax; });
    std::transform(sol_min.cbegin(), sol_min.cend(), ay.begin(), [](const auto& sol_min_i) -> auto { return sol_min_i.ay; });

    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_150/maximum_x_acceleration").set_value(vec2str(ax_max));
    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_150/minimum_x_acceleration").set_value(vec2str(ax_min));
    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_150/y_acceleration").set_value(vec2str(ay));
}


TEST_F(Steady_state_test_f1, gg_diagram_200)
{
    if ( is_valgrind ) GTEST_SKIP();

    constexpr size_t n = 49;
    Steady_state ss(car);
    const scalar v = 200.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved) << "with i = " << i;
        EXPECT_TRUE(sol_min[i].solved) << "with i = " << i;
    }

    auto max_ax_reference = references.get_element("f1_steady_state_test/gg_diagram_200/maximum_x_acceleration").get_value(std::vector<scalar>());
    auto min_ax_reference = references.get_element("f1_steady_state_test/gg_diagram_200/minimum_x_acceleration").get_value(std::vector<scalar>());
    auto ay_reference = references.get_element("f1_steady_state_test/gg_diagram_200/y_acceleration").get_value(std::vector<scalar>());

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(sol_max[i].ax, max_ax_reference[i], 2.0e-4);
        EXPECT_NEAR(sol_min[i].ax, min_ax_reference[i], 2.0e-4);
        EXPECT_NEAR(sol_min[i].ay, ay_reference[i], 2.0e-4);
    }

    std::vector<double> ax_max(sol_max.size()), ax_min(sol_min.size()), ay(sol_min.size());

    std::transform(sol_max.cbegin(), sol_max.cend(), ax_max.begin(), [](const auto& sol_max_i) -> auto { return sol_max_i.ax; });
    std::transform(sol_min.cbegin(), sol_min.cend(), ax_min.begin(), [](const auto& sol_min_i) -> auto { return sol_min_i.ax; });
    std::transform(sol_min.cbegin(), sol_min.cend(), ay.begin(), [](const auto& sol_min_i) -> auto { return sol_min_i.ay; });

    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_200/maximum_x_acceleration").set_value(vec2str(ax_max));
    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_200/minimum_x_acceleration").set_value(vec2str(ax_min));
    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_200/y_acceleration").set_value(vec2str(ay));
}


TEST_F(Steady_state_test_f1, gg_diagram_250)
{
    if ( is_valgrind ) GTEST_SKIP();

    constexpr size_t n = 49;
    Steady_state ss(car);
    const scalar v = 250.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved) << "with i = " << i;
        EXPECT_TRUE(sol_min[i].solved) << "with i = " << i;
    }

    auto max_ax_reference = references.get_element("f1_steady_state_test/gg_diagram_250/maximum_x_acceleration").get_value(std::vector<scalar>());
    auto min_ax_reference = references.get_element("f1_steady_state_test/gg_diagram_250/minimum_x_acceleration").get_value(std::vector<scalar>());
    auto ay_reference = references.get_element("f1_steady_state_test/gg_diagram_250/y_acceleration").get_value(std::vector<scalar>());

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(sol_max[i].ax, max_ax_reference[i], 2.0e-4);
        EXPECT_NEAR(sol_min[i].ax, min_ax_reference[i], 2.0e-4);
        EXPECT_NEAR(sol_min[i].ay, ay_reference[i], 2.0e-4);
    }

    std::vector<double> ax_max(sol_max.size()), ax_min(sol_min.size()), ay(sol_min.size());

    std::transform(sol_max.cbegin(), sol_max.cend(), ax_max.begin(), [](const auto& sol_max_i) -> auto { return sol_max_i.ax; });
    std::transform(sol_min.cbegin(), sol_min.cend(), ax_min.begin(), [](const auto& sol_min_i) -> auto { return sol_min_i.ax; });
    std::transform(sol_min.cbegin(), sol_min.cend(), ay.begin(), [](const auto& sol_min_i) -> auto { return sol_min_i.ay; });

    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_250/maximum_x_acceleration").set_value(vec2str(ax_max));
    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_250/minimum_x_acceleration").set_value(vec2str(ax_min));
    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_250/y_acceleration").set_value(vec2str(ay));
}


TEST_F(Steady_state_test_f1, gg_diagram_300)
{
    if ( is_valgrind ) GTEST_SKIP();

    constexpr size_t n = 49;
    Steady_state ss(car);
    const scalar v = 300.0*KMH;
    auto [sol_max, sol_min] = ss.gg_diagram(v,n);

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_TRUE(sol_max[i].solved) << "with i = " << i;
        EXPECT_TRUE(sol_min[i].solved) << "with i = " << i;
    }

    auto max_ax_reference = references.get_element("f1_steady_state_test/gg_diagram_300/maximum_x_acceleration").get_value(std::vector<scalar>());
    auto min_ax_reference = references.get_element("f1_steady_state_test/gg_diagram_300/minimum_x_acceleration").get_value(std::vector<scalar>());
    auto ay_reference = references.get_element("f1_steady_state_test/gg_diagram_300/y_acceleration").get_value(std::vector<scalar>());

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(sol_max[i].ax, max_ax_reference[i], 2.0e-4);
        EXPECT_NEAR(sol_min[i].ax, min_ax_reference[i], 2.0e-4);
        EXPECT_NEAR(sol_min[i].ay, ay_reference[i], 2.0e-4);
    }

    std::vector<double> ax_max(sol_max.size()), ax_min(sol_min.size()), ay(sol_min.size());

    std::transform(sol_max.cbegin(), sol_max.cend(), ax_max.begin(), [](const auto& sol_max_i) -> auto { return sol_max_i.ax; });
    std::transform(sol_min.cbegin(), sol_min.cend(), ax_min.begin(), [](const auto& sol_min_i) -> auto { return sol_min_i.ax; });
    std::transform(sol_min.cbegin(), sol_min.cend(), ay.begin(), [](const auto& sol_min_i) -> auto { return sol_min_i.ay; });

    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_300/maximum_x_acceleration").set_value(vec2str(ax_max));
    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_300/minimum_x_acceleration").set_value(vec2str(ax_min));
    new_reference.doc.add_element("f1_steady_state_test/gg_diagram_300/y_acceleration").set_value(vec2str(ay));
}

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
    EXPECT_NEAR(solution.controls[0], 0.0, 1.0e-4);
}


TEST_F(Steady_state_test_f1, max_longitudinal_acceleration)
{
    double ay = 0.0;

    const scalar v = 150.0*KMH;
    Steady_state ss(car);
    auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);

    EXPECT_TRUE(solution_max.solved);
    EXPECT_TRUE(solution_min.solved);

    // Check the minimum acceleration solution
    car_sc(solution_min.inputs, solution_min.controls, 0.0);

    auto max_ax_ref = references.get_element("f1_steady_state_test/maximum_longiudinal_acceleration/maximum_x_acceleration").get_value(scalar());
    auto min_ax_ref = references.get_element("f1_steady_state_test/maximum_longiudinal_acceleration/minimum_x_acceleration").get_value(scalar());
    auto kappa_min_fl_ref = references.get_element("f1_steady_state_test/maximum_longiudinal_acceleration/kappa_fl").get_value(scalar());
    auto kappa_min_fr_ref = references.get_element("f1_steady_state_test/maximum_longiudinal_acceleration/kappa_fr").get_value(scalar());
    auto kappa_min_rl_ref = references.get_element("f1_steady_state_test/maximum_longiudinal_acceleration/kappa_rl").get_value(scalar());
    auto kappa_min_rr_ref = references.get_element("f1_steady_state_test/maximum_longiudinal_acceleration/kappa_rr").get_value(scalar());

    EXPECT_NEAR(solution_max.ax, max_ax_ref, 2.0e-4);
    EXPECT_NEAR(solution_min.ax, min_ax_ref, 2.0e-4);
    EXPECT_NEAR(car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa(), kappa_min_fl_ref, 2.0e-4);
    EXPECT_NEAR(car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa(), kappa_min_fr_ref, 2.0e-4);
    EXPECT_NEAR(car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa(), kappa_min_rl_ref, 2.0e-4);
    EXPECT_NEAR(car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa(), kappa_min_rr_ref, 2.0e-4);

    new_reference.doc.add_element("f1_steady_state_test/maximum_longiudinal_acceleration/maximum_x_acceleration").set_value(vec2str({solution_max.ax}));
    new_reference.doc.add_element("f1_steady_state_test/maximum_longiudinal_acceleration/minimum_x_acceleration").set_value(vec2str({solution_min.ax}));
    new_reference.doc.add_element("f1_steady_state_test/maximum_longiudinal_acceleration/kappa_fl").set_value(vec2str({car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa()}));
    new_reference.doc.add_element("f1_steady_state_test/maximum_longiudinal_acceleration/kappa_fr").set_value(vec2str({car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa()}));
    new_reference.doc.add_element("f1_steady_state_test/maximum_longiudinal_acceleration/kappa_rl").set_value(vec2str({car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa()}));
    new_reference.doc.add_element("f1_steady_state_test/maximum_longiudinal_acceleration/kappa_rr").set_value(vec2str({car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa()}));
}


TEST_F(Steady_state_test_f1, max_longitudinal_acceleration_several_speeds)
{
    double ay = 0.0;

    size_t n = 25;

    double ax_previous = 0.0;
    std::vector<double> ax_min(n), kappa_min_acc_fl(n), kappa_min_acc_fr(n), kappa_min_acc_rl(n), kappa_min_acc_rr(n), psi_min_acc(n), delta_min_acc(n);
    std::vector<double> ax_max(n), kappa_max_acc_fl(n), kappa_max_acc_fr(n), kappa_max_acc_rl(n), kappa_max_acc_rr(n), psi_max_acc(n), delta_max_acc(n);
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 100.0*KMH + 200.0*KMH*i/(n-1);
        Steady_state ss(car);
        auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);
    
        EXPECT_TRUE(solution_max.solved) << "with i = " << i;
        EXPECT_TRUE(solution_min.solved) << "with i = " << i;
        ax_min[i] = solution_min.ax;
        ax_max[i] = solution_max.ax;

        car_sc(solution_max.inputs, solution_max.controls, 0.0);

        kappa_max_acc_fl[i] = car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa();
        kappa_max_acc_fr[i] = car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa();
        kappa_max_acc_rl[i] = car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        kappa_max_acc_rr[i] = car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        psi_max_acc[i] = std::as_const(car_sc).get_road().get_psi()*RAD;
        delta_max_acc[i] = car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD;

        car_sc(solution_min.inputs, solution_min.controls, 0.0);

        kappa_min_acc_fl[i] = car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa();
        kappa_min_acc_fr[i] = car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa();
        kappa_min_acc_rl[i] = car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        kappa_min_acc_rr[i] = car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        psi_min_acc[i] = std::as_const(car_sc).get_road().get_psi()*RAD;
        delta_min_acc[i] = car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD;

        // For now, just test that the longitudinal acceleration decreases
        EXPECT_TRUE(ax_previous > solution_min.ax);
        ax_previous = solution_min.ax;

        if ( is_valgrind ) return;
    }

    auto ax_max_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/maximum_x_acceleration").get_value(std::vector<scalar>());
    auto psi_max_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/psi_max_acc").get_value(std::vector<scalar>());
    auto delta_max_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/delta_max_acc").get_value(std::vector<scalar>());

    auto ax_min_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/minimum_x_acceleration").get_value(std::vector<scalar>());
    auto psi_min_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/psi_min_acc").get_value(std::vector<scalar>());
    auto delta_min_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/delta_min_acc").get_value(std::vector<scalar>());


    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(ax_max[i]           , ax_max_ref[i]           , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(psi_max_acc[i]      , psi_max_acc_ref[i]      , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(delta_max_acc[i]    , delta_max_acc_ref[i]    , 2.0e-4) << ", with i = " << i;

        EXPECT_NEAR(ax_min[i]           , ax_min_ref[i]           , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(psi_min_acc[i]      , psi_min_acc_ref[i]      , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(delta_min_acc[i]    , delta_min_acc_ref[i]    , 2.0e-4) << ", with i = " << i;
    }

    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/maximum_x_acceleration").set_value(vec2str(ax_max));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/psi_max_acc").set_value(vec2str(psi_max_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/delta_max_acc").set_value(vec2str(delta_max_acc));

    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/minimum_x_acceleration").set_value(vec2str(ax_min));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/psi_min_acc").set_value(vec2str(psi_min_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds/delta_min_acc").set_value(vec2str(delta_min_acc));
}


TEST_F(Steady_state_test_f1, max_longitudinal_acceleration_several_speeds_25percent_lat)
{
    size_t n = 25;

    double ax_previous = 0.0;
    std::vector<double> ax_min(n), kappa_min_acc_fl(n), kappa_min_acc_fr(n), kappa_min_acc_rl(n), kappa_min_acc_rr(n), psi_min_acc(n), delta_min_acc(n);
    std::vector<double> ax_max(n), kappa_max_acc_fl(n), kappa_max_acc_fr(n), kappa_max_acc_rl(n), kappa_max_acc_rr(n), psi_max_acc(n), delta_max_acc(n);
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 100.0*KMH + 200.0*KMH*i/(n-1);

        Steady_state ss(car);

        auto solution = ss.solve_max_lat_acc(v);
        double ay = solution.ay*0.25;
        auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);
    
        EXPECT_TRUE(solution_max.solved) << "with i = " << i;
        EXPECT_TRUE(solution_min.solved) << "with i = " << i;
        ax_min[i] = solution_min.ax;
        ax_max[i] = solution_max.ax;

        car_sc(solution_max.inputs, solution_max.controls, 0.0);

        kappa_max_acc_fl[i] = car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa();
        kappa_max_acc_fr[i] = car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa();
        kappa_max_acc_rl[i] = car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        kappa_max_acc_rr[i] = car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        psi_max_acc[i] = std::as_const(car_sc).get_road().get_psi()*RAD;
        delta_max_acc[i] = car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD;

        car_sc(solution_min.inputs, solution_min.controls, 0.0);

        kappa_min_acc_fl[i] = car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa();
        kappa_min_acc_fr[i] = car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa();
        kappa_min_acc_rl[i] = car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        kappa_min_acc_rr[i] = car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        psi_min_acc[i] = std::as_const(car_sc).get_road().get_psi()*RAD;
        delta_min_acc[i] = car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD;

        // For now, just test that the longitudinal acceleration decreases
        EXPECT_TRUE(ax_previous > solution_min.ax);
        ax_previous = solution_min.ax;

        if ( is_valgrind ) return;
    }


    auto ax_max_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/maximum_x_acceleration").get_value(std::vector<scalar>());
    auto kappa_max_acc_fl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_max_acc_fl").get_value(std::vector<scalar>());
    auto kappa_max_acc_fr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_max_acc_fr").get_value(std::vector<scalar>());
    auto kappa_max_acc_rl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_max_acc_rl").get_value(std::vector<scalar>());
    auto kappa_max_acc_rr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_max_acc_rr").get_value(std::vector<scalar>());
    auto psi_max_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/psi_max_acc").get_value(std::vector<scalar>());
    auto delta_max_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/delta_max_acc").get_value(std::vector<scalar>());

    auto ax_min_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/minimum_x_acceleration").get_value(std::vector<scalar>());
    auto kappa_min_acc_fl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_min_acc_fl").get_value(std::vector<scalar>());
    auto kappa_min_acc_fr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_min_acc_fr").get_value(std::vector<scalar>());
    auto kappa_min_acc_rl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_min_acc_rl").get_value(std::vector<scalar>());
    auto kappa_min_acc_rr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_min_acc_rr").get_value(std::vector<scalar>());
    auto psi_min_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/psi_min_acc").get_value(std::vector<scalar>());
    auto delta_min_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/delta_min_acc").get_value(std::vector<scalar>());


    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(ax_max[i]           , ax_max_ref[i]           , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_fl[i] , kappa_max_acc_fl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_fr[i] , kappa_max_acc_fr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_rl[i] , kappa_max_acc_rl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_rr[i] , kappa_max_acc_rr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(psi_max_acc[i]      , psi_max_acc_ref[i]      , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(delta_max_acc[i]    , delta_max_acc_ref[i]    , 2.0e-4) << ", with i = " << i;

        EXPECT_NEAR(ax_min[i]           , ax_min_ref[i]           , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_fl[i] , kappa_min_acc_fl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_fr[i] , kappa_min_acc_fr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_rl[i] , kappa_min_acc_rl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_rr[i] , kappa_min_acc_rr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(psi_min_acc[i]      , psi_min_acc_ref[i]      , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(delta_min_acc[i]    , delta_min_acc_ref[i]    , 2.0e-4) << ", with i = " << i;
    }


    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/maximum_x_acceleration").set_value(vec2str(ax_max));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/psi_max_acc").set_value(vec2str(psi_max_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/delta_max_acc").set_value(vec2str(delta_max_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_max_acc_fl").set_value(vec2str(kappa_max_acc_fl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_max_acc_fr").set_value(vec2str(kappa_max_acc_fr));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_max_acc_rl").set_value(vec2str(kappa_max_acc_rl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_max_acc_rr").set_value(vec2str(kappa_max_acc_rr));

    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/minimum_x_acceleration").set_value(vec2str(ax_min));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/psi_min_acc").set_value(vec2str(psi_min_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/delta_min_acc").set_value(vec2str(delta_min_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_min_acc_fl").set_value(vec2str(kappa_min_acc_fl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_min_acc_fr").set_value(vec2str(kappa_min_acc_fr));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_min_acc_rl").set_value(vec2str(kappa_min_acc_rl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_25percent_lat/kappa_min_acc_rr").set_value(vec2str(kappa_min_acc_rr));
}


TEST_F(Steady_state_test_f1, max_longitudinal_acceleration_several_speeds_50percent_lat)
{
    size_t n = 25;

    double ax_previous = 0.0;
    std::vector<double> ax_min(n), kappa_min_acc_fl(n), kappa_min_acc_fr(n), kappa_min_acc_rl(n), kappa_min_acc_rr(n), psi_min_acc(n), delta_min_acc(n);
    std::vector<double> ax_max(n), kappa_max_acc_fl(n), kappa_max_acc_fr(n), kappa_max_acc_rl(n), kappa_max_acc_rr(n), psi_max_acc(n), delta_max_acc(n);
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 100.0*KMH + 200.0*KMH*i/(n-1);

        Steady_state ss(car);

        auto solution = ss.solve_max_lat_acc(v);
        double ay = solution.ay*0.50;
        auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);
    
        EXPECT_TRUE(solution_max.solved) << "with i = " << i;
        EXPECT_TRUE(solution_min.solved) << "with i = " << i;
        ax_min[i] = solution_min.ax;
        ax_max[i] = solution_max.ax;

        car_sc(solution_max.inputs, solution_max.controls, 0.0);

        kappa_max_acc_fl[i] = car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa();
        kappa_max_acc_fr[i] = car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa();
        kappa_max_acc_rl[i] = car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        kappa_max_acc_rr[i] = car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        psi_max_acc[i] = std::as_const(car_sc).get_road().get_psi()*RAD;
        delta_max_acc[i] = car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD;

        car_sc(solution_min.inputs, solution_min.controls, 0.0);

        kappa_min_acc_fl[i] = car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa();
        kappa_min_acc_fr[i] = car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa();
        kappa_min_acc_rl[i] = car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        kappa_min_acc_rr[i] = car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        psi_min_acc[i] = std::as_const(car_sc).get_road().get_psi()*RAD;
        delta_min_acc[i] = car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD;

        // For now, just test that the longitudinal acceleration decreases
        EXPECT_TRUE(ax_previous > solution_min.ax);
        ax_previous = solution_min.ax;

        if ( is_valgrind ) return;
    }

    auto ax_max_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/maximum_x_acceleration").get_value(std::vector<scalar>());
    auto kappa_max_acc_fl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_max_acc_fl").get_value(std::vector<scalar>());
    auto kappa_max_acc_fr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_max_acc_fr").get_value(std::vector<scalar>());
    auto kappa_max_acc_rl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_max_acc_rl").get_value(std::vector<scalar>());
    auto kappa_max_acc_rr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_max_acc_rr").get_value(std::vector<scalar>());
    auto psi_max_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/psi_max_acc").get_value(std::vector<scalar>());
    auto delta_max_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/delta_max_acc").get_value(std::vector<scalar>());

    auto ax_min_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/minimum_x_acceleration").get_value(std::vector<scalar>());
    auto kappa_min_acc_fl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_min_acc_fl").get_value(std::vector<scalar>());
    auto kappa_min_acc_fr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_min_acc_fr").get_value(std::vector<scalar>());
    auto kappa_min_acc_rl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_min_acc_rl").get_value(std::vector<scalar>());
    auto kappa_min_acc_rr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_min_acc_rr").get_value(std::vector<scalar>());
    auto psi_min_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/psi_min_acc").get_value(std::vector<scalar>());
    auto delta_min_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/delta_min_acc").get_value(std::vector<scalar>());


    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(ax_max[i]           , ax_max_ref[i]           , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_fl[i] , kappa_max_acc_fl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_fr[i] , kappa_max_acc_fr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_rl[i] , kappa_max_acc_rl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_rr[i] , kappa_max_acc_rr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(psi_max_acc[i]      , psi_max_acc_ref[i]      , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(delta_max_acc[i]    , delta_max_acc_ref[i]    , 2.0e-4) << ", with i = " << i;

        EXPECT_NEAR(ax_min[i]           , ax_min_ref[i]           , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_fl[i] , kappa_min_acc_fl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_fr[i] , kappa_min_acc_fr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_rl[i] , kappa_min_acc_rl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_rr[i] , kappa_min_acc_rr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(psi_min_acc[i]      , psi_min_acc_ref[i]      , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(delta_min_acc[i]    , delta_min_acc_ref[i]    , 2.0e-4) << ", with i = " << i;
    }

    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/maximum_x_acceleration").set_value(vec2str(ax_max));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/psi_max_acc").set_value(vec2str(psi_max_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/delta_max_acc").set_value(vec2str(delta_max_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_max_acc_fl").set_value(vec2str(kappa_max_acc_fl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_max_acc_fr").set_value(vec2str(kappa_max_acc_fr));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_max_acc_rl").set_value(vec2str(kappa_max_acc_rl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_max_acc_rr").set_value(vec2str(kappa_max_acc_rr));

    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/minimum_x_acceleration").set_value(vec2str(ax_min));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/psi_min_acc").set_value(vec2str(psi_min_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/delta_min_acc").set_value(vec2str(delta_min_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_min_acc_fl").set_value(vec2str(kappa_min_acc_fl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_min_acc_fr").set_value(vec2str(kappa_min_acc_fr));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_min_acc_rl").set_value(vec2str(kappa_min_acc_rl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_50percent_lat/kappa_min_acc_rr").set_value(vec2str(kappa_min_acc_rr));

}


TEST_F(Steady_state_test_f1, max_longitudinal_acceleration_several_speeds_75percent_lat)
{
    size_t n = 25;

    double ax_previous = 0.0;
    std::vector<double> ax_min(n), kappa_min_acc_fl(n), kappa_min_acc_fr(n), kappa_min_acc_rl(n), kappa_min_acc_rr(n), psi_min_acc(n), delta_min_acc(n);
    std::vector<double> ax_max(n), kappa_max_acc_fl(n), kappa_max_acc_fr(n), kappa_max_acc_rl(n), kappa_max_acc_rr(n), psi_max_acc(n), delta_max_acc(n);
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 100.0*KMH + 200.0*KMH*i/(n-1);

        Steady_state ss(car);

        auto solution = ss.solve_max_lat_acc(v);
        double ay = solution.ay*0.75;
        auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);
    
        EXPECT_TRUE(solution_max.solved) << "with i = " << i;
        EXPECT_TRUE(solution_min.solved) << "with i = " << i;
        ax_min[i] = solution_min.ax;
        ax_max[i] = solution_max.ax;

        car_sc(solution_max.inputs, solution_max.controls, 0.0);

        kappa_max_acc_fl[i] = car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa();
        kappa_max_acc_fr[i] = car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa();
        kappa_max_acc_rl[i] = car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        kappa_max_acc_rr[i] = car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        psi_max_acc[i] = std::as_const(car_sc).get_road().get_psi()*RAD;
        delta_max_acc[i] = car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD;

        car_sc(solution_min.inputs, solution_min.controls, 0.0);

        kappa_min_acc_fl[i] = car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa();
        kappa_min_acc_fr[i] = car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa();
        kappa_min_acc_rl[i] = car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        kappa_min_acc_rr[i] = car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        psi_min_acc[i] = std::as_const(car_sc).get_road().get_psi()*RAD;
        delta_min_acc[i] = car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD;

        // For now, just test that the longitudinal acceleration decreases
        EXPECT_TRUE(ax_previous > solution_min.ax);
        ax_previous = solution_min.ax;

        if ( is_valgrind ) return;
    }


    auto ax_max_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/maximum_x_acceleration").get_value(std::vector<scalar>());
    auto kappa_max_acc_fl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_max_acc_fl").get_value(std::vector<scalar>());
    auto kappa_max_acc_fr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_max_acc_fr").get_value(std::vector<scalar>());
    auto kappa_max_acc_rl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_max_acc_rl").get_value(std::vector<scalar>());
    auto kappa_max_acc_rr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_max_acc_rr").get_value(std::vector<scalar>());
    auto psi_max_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/psi_max_acc").get_value(std::vector<scalar>());
    auto delta_max_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/delta_max_acc").get_value(std::vector<scalar>());

    auto ax_min_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/minimum_x_acceleration").get_value(std::vector<scalar>());
    auto kappa_min_acc_fl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_min_acc_fl").get_value(std::vector<scalar>());
    auto kappa_min_acc_fr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_min_acc_fr").get_value(std::vector<scalar>());
    auto kappa_min_acc_rl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_min_acc_rl").get_value(std::vector<scalar>());
    auto kappa_min_acc_rr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_min_acc_rr").get_value(std::vector<scalar>());
    auto psi_min_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/psi_min_acc").get_value(std::vector<scalar>());
    auto delta_min_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/delta_min_acc").get_value(std::vector<scalar>());


    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(ax_max[i]           , ax_max_ref[i]           , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_fl[i] , kappa_max_acc_fl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_fr[i] , kappa_max_acc_fr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_rl[i] , kappa_max_acc_rl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_rr[i] , kappa_max_acc_rr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(psi_max_acc[i]      , psi_max_acc_ref[i]      , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(delta_max_acc[i]    , delta_max_acc_ref[i]    , 2.0e-4) << ", with i = " << i;

        EXPECT_NEAR(ax_min[i]           , ax_min_ref[i]           , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_fl[i] , kappa_min_acc_fl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_fr[i] , kappa_min_acc_fr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_rl[i] , kappa_min_acc_rl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_rr[i] , kappa_min_acc_rr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(psi_min_acc[i]      , psi_min_acc_ref[i]      , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(delta_min_acc[i]    , delta_min_acc_ref[i]    , 2.0e-4) << ", with i = " << i;
    }

    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/maximum_x_acceleration").set_value(vec2str(ax_max));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/psi_max_acc").set_value(vec2str(psi_max_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/delta_max_acc").set_value(vec2str(delta_max_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_max_acc_fl").set_value(vec2str(kappa_max_acc_fl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_max_acc_fr").set_value(vec2str(kappa_max_acc_fr));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_max_acc_rl").set_value(vec2str(kappa_max_acc_rl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_max_acc_rr").set_value(vec2str(kappa_max_acc_rr));

    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/minimum_x_acceleration").set_value(vec2str(ax_min));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/psi_min_acc").set_value(vec2str(psi_min_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/delta_min_acc").set_value(vec2str(delta_min_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_min_acc_fl").set_value(vec2str(kappa_min_acc_fl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_min_acc_fr").set_value(vec2str(kappa_min_acc_fr));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_min_acc_rl").set_value(vec2str(kappa_min_acc_rl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_75percent_lat/kappa_min_acc_rr").set_value(vec2str(kappa_min_acc_rr));
}


TEST_F(Steady_state_test_f1, max_longitudinal_acceleration_several_speeds_95percent_lat)
{
    size_t n = 25;

    double ax_previous = 0.0;
    std::vector<double> ax_min(n), kappa_min_acc_fl(n), kappa_min_acc_fr(n), kappa_min_acc_rl(n), kappa_min_acc_rr(n), psi_min_acc(n), delta_min_acc(n);
    std::vector<double> ax_max(n), kappa_max_acc_fl(n), kappa_max_acc_fr(n), kappa_max_acc_rl(n), kappa_max_acc_rr(n), psi_max_acc(n), delta_max_acc(n);
    for (size_t i = 0; i < n; ++i)
    {
        const scalar v = 100.0*KMH + 200.0*KMH*i/(n-1);

        Steady_state ss(car);

        auto solution = ss.solve_max_lat_acc(v);
        double ay = solution.ay*0.95;
        auto [solution_max, solution_min] = ss.solve_max_lon_acc(v,ay);
    
        EXPECT_TRUE(solution_max.solved) << "with i = " << i;
        EXPECT_TRUE(solution_min.solved) << "with i = " << i;
        ax_min[i] = solution_min.ax;
        ax_max[i] = solution_max.ax;

        car_sc(solution_max.inputs, solution_max.controls, 0.0);

        kappa_max_acc_fl[i] = car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa();
        kappa_max_acc_fr[i] = car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa();
        kappa_max_acc_rl[i] = car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        kappa_max_acc_rr[i] = car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        psi_max_acc[i] = std::as_const(car_sc).get_road().get_psi()*RAD;
        delta_max_acc[i] = car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD;

        car_sc(solution_min.inputs, solution_min.controls, 0.0);

        kappa_min_acc_fl[i] = car_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa();
        kappa_min_acc_fr[i] = car_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa();
        kappa_min_acc_rl[i] = car_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        kappa_min_acc_rr[i] = car_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        psi_min_acc[i] = std::as_const(car_sc).get_road().get_psi()*RAD;
        delta_min_acc[i] = car_sc.get_chassis().get_front_axle().get_steering_angle()*RAD;

        // For now, just test that the longitudinal acceleration decreases
        EXPECT_TRUE(ax_previous > solution_min.ax);
        ax_previous = solution_min.ax;

        if ( is_valgrind ) return;
    }


    auto ax_max_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/maximum_x_acceleration").get_value(std::vector<scalar>());
    auto kappa_max_acc_fl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_max_acc_fl").get_value(std::vector<scalar>());
    auto kappa_max_acc_fr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_max_acc_fr").get_value(std::vector<scalar>());
    auto kappa_max_acc_rl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_max_acc_rl").get_value(std::vector<scalar>());
    auto kappa_max_acc_rr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_max_acc_rr").get_value(std::vector<scalar>());
    auto psi_max_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/psi_max_acc").get_value(std::vector<scalar>());
    auto delta_max_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/delta_max_acc").get_value(std::vector<scalar>());

    auto ax_min_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/minimum_x_acceleration").get_value(std::vector<scalar>());
    auto kappa_min_acc_fl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_min_acc_fl").get_value(std::vector<scalar>());
    auto kappa_min_acc_fr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_min_acc_fr").get_value(std::vector<scalar>());
    auto kappa_min_acc_rl_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_min_acc_rl").get_value(std::vector<scalar>());
    auto kappa_min_acc_rr_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_min_acc_rr").get_value(std::vector<scalar>());
    auto psi_min_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/psi_min_acc").get_value(std::vector<scalar>());
    auto delta_min_acc_ref = references.get_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/delta_min_acc").get_value(std::vector<scalar>());


    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(ax_max[i]           , ax_max_ref[i]           , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_fl[i] , kappa_max_acc_fl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_fr[i] , kappa_max_acc_fr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_rl[i] , kappa_max_acc_rl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_max_acc_rr[i] , kappa_max_acc_rr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(psi_max_acc[i]      , psi_max_acc_ref[i]      , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(delta_max_acc[i]    , delta_max_acc_ref[i]    , 2.0e-4) << ", with i = " << i;

        EXPECT_NEAR(ax_min[i]           , ax_min_ref[i]           , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_fl[i] , kappa_min_acc_fl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_fr[i] , kappa_min_acc_fr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_rl[i] , kappa_min_acc_rl_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(kappa_min_acc_rr[i] , kappa_min_acc_rr_ref[i] , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(psi_min_acc[i]      , psi_min_acc_ref[i]      , 2.0e-4) << ", with i = " << i;
        EXPECT_NEAR(delta_min_acc[i]    , delta_min_acc_ref[i]    , 2.0e-4) << ", with i = " << i;
    }

    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/maximum_x_acceleration").set_value(vec2str(ax_max));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/psi_max_acc").set_value(vec2str(psi_max_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/delta_max_acc").set_value(vec2str(delta_max_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_max_acc_fl").set_value(vec2str(kappa_max_acc_fl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_max_acc_fr").set_value(vec2str(kappa_max_acc_fr));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_max_acc_rl").set_value(vec2str(kappa_max_acc_rl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_max_acc_rr").set_value(vec2str(kappa_max_acc_rr));

    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/minimum_x_acceleration").set_value(vec2str(ax_min));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/psi_min_acc").set_value(vec2str(psi_min_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/delta_min_acc").set_value(vec2str(delta_min_acc));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_min_acc_fl").set_value(vec2str(kappa_min_acc_fl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_min_acc_fr").set_value(vec2str(kappa_min_acc_fr));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_min_acc_rl").set_value(vec2str(kappa_min_acc_rl));
    new_reference.doc.add_element("f1_steady_state_test/max_longitudinal_acceleration_several_speeds_95percent_lat/kappa_min_acc_rr").set_value(vec2str(kappa_min_acc_rr));
}


TEST_F(Steady_state_test_f1, max_lateral_accel_300kmh)
{
    const scalar v = 300.0*KMH;

    Steady_state ss(car);
    auto solution = ss.solve_max_lat_acc(v);

    car_sc(solution.inputs,solution.controls,0.0);
    
    EXPECT_TRUE(solution.solved);
}


