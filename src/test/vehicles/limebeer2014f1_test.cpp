#include "gtest/gtest.h"
#include "src/core/vehicles/limebeer2014f1.h"
#include "src/core/applications/optimal_laptime.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "src/core/applications/steady_state.h"

// Define convenient aliases
using Front_left_tire_type  = limebeer2014f1<scalar>::Front_left_tire_type;
using Front_right_tire_type = limebeer2014f1<scalar>::Front_left_tire_type;
using Rear_left_tire_type   = limebeer2014f1<scalar>::Rear_left_tire_type;
using Rear_right_tire_type  = limebeer2014f1<scalar>::Rear_right_tire_type;

using Front_axle_t          = limebeer2014f1<scalar>::Front_axle_t;
using Rear_axle_t           = limebeer2014f1<scalar>::Rear_axle_t;

using Chassis_t             = limebeer2014f1<scalar>::Chassis_t;

using Road_t                = limebeer2014f1<scalar>::Road_cartesian_t;

class limebeer2014f1_test : public testing::Test
{
 protected:
    Xml_document database   = {"./database/limebeer-2014-f1.xml", true};
};


// Check expected indexes for state variables
static_assert(Front_axle_t::IKAPPA_LEFT  == 0);
static_assert(Front_axle_t::IKAPPA_RIGHT == 1);
static_assert(Rear_axle_t::IKAPPA_LEFT   == 2);
static_assert(Rear_axle_t::IKAPPA_RIGHT  == 3);
static_assert(Chassis_t::IU              == 4);
static_assert(Chassis_t::IV              == 5);
static_assert(Chassis_t::IOMEGA          == 6);
static_assert(Road_t::IX                 == 7);
static_assert(Road_t::IY                 == 8);
static_assert(Road_t::IPSI               == 9);
static_assert(limebeer2014f1<scalar>::cartesian::NSTATE == 10);

// Check expected indexes for control variables
static_assert(Front_axle_t::ISTEERING == 0);
static_assert(Chassis_t::ITHROTTLE    == 1);
static_assert(limebeer2014f1<scalar>::cartesian::NCONTROL == 2);

// Check expected indexes for state derivative variables
static_assert(Front_axle_t::IIDKAPPA_LEFT  == 0);
static_assert(Front_axle_t::IIDKAPPA_RIGHT == 1);
static_assert(Rear_axle_t::IIDKAPPA_LEFT   == 2);
static_assert(Rear_axle_t::IIDKAPPA_RIGHT  == 3);
static_assert(Chassis_t::IIDU              == 4);
static_assert(Chassis_t::IIDV              == 5);
static_assert(Chassis_t::IIDOMEGA          == 6);
static_assert(Road_t::IIDX                 == 7);
static_assert(Road_t::IIDY                 == 8);
static_assert(Road_t::IIDPSI               == 9);


static_assert(limebeer2014f1<scalar>::cartesian::NALGEBRAIC == 4);


TEST_F(limebeer2014f1_test, indexes)
{
    EXPECT_EQ(Front_axle_t::IKAPPA_LEFT  , 0);
    EXPECT_EQ(Front_axle_t::IKAPPA_RIGHT , 1);
    EXPECT_EQ(Rear_axle_t::IKAPPA_LEFT   , 2);
    EXPECT_EQ(Rear_axle_t::IKAPPA_RIGHT  , 3);
    EXPECT_EQ(Chassis_t::IU              , 4);
    EXPECT_EQ(Chassis_t::IV              , 5);
    EXPECT_EQ(Chassis_t::IOMEGA          , 6);
    EXPECT_EQ(Road_t::IX                 , 7);
    EXPECT_EQ(Road_t::IY                 , 8);
    EXPECT_EQ(Road_t::IPSI               , 9);

    EXPECT_EQ(limebeer2014f1<scalar>::cartesian::NSTATE, 10);

    EXPECT_EQ(Front_axle_t::ISTEERING, 0);
    EXPECT_EQ(Chassis_t::ITHROTTLE, 1);
    EXPECT_EQ(limebeer2014f1<scalar>::cartesian::NCONTROL, 2);

    EXPECT_EQ(Front_axle_t::IIDKAPPA_LEFT  , 0);
    EXPECT_EQ(Front_axle_t::IIDKAPPA_RIGHT , 1);
    EXPECT_EQ(Rear_axle_t::IIDKAPPA_LEFT   , 2);
    EXPECT_EQ(Rear_axle_t::IIDKAPPA_RIGHT  , 3);
    EXPECT_EQ(Chassis_t::IIDU              , 4);
    EXPECT_EQ(Chassis_t::IIDV              , 5);
    EXPECT_EQ(Chassis_t::IIDOMEGA          , 6);
    EXPECT_EQ(Road_t::IIDX                 , 7);
    EXPECT_EQ(Road_t::IIDY                 , 8);
    EXPECT_EQ(Road_t::IIDPSI               , 9);
}

TEST_F(limebeer2014f1_test, variable_names)
{
    limebeer2014f1<double>::curvilinear_p car_sc(database);

    auto [s_names, q_names, u_names] = car_sc.get_state_and_control_names();
}


TEST_F(limebeer2014f1_test, jacobian_autodiff)
{
    limebeer2014f1<CppAD::AD<double>>::cartesian car_ad(database);
    limebeer2014f1<double>::cartesian            car_sc(database);

    // Fill the inputs to the operator() of the vehicle. These inputs belong to a 0g trim at 300km/h
    std::array<double,10> q0 = {0.0, 0.0, 0.0111971, 0.0111971, 83.3333, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double,4> qa0 = {-0.874103, -0.874103, -1.07386, -1.07386};
    std::array<double,2> u0 = {0.0, 0.644468};

    // Call operator()
    auto solution = car_ad.equations(q0,qa0,u0,0.0);

    // Compute the numerical Jacobian
    std::array<std::array<double,10>,10+4+2> numerical_jacobian;
    std::array<std::array<double,4>,10+4+2> numerical_jacobian_dqa;

    // derivatives w.r.t q
    for (size_t i = 0; i < 10; ++i)
    {
        // Freeze u0 and add a perturbation on q0 
        const double eps = std::max(1.0,fabs(q0[i]))*1.0e-8;
        q0[i] += eps;

        auto [dqdt_eps, dqa_eps] = car_sc(q0,qa0,u0,0.0);

        q0[i] -= 2*eps;

        auto [dqdt_meps, dqa_meps] = car_sc(q0,qa0,u0,0.0);

        numerical_jacobian[i] = (dqdt_eps - dqdt_meps)*(0.5/eps);
        numerical_jacobian_dqa[i] = (dqa_eps - dqa_meps)*(0.5/eps);

        q0[i] += eps;
    }

    // derivatives w.r.t qa
    for (size_t i = 0; i < 4; ++i)
    {
        // Freeze u0 and add a perturbation on q0 
        const double eps = std::max(1.0,fabs(q0[i]))*1.0e-8;
        qa0[i] += eps;

        auto [dqdt_eps, dqa_eps] = car_sc(q0,qa0,u0,0.0);

        qa0[i] -= 2*eps;

        auto [dqdt_meps, dqa_meps] = car_sc(q0,qa0,u0,0.0);

        numerical_jacobian[i+10] = (dqdt_eps - dqdt_meps)*(0.5/eps);
        numerical_jacobian_dqa[i+10] = (dqa_eps - dqa_meps)*(0.5/eps);

        qa0[i] += eps;
    }

    // derivatives w.r.t u
    for (size_t i = 0; i < 2; ++i)
    {
        // Freeze u0 and add a perturbation on q0 
        const double eps = std::max(1.0,fabs(u0[i]))*1.0e-8;
        u0[i] += eps;

        auto [dqdt_eps,dqa_eps] = car_sc(q0,qa0,u0,0.0);

        u0[i] -= 2*eps;

        auto [dqdt_meps,dqa_meps] = car_sc(q0,qa0,u0,0.0);

        numerical_jacobian[i+14] = (dqdt_eps - dqdt_meps)*(0.5/eps);
        numerical_jacobian_dqa[i+14] = (dqa_eps - dqa_meps)*(0.5/eps);

        u0[i] += eps;
    }


    for (size_t i = 0; i < 10+4+2; ++i)
        for (size_t j = 0; j < 10; ++j)
        {
            EXPECT_NEAR(numerical_jacobian[i][j], solution.jac_dqdt[j][i], 2.0e-6*std::max(fabs(solution.jac_dqdt[j][i]),1.0)) << "with i = " << i << " and j = " << j ;
        }

    for (size_t i = 0; i < 10+4+2; ++i)
        for (size_t j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(numerical_jacobian_dqa[i][j], solution.jac_dqa[j][i], 2.0e-6*std::max(fabs(solution.jac_dqa[j][i]),1.0)) << "with i = " << i << " and j = " << j ;
        }
}

TEST_F(limebeer2014f1_test, jacobian_autodiff_random)
{
    limebeer2014f1<CppAD::AD<double>>::cartesian car_ad(database);
    limebeer2014f1<double>::cartesian            car_sc(database);

    // Fill the inputs to the operator() of the vehicle. These are made up inputs
    std::array<double,10> q0 = {-0.05, -0.08, 0.0200000, 0.0800000, 50.0000, -5.0, 0.4, 0.0, 0.0, 5.0*DEG};
    std::array<double,4> qa0 = {-0.674103, -0.474103, -0.80386, -0.70386};
    std::array<double,2> u0 = {-2.0*DEG, 0.100000};

    // Call operator()
    auto solution = car_ad.equations(q0,qa0,u0,0.0);

    // Compute the numerical Jacobian
    std::array<std::array<double,10>,10+4+2> numerical_jacobian;
    std::array<std::array<double,4>,10+4+2> numerical_jacobian_dqa;

    // derivatives w.r.t q
    for (size_t i = 0; i < 10; ++i)
    {
        // Freeze u0 and add a perturbation on q0 
        const double eps = std::max(1.0,fabs(q0[i]))*1.0e-8;
        q0[i] += eps;

        auto [dqdt_eps, dqa_eps] = car_sc(q0,qa0,u0,0.0);

        q0[i] -= 2*eps;

        auto [dqdt_meps, dqa_meps] = car_sc(q0,qa0,u0,0.0);

        numerical_jacobian[i] = (dqdt_eps - dqdt_meps)*(0.5/eps);
        numerical_jacobian_dqa[i] = (dqa_eps - dqa_meps)*(0.5/eps);

        q0[i] += eps;
    }

    // derivatives w.r.t qa
    for (size_t i = 0; i < 4; ++i)
    {
        // Freeze u0 and add a perturbation on q0 
        const double eps = std::max(1.0,fabs(q0[i]))*1.0e-8;
        qa0[i] += eps;

        auto [dqdt_eps, dqa_eps] = car_sc(q0,qa0,u0,0.0);

        qa0[i] -= 2*eps;

        auto [dqdt_meps, dqa_meps] = car_sc(q0,qa0,u0,0.0);

        numerical_jacobian[i+10] = (dqdt_eps - dqdt_meps)*(0.5/eps);
        numerical_jacobian_dqa[i+10] = (dqa_eps - dqa_meps)*(0.5/eps);

        qa0[i] += eps;
    }

    // derivatives w.r.t u
    for (size_t i = 0; i < 2; ++i)
    {
        // Freeze u0 and add a perturbation on q0 
        const double eps = std::max(1.0,fabs(u0[i]))*1.0e-8;
        u0[i] += eps;

        auto [dqdt_eps,dqa_eps] = car_sc(q0,qa0,u0,0.0);

        u0[i] -= 2*eps;

        auto [dqdt_meps,dqa_meps] = car_sc(q0,qa0,u0,0.0);

        numerical_jacobian[i+14] = (dqdt_eps - dqdt_meps)*(0.5/eps);
        numerical_jacobian_dqa[i+14] = (dqa_eps - dqa_meps)*(0.5/eps);

        u0[i] += eps;
    }


    for (size_t i = 0; i < 10+4+2; ++i)
        for (size_t j = 0; j < 10; ++j)
        {
            EXPECT_NEAR(numerical_jacobian[i][j], solution.jac_dqdt[j][i], 2.0e-6*std::max(fabs(solution.jac_dqdt[j][i]),1.0)) << "with i = " << i << " and j = " << j ;
        }

    for (size_t i = 0; i < 10+4+2; ++i)
        for (size_t j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(numerical_jacobian_dqa[i][j], solution.jac_dqa[j][i], 2.0e-6*std::max(fabs(solution.jac_dqa[j][i]),1.0)) << "with i = " << i << " and j = " << j ;
        }
}

TEST_F(limebeer2014f1_test, set_parameter)
{
    Xml_document dummy_database("./data/dummy-f1.xml",true);

    Xml_document catalunya_xml("./database/catalunya_discrete.xml",true);
    Track_by_polynomial catalunya(catalunya_xml);
    

    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial> car(dummy_database, road);
    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial> car_correct(database, road);

    car.set_parameter("vehicle/front-axle/track", 1.46);
    car.set_parameter("vehicle/front-axle/inertia", 1.0);
    car.set_parameter("vehicle/front-axle/smooth_throttle_coeff", 1.0e-5);
    car.set_parameter("vehicle/front-axle/brakes/max_torque", 5000.0);

    car.set_parameter("vehicle/rear-axle/track", 1.46);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);
    car.set_parameter("vehicle/rear-axle/smooth_throttle_coeff", 1.0e-5);
    car.set_parameter("vehicle/rear-axle/differential_stiffness", 10.47);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque", 5000.0);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power", 735.499);

    car.set_parameter("vehicle/chassis/mass", 660.0);
    car.set_parameter("vehicle/chassis/inertia", sMatrix3x3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 450.0));
    car.set_parameter("vehicle/chassis/aerodynamics/rho", 1.2);
    car.set_parameter("vehicle/chassis/aerodynamics/area", 1.5);
    car.set_parameter("vehicle/chassis/aerodynamics/cd", 0.9);
    car.set_parameter("vehicle/chassis/aerodynamics/cl", 3.0);
    car.set_parameter("vehicle/chassis/com", sVector3d(0.0, 0.0, -0.3));
    car.set_parameter("vehicle/chassis/front_axle", sVector3d(1.8, 0.0, -0.33));
    car.set_parameter("vehicle/chassis/rear_axle", sVector3d(-1.6, 0.0, -0.33));
    car.set_parameter("vehicle/chassis/pressure_center", sVector3d(-0.1, 0.0, -0.3));
    car.set_parameter("vehicle/chassis/brake_bias", 0.6);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", 0.5);
    car.set_parameter("vehicle/chassis/Fz_max_ref2", 1.0);
    
    car.set_parameter("vehicle/front-tire/radius",0.330); 
    car.set_parameter("vehicle/front-tire/radial-stiffness",0.0);
    car.set_parameter("vehicle/front-tire/radial-damping",0.0);
    car.set_parameter("vehicle/front-tire/Fz-max-ref2", 1.0 );
    car.set_parameter("vehicle/front-tire/reference-load-1", 2000.0 ); 
    car.set_parameter("vehicle/front-tire/reference-load-2", 6000.0 ); 
    car.set_parameter("vehicle/front-tire/mu-x-max-1", 1.75 );
    car.set_parameter("vehicle/front-tire/mu-x-max-2", 1.40 );
    car.set_parameter("vehicle/front-tire/kappa-max-1", 0.11 );
    car.set_parameter("vehicle/front-tire/kappa-max-2", 0.10 );
    car.set_parameter("vehicle/front-tire/mu-y-max-1", 1.80 );
    car.set_parameter("vehicle/front-tire/mu-y-max-2", 1.45 );
    car.set_parameter("vehicle/front-tire/lambda-max-1", 9.0 );
    car.set_parameter("vehicle/front-tire/lambda-max-2", 8.0 );
    car.set_parameter("vehicle/front-tire/Qx", 1.9 );
    car.set_parameter("vehicle/front-tire/Qy", 1.9 );

    car.set_parameter("vehicle/rear-tire/radius",0.330); 
    car.set_parameter("vehicle/rear-tire/radial-stiffness",0.0);
    car.set_parameter("vehicle/rear-tire/radial-damping",0.0);
    car.set_parameter("vehicle/rear-tire/Fz-max-ref2", 1.0 );
    car.set_parameter("vehicle/rear-tire/reference-load-1", 2000.0 ); 
    car.set_parameter("vehicle/rear-tire/reference-load-2", 6000.0 ); 
    car.set_parameter("vehicle/rear-tire/mu-x-max-1", 1.75 );
    car.set_parameter("vehicle/rear-tire/mu-x-max-2", 1.40 );
    car.set_parameter("vehicle/rear-tire/kappa-max-1", 0.11 );
    car.set_parameter("vehicle/rear-tire/kappa-max-2", 0.10 );
    car.set_parameter("vehicle/rear-tire/mu-y-max-1", 1.80 );
    car.set_parameter("vehicle/rear-tire/mu-y-max-2", 1.45 );
    car.set_parameter("vehicle/rear-tire/lambda-max-1", 9.0);
    car.set_parameter("vehicle/rear-tire/lambda-max-2", 8.0);
    car.set_parameter("vehicle/rear-tire/Qx", 1.9 );
    car.set_parameter("vehicle/rear-tire/Qy", 1.9 );

    std::array<double,10> q0;
    std::array<double,4> qa0;
    std::array<double,2> u0;
    
    constexpr const size_t n = 500;

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_discrete.xml", true);

    auto arclength_saved = opt_saved.get_element("optimal_laptime/arclength").get_value(std::vector<scalar>());
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    auto u_saved        = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    auto v_saved        = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    auto omega_saved    = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    auto time_saved     = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    auto n_saved        = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    auto alpha_saved    = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    auto delta_saved    = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    auto throttle_saved = opt_saved.get_element("optimal_laptime/throttle").get_value(std::vector<scalar>());
    auto Fz_fl_saved    = opt_saved.get_element("optimal_laptime/Fz_fl").get_value(std::vector<scalar>());
    auto Fz_fr_saved    = opt_saved.get_element("optimal_laptime/Fz_fr").get_value(std::vector<scalar>());
    auto Fz_rl_saved    = opt_saved.get_element("optimal_laptime/Fz_rl").get_value(std::vector<scalar>());
    auto Fz_rr_saved    = opt_saved.get_element("optimal_laptime/Fz_rr").get_value(std::vector<scalar>());

    for (size_t i = 0; i < n; ++i)
    {
        q0 = {kappa_fl_saved[i], kappa_fr_saved[i], kappa_rl_saved[i], kappa_rr_saved[i], u_saved[i], v_saved[i], omega_saved[i],
              time_saved[i], n_saved[i], alpha_saved[i]};

        u0 = {delta_saved[i], throttle_saved[i]};
        qa0 = {Fz_fl_saved[i], Fz_fr_saved[i], Fz_rl_saved[i], Fz_rr_saved[i]};

        auto [dqdt, dqa] = car(q0,qa0,u0,arclength_saved[i]);
        auto [dqdt_c, dqa_c] = car_correct(q0,qa0,u0,arclength_saved[i]);

        for (size_t j = 0; j < dqdt.size(); ++j)
            EXPECT_NEAR(dqdt[j], dqdt_c[j], 2.0e-15);

        for (size_t j = 0; j < dqa.size(); ++j)
            EXPECT_NEAR(dqa[j], dqa_c[j], 2.0e-15);
    }
}
