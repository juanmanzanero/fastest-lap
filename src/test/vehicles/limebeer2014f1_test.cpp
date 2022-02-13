#include "gtest/gtest.h"
#include "src/core/vehicles/limebeer2014f1.h"

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

    std::cout << q_names << std::endl;
    std::cout << u_names << std::endl;
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
