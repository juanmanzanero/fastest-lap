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
