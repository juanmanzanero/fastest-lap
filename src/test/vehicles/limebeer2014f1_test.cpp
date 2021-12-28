#include "gtest/gtest.h"
#include "src/core/vehicles/limebeer2014f1.h"

// Define convenient aliases
using Front_left_tire_type  = limebeer2014f1<scalar>::Front_left_tire_type;
using Front_right_tire_type = limebeer2014f1<scalar>::Front_left_tire_type;
using Rear_left_tire_type   = limebeer2014f1<scalar>::Rear_left_tire_type;
using Rear_right_tire_type  = limebeer2014f1<scalar>::Rear_right_tire_type;

using Front_axle_t          = limebeer2014f1<scalar>::Front_axle_t;
using Rear_axle_t           = limebeer2014f1<scalar>::Rear_axle_t;

class limebeer2014f1_test : public testing::Test
{
};


// Check expected indexes for state variables
static_assert(Front_axle_t::IOMEGA_LEFT  == 0);
static_assert(Front_axle_t::IOMEGA_RIGHT == 1);
static_assert(Rear_axle_t::IOMEGA_LEFT   == 2);
static_assert(Rear_axle_t::IOMEGA_RIGHT  == 3);

// Check expected indexes for control variables
static_assert(Front_axle_t::ISTEERING == 0);

// Check expected indexes for state derivative variables
static_assert(Front_axle_t::IIDOMEGA_LEFT  == 0);
static_assert(Front_axle_t::IIDOMEGA_RIGHT == 1);
static_assert(Rear_axle_t::IIDOMEGA_LEFT   == 2);
static_assert(Rear_axle_t::IIDOMEGA_RIGHT  == 3);

TEST_F(limebeer2014f1_test, indexes)
{
    EXPECT_EQ(Front_axle_t::IOMEGA_LEFT  , 0);
    EXPECT_EQ(Front_axle_t::IOMEGA_RIGHT , 1);
    EXPECT_EQ(Rear_axle_t::IOMEGA_LEFT   , 2);
    EXPECT_EQ(Rear_axle_t::IOMEGA_RIGHT  , 3);

    EXPECT_EQ(Front_axle_t::ISTEERING, 0);

    EXPECT_EQ(Front_axle_t::IIDOMEGA_LEFT  , 0);
    EXPECT_EQ(Front_axle_t::IIDOMEGA_RIGHT , 1);
    EXPECT_EQ(Rear_axle_t::IIDOMEGA_LEFT   , 2);
    EXPECT_EQ(Rear_axle_t::IIDOMEGA_RIGHT  , 3);
}
