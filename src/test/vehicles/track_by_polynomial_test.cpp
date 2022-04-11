#include "gtest/gtest.h"
#include "src/core/vehicles/track_by_polynomial.h"

TEST(Track_by_polynomial_test, evaluation_at_nodes)
{
    Xml_document catalunya = {"./database/tracks/catalunya/catalunya_discrete.xml", true};
     
    const auto arclength = catalunya.get_element("circuit/data/arclength").get_value(std::vector<scalar>());

    const Circuit_preprocessor circuit(catalunya);
    
    Track_by_polynomial track(circuit);

    EXPECT_NEAR(track.get_total_length(),circuit.track_length, 1.0e-16);
    scalar ds = track.get_total_length()/circuit.n_elements;
    for (size_t i = 0; i < circuit.n_points; ++i)
    {
        EXPECT_NEAR(arclength[i],i*ds,1.0e-14*ds*circuit.n_points);
        auto [r,dr,d2r] = track(arclength[i]);
        EXPECT_NEAR(r.x(), circuit.r_centerline[i].x(), 2.0e-13);
        EXPECT_NEAR(r.y(), -circuit.r_centerline[i].y(), 2.0e-13);

        EXPECT_NEAR(dr.x(), cos(-circuit.theta[i]), 1.0e-15);
        EXPECT_NEAR(dr.y(), sin(-circuit.theta[i]), 1.0e-15);

        EXPECT_NEAR(d2r.x(), circuit.kappa[i]*sin(-circuit.theta[i]), 1.0e-15);
        EXPECT_NEAR(d2r.y(), -circuit.kappa[i]*cos(-circuit.theta[i]), 1.0e-15);
    }
}
