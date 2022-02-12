#include "src/core/applications/circuit_preprocessor.h"
#include "gtest/gtest.h"

extern bool is_valgrind;

TEST(Circuit_preprocessor_test, museo_closed)
{
    Xml_document coord_left_kml("./database/google_earth/Museo_short_left.kml", true);
    Xml_document coord_right_kml("./database/google_earth/Museo_short_right.kml", true);

    Circuit_preprocessor::Options options;

    options.eps_k *= 0.001;
    options.eps_n *= 0.001;
    options.eps_c *= 0.001;
    options.eps_d *= 0.001;

    options.maximum_kappa = 1.0;
    options.maximum_dkappa = 1.0;
    
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, 100, options);

    circuit.xml()->save("museo_short_discrete.xml");

    Xml_document solution_saved("./data/museo_short_discrete.xml", true);

    const std::vector<scalar> s     = solution_saved.get_element("circuit/data/arclength").get_value(std::vector<scalar>());
    const std::vector<scalar> x     = solution_saved.get_element("circuit/data/centerline/x").get_value(std::vector<scalar>());
    const std::vector<scalar> y     = solution_saved.get_element("circuit/data/centerline/y").get_value(std::vector<scalar>());
    const std::vector<scalar> theta = solution_saved.get_element("circuit/data/theta").get_value(std::vector<scalar>());
    const std::vector<scalar> kappa = solution_saved.get_element("circuit/data/kappa").get_value(std::vector<scalar>());
    const std::vector<scalar> nl    = solution_saved.get_element("circuit/data/nl").get_value(std::vector<scalar>());
    const std::vector<scalar> nr    = solution_saved.get_element("circuit/data/nr").get_value(std::vector<scalar>());
    const std::vector<scalar> dkappa = solution_saved.get_element("circuit/data/dkappa").get_value(std::vector<scalar>());
    const std::vector<scalar> dnl    = solution_saved.get_element("circuit/data/dnl").get_value(std::vector<scalar>());
    const std::vector<scalar> dnr    = solution_saved.get_element("circuit/data/dnr").get_value(std::vector<scalar>());

    EXPECT_EQ(circuit.n_points,100);
    EXPECT_EQ(circuit.r_centerline.size(),100);
    EXPECT_EQ(circuit.theta.size(),100);
    EXPECT_EQ(circuit.kappa.size(),100);
    EXPECT_EQ(circuit.nl.size(),100);
    EXPECT_EQ(circuit.nr.size(),100);
    EXPECT_EQ(circuit.dkappa.size(),100);
    EXPECT_EQ(circuit.dnl.size(),100);
    EXPECT_EQ(circuit.dnr.size(),100);

    // compare centerline
    for (size_t i = 0; i < circuit.n_points; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i].x(), x[i]     , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.r_centerline[i].y(), y[i]     , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.theta[i]           , theta[i] , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.kappa[i]           , kappa[i] , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.nl[i]              , nl[i]    , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.nr[i]              , nr[i]    , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.dkappa[i]          , dkappa[i], 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.dnl[i]             , dnl[i]   , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.dnr[i]             , dnr[i]   , 1.0e-8) << " with i = " << i;
    }

    // Check that the reference coordinates are recovered
    std::vector<scalar> coord_left  = coord_left_kml.get_element("kml/Document/Placemark/LineString/coordinates").get_value(std::vector<scalar>());
    std::vector<scalar> coord_right = coord_right_kml.get_element("kml/Document/Placemark/LineString/coordinates").get_value(std::vector<scalar>());

    EXPECT_EQ(circuit.r_left_measured.size()*3, coord_left.size());
    EXPECT_EQ(circuit.r_right_measured.size()*3, coord_right.size());

    scalar theta0 = coord_right[0];
    scalar phi0 = coord_right[1];

    for (size_t i = 0; i < circuit.r_left_measured.size(); ++i)
    {
        scalar lon = coord_left[3*i];
        scalar lat = coord_left[3*i+1];
    
        EXPECT_DOUBLE_EQ(lon, circuit.r_left_measured[i].x()/(circuit.R_earth*cos(phi0*DEG))*RAD + theta0);
        EXPECT_DOUBLE_EQ(lat, circuit.r_left_measured[i].y()/(circuit.R_earth)*RAD + phi0);
    }
}


TEST(Circuit_preprocessor_test, catalunya_500)
{
    #ifndef NDEBUG
        GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document coord_left_kml("./database/google_earth/Catalunya_left.kml", true);
    Xml_document coord_right_kml("./database/google_earth/Catalunya_right.kml", true);
    
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, 500, {});

    Xml_document solution_saved("./database/catalunya_discrete.xml", true);

    const std::vector<scalar> s     = solution_saved.get_element("circuit/data/arclength").get_value(std::vector<scalar>());
    const std::vector<scalar> x     = solution_saved.get_element("circuit/data/centerline/x").get_value(std::vector<scalar>());
    const std::vector<scalar> y     = solution_saved.get_element("circuit/data/centerline/y").get_value(std::vector<scalar>());
    const std::vector<scalar> theta = solution_saved.get_element("circuit/data/theta").get_value(std::vector<scalar>());
    const std::vector<scalar> kappa = solution_saved.get_element("circuit/data/kappa").get_value(std::vector<scalar>());
    const std::vector<scalar> nl    = solution_saved.get_element("circuit/data/nl").get_value(std::vector<scalar>());
    const std::vector<scalar> nr    = solution_saved.get_element("circuit/data/nr").get_value(std::vector<scalar>());
    const std::vector<scalar> dkappa = solution_saved.get_element("circuit/data/dkappa").get_value(std::vector<scalar>());
    const std::vector<scalar> dnl    = solution_saved.get_element("circuit/data/dnl").get_value(std::vector<scalar>());
    const std::vector<scalar> dnr    = solution_saved.get_element("circuit/data/dnr").get_value(std::vector<scalar>());

    EXPECT_EQ(circuit.n_points,500);
    EXPECT_EQ(circuit.r_centerline.size(),500);
    EXPECT_EQ(circuit.theta.size(),500);
    EXPECT_EQ(circuit.kappa.size(),500);
    EXPECT_EQ(circuit.nl.size(),500);
    EXPECT_EQ(circuit.nr.size(),500);
    EXPECT_EQ(circuit.dkappa.size(),500);
    EXPECT_EQ(circuit.dnl.size(),500);
    EXPECT_EQ(circuit.dnr.size(),500);

    // compare centerline
    for (size_t i = 0; i < circuit.n_points; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i].x(), x[i]     , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.r_centerline[i].y(), y[i]     , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.theta[i]           , theta[i] , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.kappa[i]           , kappa[i] , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.nl[i]              , nl[i]    , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.nr[i]              , nr[i]    , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.dkappa[i]          , dkappa[i], 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.dnl[i]             , dnl[i]   , 1.0e-8) << " with i = " << i;
        EXPECT_NEAR(circuit.dnr[i]             , dnr[i]   , 1.0e-8) << " with i = " << i;
    }

    // Check that the reference coordinates are recovered
    std::vector<scalar> coord_left  = coord_left_kml.get_element("kml/Document/Placemark/LineString/coordinates").get_value(std::vector<scalar>());
    std::vector<scalar> coord_right = coord_right_kml.get_element("kml/Document/Placemark/LineString/coordinates").get_value(std::vector<scalar>());

    EXPECT_EQ(circuit.r_left_measured.size()*3, coord_left.size());
    EXPECT_EQ(circuit.r_right_measured.size()*3, coord_right.size());

    scalar theta0 = coord_right[0];
    scalar phi0 = coord_right[1];

    for (size_t i = 0; i < circuit.r_left_measured.size(); ++i)
    {
        scalar lon = coord_left[3*i];
        scalar lat = coord_left[3*i+1];
    
        EXPECT_DOUBLE_EQ(lon, circuit.r_left_measured[i].x()/(circuit.R_earth*cos(phi0*DEG))*RAD + theta0);
        EXPECT_DOUBLE_EQ(lat, circuit.r_left_measured[i].y()/(circuit.R_earth)*RAD + phi0);
    }
}
