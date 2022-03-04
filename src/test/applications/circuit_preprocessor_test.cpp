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
    
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, options, 100);

    // Call the xml() method, just to confirm there are no errors/valgrind issues
    circuit.xml();

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


TEST(Circuit_preprocessor_test, catalunya_chicane)
{
    Xml_document coord_left_kml("./database/google_earth/Catalunya_left.kml", true);
    Xml_document coord_right_kml("./database/google_earth/Catalunya_right.kml", true);

    Circuit_preprocessor::Coordinates start = {2.261, 41.57455};
    Circuit_preprocessor::Coordinates finish = {2.26325, 41.57385};
    
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, {}, start, finish, 20);

    circuit.xml();

    Xml_document solution_saved("./data/catalunya_chicane_discrete.xml", true);

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

    EXPECT_EQ(circuit.n_points,21);
    EXPECT_EQ(circuit.r_centerline.size(),21);
    EXPECT_EQ(circuit.theta.size(),21);
    EXPECT_EQ(circuit.kappa.size(),21);
    EXPECT_EQ(circuit.nl.size(),21);
    EXPECT_EQ(circuit.nr.size(),21);
    EXPECT_EQ(circuit.dkappa.size(),21);
    EXPECT_EQ(circuit.dnl.size(),21);
    EXPECT_EQ(circuit.dnr.size(),21);

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
}


TEST(Circuit_preprocessor_test, catalunya_500)
{
    #ifndef NDEBUG
          GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document coord_left_kml("./database/google_earth/Catalunya_left.kml", true);
    Xml_document coord_right_kml("./database/google_earth/Catalunya_right.kml", true);
    
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, {}, 500);

    circuit.xml();

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


TEST(Circuit_preprocessor_test, catalunya_adapted_by_coords)
{
    #ifndef NDEBUG
        GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document coord_left_kml("./database/google_earth/Catalunya_left.kml", true);
    Xml_document coord_right_kml("./database/google_earth/Catalunya_right.kml", true);

    std::vector<std::pair<Circuit_preprocessor::Coordinates,scalar>> ds_breakpoints = 
    { {{0.0,              0.0              }, 20.0},
      {{2.257193719176818,41.56522599658208}, 8.0},
      {{2.254933238159729,41.56471245384411}, 10.0},
      {{2.253853158059371,41.56800644695232}, 8.0},
      {{2.2547819299335,  41.56663345239372}, 10.0},
      {{2.261124403684718,41.57244119725051}, 5.0},
      {{2.260173022232699,41.57444062293845}, 3.0},
      {{2.263173649148178,41.57388038009333}, 10.0} 
    };

    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, {}, ds_breakpoints);

    circuit.xml();

    Xml_document solution_saved("./data/catalunya_adapted.xml", true);

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

    EXPECT_EQ(circuit.n_points,555);
    EXPECT_EQ(circuit.r_centerline.size(),555);
    EXPECT_EQ(circuit.theta.size(),555);
    EXPECT_EQ(circuit.kappa.size(),555);
    EXPECT_EQ(circuit.nl.size(),555);
    EXPECT_EQ(circuit.nr.size(),555);
    EXPECT_EQ(circuit.dkappa.size(),555);
    EXPECT_EQ(circuit.dnl.size(),555);
    EXPECT_EQ(circuit.dnr.size(),555);

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

TEST(Circuit_preprocessor_test, catalunya_adapted_by_ds_distribution)
{
    #ifndef NDEBUG
        GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document coord_left_kml("./database/google_earth/Catalunya_left.kml", true);
    Xml_document coord_right_kml("./database/google_earth/Catalunya_right.kml", true);

    std::vector<scalar> s_distr = {0, 898.9100, 928.8740, 953.8440, 974.6520, 
                                   991.9920, 1006.440, 1018.480, 1150.320, 
                                   1162.310, 1176.690, 1805.930, 1820.910, 
                                   1833.400, 2133.030, 2145.020, 2159.400, 
                                   3552.710, 3567.690, 3580.180, 3590.580, 
                                   3599.250, 4123.620, 4131.110, 4137.350, 
                                   4142.550, 4425.710, 4430.200, 4435.600, 
                                   4442.070, 4449.840, 4459.160, 4470.340, 
                                   4483.760, 4543.690, 4558.670};

    std::vector<scalar> ds_distr = {29.96370, 29.96370, 24.96970, 20.80810, 17.34010, 14.45010,     
                                    12.04170, 11.98550, 11.98550, 14.38260, 14.98180, 14.98180, 
                                    12.48490, 11.98550, 11.98550, 14.38260, 14.98180, 14.98180, 
                                    12.48490, 10.40410, 8.670050, 7.490920, 7.490920, 6.242430, 
                                    5.202030, 4.494550, 4.494550, 5.393460, 6.472150, 7.766590, 
                                    9.319900, 11.18390, 13.42070, 14.98180, 14.98180, 12.60130};

    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, {}, s_distr, ds_distr);

    circuit.xml();

    Xml_document solution_saved("./data/catalunya_ds_distribution.xml", true);

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

    EXPECT_EQ(circuit.n_points,372);
    EXPECT_EQ(circuit.r_centerline.size(),372);
    EXPECT_EQ(circuit.theta.size(),372);
    EXPECT_EQ(circuit.kappa.size(),372);
    EXPECT_EQ(circuit.nl.size(),372);
    EXPECT_EQ(circuit.nr.size(),372);
    EXPECT_EQ(circuit.dkappa.size(),372);
    EXPECT_EQ(circuit.dnl.size(),372);
    EXPECT_EQ(circuit.dnr.size(),372);

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


TEST(Circuit_preprocessor_test, vendrell)
{
    #ifndef NDEBUG
          GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document coord_left_kml("./database/google_earth/vendrell_left.kml", true);
    Xml_document coord_right_kml("./database/google_earth/vendrell_right.kml", true);

    Circuit_preprocessor::Options options;

    options.eps_k *= 0.0002;
    options.eps_n *= 0.001;
    options.eps_c *= 0.001;
    options.eps_d *= 0.001;
    options.maximum_distance_find = 70.0;

    options.maximum_kappa = 4.0;
    options.maximum_dkappa = 4.0;
    options.maximum_dn = 2.0;
    
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, options, 500);

    // Call the xml() method, just to confirm there are no errors/valgrind issues
    circuit.xml();

    Xml_document solution_saved("./database/vendrell.xml", true);

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
