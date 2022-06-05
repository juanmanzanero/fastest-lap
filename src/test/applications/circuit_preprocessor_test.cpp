#include "src/core/applications/circuit_preprocessor.h"
#include "gtest/gtest.h"

extern bool is_valgrind;

/*
TEST(Circuit_preprocessor_test, miami_2000)
{
    #ifndef NDEBUG
          GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document coord_left_kml("./database/tracks/miami/miami_left.kml", true);
    Xml_document coord_right_kml("./database/tracks/miami/miami_right.kml", true);
    
    Circuit_preprocessor::Options opts;
    opts.print_level = 5;
    opts.eps_c *= 5.0;
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, opts, 2000);

    circuit.xml()->save("miami_2000.xml");

    Xml_document solution_saved("./database/tracks/miami/miami_1000.xml", true);

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

    EXPECT_EQ(circuit.n_points,700);
    EXPECT_EQ(circuit.r_centerline.size(),700);
    EXPECT_EQ(circuit.theta.size(),700);
    EXPECT_EQ(circuit.kappa.size(),700);
    EXPECT_EQ(circuit.nl.size(),700);
    EXPECT_EQ(circuit.nr.size(),700);
    EXPECT_EQ(circuit.dkappa.size(),700);
    EXPECT_EQ(circuit.dnl.size(),700);
    EXPECT_EQ(circuit.dnr.size(),700);

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

    for (size_t i = 0; i < circuit.n_points-1; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i+1].x()-circuit.r_centerline[i].x(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(cos(circuit.theta[i])+cos(circuit.theta[i+1])), 1.0e-7);
        EXPECT_NEAR(circuit.r_centerline[i+1].y()-circuit.r_centerline[i].y(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(sin(circuit.theta[i])+sin(circuit.theta[i+1])), 1.0e-7);
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
*/



TEST(Circuit_preprocessor_test, museo_closed)
{
    Xml_document coord_left_kml("./database/tracks/fa_museo/Museo_short_left.kml", true);
    Xml_document coord_right_kml("./database/tracks/fa_museo/Museo_short_right.kml", true);

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

    for (size_t i = 0; i < circuit.n_points-1; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i+1].x()-circuit.r_centerline[i].x(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(cos(circuit.theta[i])+cos(circuit.theta[i+1])), 1.0e-7);
        EXPECT_NEAR(circuit.r_centerline[i+1].y()-circuit.r_centerline[i].y(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(sin(circuit.theta[i])+sin(circuit.theta[i+1])), 1.0e-7);
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
    Xml_document coord_left_kml("./database/tracks/catalunya/Catalunya_left.kml", true);
    Xml_document coord_right_kml("./database/tracks/catalunya/Catalunya_right.kml", true);

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

    for (size_t i = 0; i < circuit.n_points-1; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i+1].x()-circuit.r_centerline[i].x(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(cos(circuit.theta[i])+cos(circuit.theta[i+1])), 1.0e-7);
        EXPECT_NEAR(circuit.r_centerline[i+1].y()-circuit.r_centerline[i].y(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(sin(circuit.theta[i])+sin(circuit.theta[i+1])), 1.0e-7);
    }


}

TEST(Circuit_preprocessor_test, melbourne_700)
{
    #ifndef NDEBUG
          GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document coord_left_kml("./database/tracks/melbourne/melbourne_left.kml", true);
    Xml_document coord_right_kml("./database/tracks/melbourne/melbourne_right.kml", true);
    
    Circuit_preprocessor::Options opts;
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, opts, 700);

    circuit.xml();

    Xml_document solution_saved("./database/tracks/melbourne/melbourne_700.xml", true);

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

    EXPECT_EQ(circuit.n_points,700);
    EXPECT_EQ(circuit.r_centerline.size(),700);
    EXPECT_EQ(circuit.theta.size(),700);
    EXPECT_EQ(circuit.kappa.size(),700);
    EXPECT_EQ(circuit.nl.size(),700);
    EXPECT_EQ(circuit.nr.size(),700);
    EXPECT_EQ(circuit.dkappa.size(),700);
    EXPECT_EQ(circuit.dnl.size(),700);
    EXPECT_EQ(circuit.dnr.size(),700);

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

    for (size_t i = 0; i < circuit.n_points-1; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i+1].x()-circuit.r_centerline[i].x(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(cos(circuit.theta[i])+cos(circuit.theta[i+1])), 1.0e-7);
        EXPECT_NEAR(circuit.r_centerline[i+1].y()-circuit.r_centerline[i].y(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(sin(circuit.theta[i])+sin(circuit.theta[i+1])), 1.0e-7);
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


TEST(Circuit_preprocessor_test, melbourne_adapted)
{
    #ifndef NDEBUG
          GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document coord_left_kml("./database/tracks/melbourne/melbourne_left.kml", true);
    Xml_document coord_right_kml("./database/tracks/melbourne/melbourne_right.kml", true);

    std::vector<scalar> s_bkp = {0.0000000000000000,     215.6155437858050732,     225.6155437858050732,     426.1900513192233007,     436.1900513192233007,     930.0647657741851617,     940.0647657741851617,    1291.0496358314749159,    1301.0496358314749159,    1749.8012415292823789,    1759.8012415292823789,    1975.4167853150893279,    1985.4167853150893279,    3171.1791673798661577,    3181.1791673798661577,    3562.2461099419315360,    3572.2461099419315360,    3945.7925343778033493,    3955.7925343778033493,    4178.9285962898038633,    4188.9285962898038633,    4276.6953319303202079,    4286.6953319303202079,    4690.3238288709662811,    4700.3238288709662811 };

    std::vector<scalar> ds_bkp = {14.0000000000000000,      14.0000000000000000,       6.0000000000000000,       6.0000000000000000,      14.0000000000000000,      14.0000000000000000,       6.0000000000000000,       6.0000000000000000,      14.0000000000000000,      14.0000000000000000,       6.0000000000000000,       6.0000000000000000,      14.0000000000000000,      14.0000000000000000,       6.0000000000000000,       6.0000000000000000,      14.0000000000000000,      14.0000000000000000,       6.0000000000000000,       6.0000000000000000,      14.0000000000000000,      14.0000000000000000,       6.0000000000000000,       6.0000000000000000,      14.0000000000000000};
    
    Circuit_preprocessor::Options opts;
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, opts, s_bkp, ds_bkp);

    circuit.xml();

    Xml_document solution_saved("./database/tracks/melbourne/melbourne_adapted.xml", true);

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

    EXPECT_EQ(circuit.n_points,545);
    EXPECT_EQ(circuit.r_centerline.size(),545);
    EXPECT_EQ(circuit.theta.size(),545);
    EXPECT_EQ(circuit.kappa.size(),545);
    EXPECT_EQ(circuit.nl.size(),545);
    EXPECT_EQ(circuit.nr.size(),545);
    EXPECT_EQ(circuit.dkappa.size(),545);
    EXPECT_EQ(circuit.dnl.size(),545);
    EXPECT_EQ(circuit.dnr.size(),545);

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

    for (size_t i = 0; i < circuit.n_points-1; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i+1].x()-circuit.r_centerline[i].x(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(cos(circuit.theta[i])+cos(circuit.theta[i+1])), 1.0e-7);
        EXPECT_NEAR(circuit.r_centerline[i+1].y()-circuit.r_centerline[i].y(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(sin(circuit.theta[i])+sin(circuit.theta[i+1])), 1.0e-7);
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

    Xml_document coord_left_kml("./database/tracks/catalunya/Catalunya_left.kml", true);
    Xml_document coord_right_kml("./database/tracks/catalunya/Catalunya_right.kml", true);
    
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, {}, 500);

    circuit.xml();

    Xml_document solution_saved("./database/tracks/catalunya/catalunya_discrete.xml", true);

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

    for (size_t i = 0; i < circuit.n_points-1; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i+1].x()-circuit.r_centerline[i].x(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(cos(circuit.theta[i])+cos(circuit.theta[i+1])), 1.0e-7);
        EXPECT_NEAR(circuit.r_centerline[i+1].y()-circuit.r_centerline[i].y(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(sin(circuit.theta[i])+sin(circuit.theta[i+1])), 1.0e-7);
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

    Xml_document coord_left_kml("./database/tracks/catalunya/Catalunya_left.kml", true);
    Xml_document coord_right_kml("./database/tracks/catalunya/Catalunya_right.kml", true);

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

    Xml_document solution_saved("./data/catalunya_adapted_by_coords.xml", true);

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

    for (size_t i = 0; i < circuit.n_points-1; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i+1].x()-circuit.r_centerline[i].x(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(cos(circuit.theta[i])+cos(circuit.theta[i+1])), 1.0e-7);
        EXPECT_NEAR(circuit.r_centerline[i+1].y()-circuit.r_centerline[i].y(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(sin(circuit.theta[i])+sin(circuit.theta[i+1])), 1.0e-7);
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

    Xml_document coord_left_kml("./database/tracks/catalunya/Catalunya_left.kml", true);
    Xml_document coord_right_kml("./database/tracks/catalunya/Catalunya_right.kml", true);

    std::vector<scalar> s_distr = {0.0};

    while (s_distr.back() < 4633.0 )
        s_distr.push_back(s_distr.back() + 4.5);

    std::vector<scalar> ds_distr(s_distr.size());
    for (size_t i = 0; i < s_distr.size(); ++i)
    {
        ds_distr[i] = 9.0;
        if (s_distr[i] > 950.0 && s_distr[i] < 1200.0)
            ds_distr[i] = 4.5;

        if (s_distr[i] > 1150.0 && s_distr[i] < 1418.0)
            ds_distr[i] = 4.5;

        if (s_distr[i] > 3633.17 && s_distr[i] < 3850.0)
            ds_distr[i] = 4.5;

        if (s_distr[i] > 4050.0 && s_distr[i] < 4430.0)
            ds_distr[i] = 2.5;
    }


    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, {}, s_distr, ds_distr);

    circuit.xml();

    Xml_document solution_saved("./database/tracks/catalunya/catalunya_adapted.xml", true);

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

    EXPECT_EQ(circuit.n_points,697);
    EXPECT_EQ(circuit.r_centerline.size(),697);
    EXPECT_EQ(circuit.theta.size(),697);
    EXPECT_EQ(circuit.kappa.size(),697);
    EXPECT_EQ(circuit.nl.size(),697);
    EXPECT_EQ(circuit.nr.size(),697);
    EXPECT_EQ(circuit.dkappa.size(),697);
    EXPECT_EQ(circuit.dnl.size(),697);
    EXPECT_EQ(circuit.dnr.size(),697);

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

    for (size_t i = 0; i < circuit.n_points-1; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i+1].x()-circuit.r_centerline[i].x(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(cos(circuit.theta[i])+cos(circuit.theta[i+1])), 1.0e-7);
        EXPECT_NEAR(circuit.r_centerline[i+1].y()-circuit.r_centerline[i].y(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(sin(circuit.theta[i])+sin(circuit.theta[i+1])), 1.0e-7);
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

    Xml_document coord_left_kml("./database/tracks/vendrell/vendrell_left.kml", true);
    Xml_document coord_right_kml("./database/tracks/vendrell/vendrell_right.kml", true);

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

    Xml_document solution_saved("./database/tracks/vendrell/vendrell.xml", true);

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

    for (size_t i = 0; i < circuit.n_points-1; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i+1].x()-circuit.r_centerline[i].x(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(cos(circuit.theta[i])+cos(circuit.theta[i+1])), 1.0e-7);
        EXPECT_NEAR(circuit.r_centerline[i+1].y()-circuit.r_centerline[i].y(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(sin(circuit.theta[i])+sin(circuit.theta[i+1])), 1.0e-7);
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


TEST(Circuit_preprocessor_test, imola_adapted)
{
    #ifndef NDEBUG
          GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document coord_left_kml("./database/tracks/imola/imola_left.kml", true);
    Xml_document coord_right_kml("./database/tracks/imola/imola_right.kml", true);

    std::vector<scalar> s_bkp = {  0.0, 2650.0, 2660.0, 2950.0, 2960.0, 3300.0, 3310.0, 3450.0, 3460.0, 7000.0};
    std::vector<scalar> ds_bkp = { 8.0,    8.0,    6.0,    6.0,    8.0,    8.0,    4.0,    4.0,    8.0,  8.0};
    
    Circuit_preprocessor::Options opts;
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, opts, s_bkp, ds_bkp);

    circuit.xml();

    Xml_document solution_saved("./database/tracks/imola/imola_adapted.xml", true);

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

    EXPECT_EQ(circuit.n_points,643);
    EXPECT_EQ(circuit.r_centerline.size(),643);
    EXPECT_EQ(circuit.theta.size(),643);
    EXPECT_EQ(circuit.kappa.size(),643);
    EXPECT_EQ(circuit.nl.size(),643);
    EXPECT_EQ(circuit.nr.size(),643);
    EXPECT_EQ(circuit.dkappa.size(),643);
    EXPECT_EQ(circuit.dnl.size(),643);
    EXPECT_EQ(circuit.dnr.size(),643);

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

    for (size_t i = 0; i < circuit.n_points-1; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i+1].x()-circuit.r_centerline[i].x(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(cos(circuit.theta[i])+cos(circuit.theta[i+1])), 1.0e-7);
        EXPECT_NEAR(circuit.r_centerline[i+1].y()-circuit.r_centerline[i].y(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(sin(circuit.theta[i])+sin(circuit.theta[i+1])), 1.0e-7);
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

TEST(Circuit_preprocessor_test, catalunya_2022_adapted)
{
    #ifndef NDEBUG
        GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document coord_left_kml("./database/tracks/catalunya_2022/catalunya_2022_left.kml", true);
    Xml_document coord_right_kml("./database/tracks/catalunya_2022/catalunya_2022_right.kml", true);

    std::vector<scalar> s_distr = {0.0};

    while (s_distr.back() < 5000.0 )
        s_distr.push_back(s_distr.back() + 4.5);

    std::vector<scalar> ds_distr(s_distr.size());
    for (size_t i = 0; i < s_distr.size(); ++i)
    {
        ds_distr[i] = 9.0;
        if (s_distr[i] > 950.0 && s_distr[i] < 1200.0)
            ds_distr[i] = 4.5;

        if (s_distr[i] > 1150.0 && s_distr[i] < 1418.0)
            ds_distr[i] = 4.5;

        if (s_distr[i] > 3633.17 && s_distr[i] < 3850.0)
            ds_distr[i] = 4.5;

        if (s_distr[i] > 4050.0 && s_distr[i] < 4430.0)
            ds_distr[i] = 2.5;
    }


    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, {}, s_distr, ds_distr);

    circuit.xml();

    Xml_document solution_saved("./database/tracks/catalunya_2022/catalunya_2022_adapted.xml", true);

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

    EXPECT_EQ(circuit.n_points,700);
    EXPECT_EQ(circuit.r_centerline.size(),700);
    EXPECT_EQ(circuit.theta.size(),700);
    EXPECT_EQ(circuit.kappa.size(),700);
    EXPECT_EQ(circuit.nl.size(),700);
    EXPECT_EQ(circuit.nr.size(),700);
    EXPECT_EQ(circuit.dkappa.size(),700);
    EXPECT_EQ(circuit.dnl.size(),700);
    EXPECT_EQ(circuit.dnr.size(),700);

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

    for (size_t i = 0; i < circuit.n_points-1; ++i)
    {
        EXPECT_NEAR(circuit.r_centerline[i+1].x()-circuit.r_centerline[i].x(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(cos(circuit.theta[i])+cos(circuit.theta[i+1])), 1.0e-7);
        EXPECT_NEAR(circuit.r_centerline[i+1].y()-circuit.r_centerline[i].y(), 
                    (circuit.s[i+1]-circuit.s[i])*0.5*(sin(circuit.theta[i])+sin(circuit.theta[i+1])), 1.0e-7);
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
