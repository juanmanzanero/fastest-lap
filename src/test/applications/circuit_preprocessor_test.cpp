#include "src/core/applications/circuit_preprocessor.h"
#include "gtest/gtest.h"

extern bool is_valgrind;

TEST(Circuit_preprocessor_test, catalunya_500)
{
    #ifndef NDEBUG
        GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document coord_left_kml("./database/google_earth/Catalunya_left.kml", true);
    Xml_document coord_right_kml("./database/google_earth/Catalunya_right.kml", true);
    
    Circuit_preprocessor circuit(coord_left_kml, coord_right_kml, 500, true);

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
        EXPECT_DOUBLE_EQ(circuit.r_centerline[i].x(), x[i]) << " with i = " << i; 
        EXPECT_DOUBLE_EQ(circuit.r_centerline[i].y(), y[i]) << " with i = " << i; 
        EXPECT_DOUBLE_EQ(circuit.theta[i], theta[i]) << " with i = " << i; 
        EXPECT_DOUBLE_EQ(circuit.kappa[i], kappa[i]) << " with i = " << i; 
        EXPECT_DOUBLE_EQ(circuit.nl[i], nl[i]) << " with i = " << i; 
        EXPECT_DOUBLE_EQ(circuit.nr[i], nr[i]) << " with i = " << i; 
        EXPECT_DOUBLE_EQ(circuit.dkappa[i], dkappa[i]) << " with i = " << i; 
        EXPECT_DOUBLE_EQ(circuit.dnl[i], dnl[i]) << " with i = " << i; 
        EXPECT_DOUBLE_EQ(circuit.dnr[i], dnr[i]) << " with i = " << i; 
    }
}
