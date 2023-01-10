#include "gtest/gtest.h"
#include "src/core/vehicles/track_by_polynomial.h"
#include "src/main/c/fastestlapc.h"
#include <unordered_map>

TEST(Track_by_polynomial_test, evaluation_at_nodes)
{
    Xml_document catalunya = {"./database/tracks/catalunya/catalunya.xml", true};
     
    const auto arclength = catalunya.get_element("circuit/data/arclength").get_value(std::vector<scalar>());

    const Circuit_preprocessor circuit(catalunya);
    
    Track_by_polynomial track(circuit);

    EXPECT_NEAR(track.get_total_length(),circuit.track_length, 1.0e-16);
    scalar ds = track.get_total_length()/circuit.n_elements;
    for (size_t i = 0; i < circuit.n_points; ++i)
    {
        EXPECT_NEAR(arclength[i],i*ds,1.0e-14*ds*circuit.n_points);
        auto [r,euler_angles,euler_angles_dot] = track(arclength[i]);
        EXPECT_NEAR(r.x(), circuit.r_centerline[i].x(), 2.0e-13);
        EXPECT_NEAR(r.y(), circuit.r_centerline[i].y(), 2.0e-13);

        EXPECT_NEAR(euler_angles.roll(), 0.0, 1.0e-15);
        EXPECT_NEAR(euler_angles.pitch(), 0.0, 1.0e-15);
        EXPECT_NEAR(euler_angles.yaw(), circuit.yaw[i], 2.0e-15);

        EXPECT_NEAR(euler_angles_dot.roll(), 0.0, 1.0e-15);
        EXPECT_NEAR(euler_angles_dot.pitch(), 0.0, 1.0e-15);
        EXPECT_NEAR(euler_angles_dot.yaw(), circuit.yaw_dot[i], 2.0e-15);
    }
}

std::unordered_map<std::string,Track_by_polynomial>& get_table_track();

TEST(Track_by_polynomial_test, create_track_c_api)
{
#ifdef TEST_LIBFASTESTLAPC
    set_print_level(0);
    create_track_from_xml("test_track", "./database/tracks/catalunya/catalunya.xml");
    
    // Create the track herein
    Xml_document catalunya = {"./database/tracks/catalunya/catalunya.xml", true};
    const Circuit_preprocessor circuit(catalunya);
    Track_by_polynomial track(circuit);

    // Check that the track is on the table
    EXPECT_EQ(get_table_track().count("test_track"), 1);

    // Compare number of points
    EXPECT_EQ(circuit.n_points, track_download_number_of_points("test_track"));

    // Compare variables
    std::vector<scalar> var(circuit.n_points);

    track_download_data(var.data(), "test_track", circuit.n_points, "arclength");
    for (size_t i = 0; i < circuit.n_points; ++i)
        EXPECT_EQ(circuit.s[i], var[i]);

    track_download_data(var.data(), "test_track", circuit.n_points, "heading-angle");
    for (size_t i = 0; i < circuit.n_points; ++i)
        EXPECT_EQ(circuit.yaw[i], var[i]);

    track_download_data(var.data(), "test_track", circuit.n_points, "curvature");
    for (size_t i = 0; i < circuit.n_points; ++i)
        EXPECT_EQ(circuit.yaw_dot[i], var[i]);

    track_download_data(var.data(), "test_track", circuit.n_points, "distance-left-boundary");
    for (size_t i = 0; i < circuit.n_points; ++i)
        EXPECT_EQ(circuit.nl[i], var[i]);

    track_download_data(var.data(), "test_track", circuit.n_points, "distance-right-boundary");
    for (size_t i = 0; i < circuit.n_points; ++i)
        EXPECT_EQ(circuit.nr[i], var[i]);

    track_download_data(var.data(), "test_track", circuit.n_points, "left.x");
    for (size_t i = 0; i < circuit.n_points; ++i)
        EXPECT_EQ(circuit.r_left[i].x(), var[i]);

    track_download_data(var.data(), "test_track", circuit.n_points, "left.y");
    for (size_t i = 0; i < circuit.n_points; ++i)
        EXPECT_EQ(circuit.r_left[i].y(), var[i]);

    track_download_data(var.data(), "test_track", circuit.n_points, "right.x");
    for (size_t i = 0; i < circuit.n_points; ++i)
        EXPECT_EQ(circuit.r_right[i].x(), var[i]);

    track_download_data(var.data(), "test_track", circuit.n_points, "right.y");
    for (size_t i = 0; i < circuit.n_points; ++i)
        EXPECT_EQ(circuit.r_right[i].y(), var[i]);

    track_download_data(var.data(), "test_track", circuit.n_points, "centerline.x");
    for (size_t i = 0; i < circuit.n_points; ++i)
        EXPECT_EQ(circuit.r_centerline[i].x(), var[i]);

    track_download_data(var.data(), "test_track", circuit.n_points, "centerline.y");
    for (size_t i = 0; i < circuit.n_points; ++i)
        EXPECT_EQ(circuit.r_centerline[i].y(), var[i]);

    // Delete
    delete_variable("test_track");
    EXPECT_EQ(get_table_track().count("test_track"), 0);
#else
    GTEST_SKIP();
#endif
}
