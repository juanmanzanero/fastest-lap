#include "gtest/gtest.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "lion/io/Xml_document.h"

#include "src/core/applications/minimum_curvature_path.h"
#include "src/core/applications/circuit_preprocessor.h"

static void check_minimum_curvature(Xml_document& minimum_path_saved, const Minimum_curvature_path& minimum_path)
{
    const auto& x_saved = minimum_path_saved.get_element("minimum_curvature/x").get_value(std::vector<scalar>());
    EXPECT_EQ(x_saved.size(), minimum_path.x.size());

    for (size_t i = 0; i < minimum_path.x.size(); ++i)
        EXPECT_NEAR(minimum_path.x.at(i), x_saved.at(i), 1.0e-7);

    const auto& y_saved = minimum_path_saved.get_element("minimum_curvature/y").get_value(std::vector<scalar>());
    EXPECT_EQ(y_saved.size(), minimum_path.y.size());

    for (size_t i = 0; i < minimum_path.y.size(); ++i)
        EXPECT_NEAR(minimum_path.y.at(i), y_saved.at(i), 1.0e-7);

    const auto& z_saved = minimum_path_saved.get_element("minimum_curvature/z").get_value(std::vector<scalar>());
    EXPECT_EQ(z_saved.size(), minimum_path.z.size());

    for (size_t i = 0; i < minimum_path.z.size(); ++i)
        EXPECT_NEAR(minimum_path.z.at(i), z_saved.at(i), 1.0e-7);

    const auto& yaw_saved = minimum_path_saved.get_element("minimum_curvature/yaw").get_value(std::vector<scalar>());
    EXPECT_EQ(yaw_saved.size(), minimum_path.yaw.size());

    for (size_t i = 0; i < minimum_path.yaw.size(); ++i)
        EXPECT_NEAR(minimum_path.yaw.at(i), yaw_saved.at(i), 1.0e-7);

    const auto& pitch_saved = minimum_path_saved.get_element("minimum_curvature/pitch").get_value(std::vector<scalar>());
    EXPECT_EQ(pitch_saved.size(), minimum_path.pitch.size());

    for (size_t i = 0; i < minimum_path.pitch.size(); ++i)
        EXPECT_NEAR(minimum_path.pitch.at(i), pitch_saved.at(i), 1.0e-7);

    const auto& roll_saved = minimum_path_saved.get_element("minimum_curvature/roll").get_value(std::vector<scalar>());
    EXPECT_EQ(roll_saved.size(), minimum_path.roll.size());

    for (size_t i = 0; i < minimum_path.roll.size(); ++i)
        EXPECT_NEAR(minimum_path.roll.at(i), roll_saved.at(i), 1.0e-7);

    const auto& yaw_dot_saved = minimum_path_saved.get_element("minimum_curvature/yaw_dot").get_value(std::vector<scalar>());
    EXPECT_EQ(yaw_dot_saved.size(), minimum_path.yaw_dot.size());

    for (size_t i = 0; i < minimum_path.yaw_dot.size(); ++i)
        EXPECT_NEAR(minimum_path.yaw_dot.at(i), yaw_dot_saved.at(i), 1.0e-7);

    const auto& pitch_dot_saved = minimum_path_saved.get_element("minimum_curvature/pitch_dot").get_value(std::vector<scalar>());
    EXPECT_EQ(pitch_dot_saved.size(), minimum_path.pitch_dot.size());

    for (size_t i = 0; i < minimum_path.pitch_dot.size(); ++i)
        EXPECT_NEAR(minimum_path.pitch_dot.at(i), pitch_dot_saved.at(i), 1.0e-7);

    const auto& roll_dot_saved = minimum_path_saved.get_element("minimum_curvature/roll_dot").get_value(std::vector<scalar>());
    EXPECT_EQ(roll_dot_saved.size(), minimum_path.roll_dot.size());

    for (size_t i = 0; i < minimum_path.roll_dot.size(); ++i)
        EXPECT_NEAR(minimum_path.roll_dot.at(i), roll_dot_saved.at(i), 1.0e-7);

    const auto& lateral_displacement_saved = minimum_path_saved.get_element("minimum_curvature/lateral_displacement").get_value(std::vector<scalar>());
    EXPECT_EQ(lateral_displacement_saved.size(), minimum_path.n.size());

    for (size_t i = 0; i < minimum_path.n.size(); ++i)
        EXPECT_NEAR(minimum_path.n.at(i), lateral_displacement_saved.at(i), 1.0e-7);
}


TEST(Minimum_curvature_path,oval_50)
{
    Xml_document track_xml("./data/ovaltrack.xml",true);
    Track_by_arcs oval(track_xml,true);

    Xml_document matlab_result("./data/minimum_curvature_path.xml",true);

    std::vector<double> x_saved = matlab_result.get_root_element().get_child("ovaltrack_50/w").get_value(std::vector<double>());

    Minimum_curvature_path result(oval,50);

    EXPECT_TRUE(result.success);

    for (size_t i = 0; i < 50; ++i)
        EXPECT_NEAR(x_saved[i], result.get_x()[i],5.0e-2);

}


TEST(Minimum_curvature_path, catalunya_2022)
{
    Xml_document track_xml("database/tracks/catalunya_2022/catalunya_2022.xml", true);
    Circuit_preprocessor track(track_xml);

    const auto& s = track.s;
    
    auto options = Minimum_curvature_path::Options{};
    Minimum_curvature_path minimum_path(track, s, true, options);
    minimum_path.xml();

    Xml_document minimum_path_saved("data/minimum_path_catalunya_2022.xml", true);

    check_minimum_curvature(minimum_path_saved, minimum_path);
}

TEST(Minimum_curvature_path, catalunya_2022_3d)
{
    Xml_document track_xml("database/tracks/catalunya_2022/catalunya_2022_3d.xml", true);
    Circuit_preprocessor track(track_xml);

    const auto& s = track.s;
    
    auto options = Minimum_curvature_path::Options{};
    Minimum_curvature_path minimum_path(track, s, true, options);
    minimum_path.xml();

    Xml_document minimum_path_saved("data/minimum_path_catalunya_2022_3d.xml", true);

    check_minimum_curvature(minimum_path_saved, minimum_path);
}

