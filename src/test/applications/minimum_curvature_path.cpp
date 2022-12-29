#include "gtest/gtest.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "lion/io/Xml_document.h"

#include "src/core/applications/minimum_curvature_path.h"

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

    const auto& theta_saved = minimum_path_saved.get_element("minimum_curvature/theta").get_value(std::vector<scalar>());
    EXPECT_EQ(theta_saved.size(), minimum_path.theta.size());

    for (size_t i = 0; i < minimum_path.theta.size(); ++i)
        EXPECT_NEAR(minimum_path.theta.at(i), theta_saved.at(i), 1.0e-7);

    const auto& kappa_saved = minimum_path_saved.get_element("minimum_curvature/kappa").get_value(std::vector<scalar>());
    EXPECT_EQ(kappa_saved.size(), minimum_path.kappa.size());

    for (size_t i = 0; i < minimum_path.kappa.size(); ++i)
        EXPECT_NEAR(minimum_path.kappa.at(i), kappa_saved.at(i), 1.0e-7);

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
    Track_by_polynomial track(track_xml);

    const auto& s = track.get_preprocessor().s;
    
    auto options = Minimum_curvature_path::Options{};
    Minimum_curvature_path minimum_path(track, s, true, options);
    minimum_path.xml();

    Xml_document minimum_path_saved("data/minimum_path_catalunya_2022.xml", true);

    check_minimum_curvature(minimum_path_saved, minimum_path);
}
