#include "gtest/gtest.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"

#define SKIP_TIMESERIES
using timeseries = CppAD::AD<double>;

#include "src/core/vehicles/track_by_arcs.h"
#include "src/core/applications/minimum_curvature_path.h"


TEST(Minimum_curvature_path,oval_50)
{
    Xml_document track_xml("./data/ovaltrack.xml",true);
    Track_by_arcs oval(track_xml,true);

    Xml_document matlab_result("./data/minimum_curvature_path.xml",true);

    std::vector<double> x_saved = matlab_result.get_root_element().get_child("ovaltrack_50/w").get_value(std::vector<double>());

    Minimum_curvature_path result(oval,50);

    EXPECT_TRUE(result.get_success());

    for (size_t i = 0; i < 50; ++i)
        EXPECT_NEAR(x_saved[i], result.get_x()[i],5.0e-2);

}
