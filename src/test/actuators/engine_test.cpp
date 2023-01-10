#include "gtest/gtest.h"
#include "src/core/actuators/engine.h"
#include <iomanip>
#include <fstream>


class Engine_curve : public ::testing::Test
{
 protected:
    Engine_curve()
    {
        database.create_root_element("vehicle");

        database.add_element("vehicle/rear-axle/engine/rpm-data").set_value("10049.296 10225.352 10478.873 10774.648 10943.662 11169.014 11535.211 11929.577 "
                "12197.183 12464.789 12690.141 12971.831 13183.099 13366.197 13507.042 13647.887 "
                "13746.479 13816.901 13859.155 13901.408 14007.042");
        database.add_element("vehicle/rear-axle/engine/power-data").set_value("14.828 17.724 22.138 26.552 29.31  32.897 35.931 "
                "38.414 40.207 42.138 44.483 46.276 44.897 42 "
                "39.655 36.207 33.31  30.276 27.103 24.207 20.345");
        database.add_element("vehicle/rear-axle/engine/gear-ratio").set_value("8.15");
        _engine = Engine<scalar>(database, "vehicle/rear-axle/engine/",false);
    }

    Xml_document database ;
    Engine<scalar> _engine;
};


TEST_F(Engine_curve,evaluation_at_control_points)
{
    const std::vector<scalar> speed = 
        database.get_element("vehicle/rear-axle/engine/rpm-data").get_value(std::vector<double>())*RPM;
    const std::vector<scalar> power =
        database.get_element("vehicle/rear-axle/engine/power-data").get_value(std::vector<double>())*CV;

    EXPECT_EQ(speed.size(), power.size());

    for (size_t i = 0; i < speed.size(); ++i)
    {
        const scalar axle_speed = speed[i]/_engine.gear_ratio();
        const scalar power_computed = _engine(1.0,axle_speed)*axle_speed;
        EXPECT_NEAR( std::abs(power[i]-power_computed)/power[i], 0.0, 2.0e-2) << "with i = " << i ;
    }

}
