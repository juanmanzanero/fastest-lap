#include "gtest/gtest.h"
#include "src/core/actuators/engine.h"
#include "src/core/chassis/chassis_car.h"
#include <iomanip>
#include <fstream>


class Engine_curve : public ::testing::Test
{
 protected:

    Xml_document database = { "database/roberto-lot-kart-2016.xml", true };
    Engine<scalar> _engine = {database,"vehicle/rear-axle/engine/"};

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
        EXPECT_NEAR( std::abs(power[i]-_engine(speed[i]))/power[i], 0.0, 2.0e-2) << "with i = " << i ;
    }

}
