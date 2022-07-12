#include "gtest/gtest.h"
#include "src/core/actuators/engine.h"
#include "src/core/chassis/chassis_car_6dof.h"
#include <iomanip>
#include <fstream>


class Engine_curve : public ::testing::Test
{
 protected:

    Xml_document database = { "data/kart-sample.xml", true };
    Engine<scalar> _engine = {database,"vehicle/rear-axle/engine/",false};
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
