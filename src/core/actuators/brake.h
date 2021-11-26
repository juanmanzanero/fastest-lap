#ifndef __BRAKE_H__
#define __BRAKE_H__

#include "lion/foundation/types.h"
#include "lion/io/Xml_document.h"

template<typename Timeseries_t>
class Brake
{
 public:
    using Timeseries_type = Timeseries_t;

    //! Default constructor
    Brake() = default;

    //! Constructor: brake from maximum decceleration, wheel radius, and vehicle mass
    Brake(Xml_document& database, const std::string& path) :
        _Tmax(database.get_element(path + "max_torque").get_value(double())) {}; 

    //! Operator(): Apply a scaling of the maximum braking torque with the brake percentage applied
    Timeseries_t operator()(const Timeseries_t brake_percentage) { return std::min(Value(brake_percentage),1.0)*_Tmax; }

 private:
    scalar _Tmax;

};

#endif 
