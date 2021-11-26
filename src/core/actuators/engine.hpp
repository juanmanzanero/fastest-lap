#ifndef __ENGINE_HPP__
#define __ENGINE_HPP__

#include "lion/math/matrix_extensions.h"

template<typename Timeseries_t>
inline Engine<Timeseries_t>::Engine(Xml_document& database, const std::string& path)
{
    _gear_ratio = database.get_element(path+"gear-ratio").get_value(double());

    std::vector<scalar> speed = database.get_element(path+"rpm-data").get_value(std::vector<double>())*RPM;
    std::vector<scalar> p = database.get_element(path+"power-data").get_value(std::vector<double>())*CV;
    _p = sPolynomial(speed,p,speed.size()-1,true);


    _direct_torque = false;
}


template<typename Timeseries_t>
inline Timeseries_t Engine<Timeseries_t>::operator()(const Timeseries_t angular_speed)
{
    const double ang_speed_dbl = Value(angular_speed);
    const double speed_for_zero = 15000.0/14000.0;
    // If out of bounds, just extend the power at the bound
    if ( ang_speed_dbl < _p.get_left_bound() ) 
        return std::max(0.0,ang_speed_dbl*_p[_p.get_left_bound()]/_p.get_left_bound());

    else if ( ang_speed_dbl > _p.get_right_bound() ) 
        return std::max(0.0,_p[_p.get_right_bound()]*(speed_for_zero - ang_speed_dbl/_p.get_right_bound())/(speed_for_zero - 1.0));

    else
        return _p[ang_speed_dbl];

}

#endif
