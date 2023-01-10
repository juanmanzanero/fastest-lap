#ifndef TIRE_HPP
#define TIRE_HPP

#include <iomanip>
#include "lion/thirdparty/include/logger.hpp"
#include "src/core/foundation/fastest_lap_exception.h"

template<typename Timeseries_t, size_t state_start, size_t control_start>
inline Tire<Timeseries_t,state_start,control_start>::Tire(const std::string& name, Xml_document& database, 
                  const std::string& path)
: _name(name),
  _path(path),
  _type(get_type(database.get_element(path).get_attribute("type"))),
  _frame(),
  _omega(0.0),
  _w(0.0),
  _dw(0.0),
  _v(Vector3d<Timeseries_t>(0.0)),
  _kappa(0.0),
  _lambda(0.0),
  _F(Vector3d<Timeseries_t>(0.0)),
  _T(Vector3d<Timeseries_t>(0.0))
{
    read_parameters(database, path, get_parameters(), __used_parameters);
}


template<typename Timeseries_t, size_t state_start, size_t control_start>
template<typename T>
void Tire<Timeseries_t,state_start,control_start>::set_parameter(const std::string& parameter, const T value)
{
    if ( parameter.find(_path) == 0 )
    {
        const auto found = ::set_parameter(get_parameters(), __used_parameters, parameter, _path, value); 

        if ( !found )    
            throw fastest_lap_exception(std::string("Parameter \"") + parameter + "\" was not found in Tire");
    }
    else
        throw fastest_lap_exception(std::string("Parameter \"") + parameter + "\" was not found in Tire");
}


template<typename Timeseries_t, size_t state_start, size_t control_start>
inline void Tire<Timeseries_t,state_start,control_start>::update(const Vector3d<Timeseries_t>& x0, const Vector3d<Timeseries_t>& v0, Timeseries_t omega)
{

    // Set inputs ---
    _frame.set_origin(x0, v0, Frame<Timeseries_t>::Frame_velocity_types::parent_frame);

    update(omega);
}


template<typename Timeseries_t, size_t state_start, size_t control_start>
inline void Tire<Timeseries_t,state_start,control_start>::update(Timeseries_t omega)
{
    _omega = omega;

    // Compute outputs ---

    // Tire deformations: position and velocity of the point at (0,0,R0)
    _w  =  _frame.get_absolute_position({0.0,0.0,_R0}).at(Z);
    _dw = _frame.get_absolute_velocity_in_inertial({0.0,0.0,_R0}).at(Z);

    // Contact point velocity: velocity of the point at (0,0,R0-w)
    _v =  _frame.get_absolute_velocity_in_body(get_contact_point());

    if ( _type == ONLY_LATERAL )
        _omega = _v[X]/_R0;

    // Kappa and lambda
    _kappa = kappa();
    _lambda = lambda();
}


template<typename Timeseries_t, size_t state_start, size_t control_start>
inline void Tire<Timeseries_t,state_start,control_start>::update_from_kappa(Timeseries_t kappa, const Frame<Timeseries_t>& road_frame)
{
    _kappa = kappa;

    // Compute outputs ---

    // Tire deformations: position and velocity of the point at (0,0,R0)
    _w  =  _frame.get_absolute_position({0.0,0.0,_R0}).at(Z);
    _dw = _frame.get_absolute_velocity_in_inertial({0.0,0.0,_R0}).at(Z);

    const auto fully_inflated_tire_contact_point_position_and_velocity = _frame.get_position_and_velocity_in_target(road_frame, { 0.0, 0.0, _R0 });

    _w = fully_inflated_tire_contact_point_position_and_velocity.first.z();
    _dw = fully_inflated_tire_contact_point_position_and_velocity.second.z();

    // Contact point velocity: velocity of the point at (0,0,R0-w)
    _v =  _frame.get_absolute_velocity_in_body(get_contact_point());

    if ( _type == ONLY_LATERAL )
        _omega = _v.x()/_R0;

    // omega and lambda
    _lambda = lambda();
    _omega = (1.0+kappa)*_v.x()/_R0;
}


template<typename Timeseries_t, size_t state_start, size_t control_start>
inline std::ostream& Tire<Timeseries_t,state_start,control_start>::print(std::ostream& os) const
{
    os << "Tire \"" << _name << "\"" << std::endl;    
    os << "======" << std::string(_name.length()+1, '=') << std::endl;

    out(2) << std::left << std::setw(16) << "   * R0: " << std::right << std::setw(5) << _R0  << std::endl;

    return os;
}

#endif
