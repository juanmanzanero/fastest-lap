#ifndef __CHASSIS_HPP__
#define __CHASSIS_HPP__

#include "src/core/foundation/fastest_lap_exception.h"

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis(
    const FrontAxle_t& front_axle, const RearAxle_t& rear_axle, const std::string& path)
: _inertial_frame(),
  _road_frame(Frame<Timeseries_t>(Vector3d<Timeseries_t>(0.0), Vector3d<Timeseries_t>(0.0), {0.0}, {0.0}, {Z}, _inertial_frame)),
  _chassis_frame(Frame<Timeseries_t>(Vector3d<Timeseries_t>(0.0), Vector3d<Timeseries_t>(0.0), {}, {}, {}, _road_frame)),
  _m(0.0),
  _I(0.0),
  _front_axle(front_axle),
  _rear_axle(rear_axle),
  _F(Vector3d<Timeseries_t>(0.0)),
  _T(Vector3d<Timeseries_t>(0.0))
{
    // Set the front and rear axles new frames parents
    _front_axle.get_frame().set_parent(_chassis_frame); 
    _rear_axle.get_frame().set_parent(_chassis_frame); 
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis(
    const FrontAxle_t& front_axle, const RearAxle_t& rear_axle,
    Xml_document& database, const std::string& path)
: Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis(front_axle, rear_axle, path)
{
    read_parameters(database, path, get_parameters());
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis(const Chassis& other)
: _inertial_frame(),
  _road_frame(other._road_frame),
  _chassis_frame(other._chassis_frame),
  _m(other._m),
  _I(other._I),
  _rho(other._rho),
  _cd(other._cd),
  _cl(other._cl),
  _A(other._A),
  _front_axle(other._front_axle),
  _rear_axle(other._rear_axle),
  _F(other._F),
  _T(other._T)
{
    _road_frame.set_parent(_inertial_frame);

    // Update parent of chassis frame to new road frame
    _chassis_frame.set_parent(_road_frame);

    // Set the front and rear axles new frames parents
    _front_axle.get_frame().set_parent(_chassis_frame); 
    _rear_axle.get_frame().set_parent(_chassis_frame); 
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>& Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::operator=(const Chassis& other)
{
    _inertial_frame = Frame<Timeseries_t>();
    _road_frame    = other._road_frame;
    _chassis_frame = other._chassis_frame;
    _front_axle    = other._front_axle;
    _rear_axle     = other._rear_axle;
    _m             = other._m;
    _I             = other._I;
    _rho           = other._rho;
    _cd            = other._cd; 
    _cl            = other._cl; 
    _A             = other._A;  

    _road_frame.set_parent(_inertial_frame);

    // Update parent of chassis frame to new road frame
    _chassis_frame.set_parent(_road_frame);

    // Set the front and rear axles new frames parents
    _front_axle.get_frame().set_parent(_chassis_frame); 
    _rear_axle.get_frame().set_parent(_chassis_frame); 

    return *this;
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<typename T>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_parameter(const std::string& parameter, const T value)
{
    // Check if the parameter goes to this object
    if ( parameter.find("vehicle/chassis/") == 0 )
    {
        // Find the parameter in the database
        const auto found = ::set_parameter(get_parameters(), parameter, "vehicle/chassis/", value); 

        // If not found, throw an exception
        if ( !found )
            throw fastest_lap_exception("Parameter \"" + parameter + "\" was not found");
    }
    else
    {
        // Look for the parameter in front axle
        auto found = get_front_axle().set_parameter(parameter, value);

        if ( !found )
            found = get_rear_axle().set_parameter(parameter, value);
    
        if ( !found )
            throw fastest_lap_exception("Parameter \"" + parameter + "\" was not found");
    }
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::fill_xml(Xml_document& doc) const
{
    // Write the parameters of this class
    ::write_parameters(doc, "vehicle/chassis/", get_parameters());

    // Write the parameters of the axles
    _front_axle.fill_xml(doc);
    _rear_axle.fill_xml(doc); 
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state(Timeseries_t u, Timeseries_t v, Timeseries_t omega)
{
    _u = u;
    _v = v;
    _omega = omega;
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Vector3d<Timeseries_t> Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_aerodynamic_force() const
{
    const Vector3d<Timeseries_t> vel = _road_frame.get_absolute_velocity_in_body();

    return { -0.5*_rho*_cd*_A*vel[X]*vel[X], 0.0, 0.5*_rho*_cl*_A*vel[X]*vel[X] };
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Timeseries_t Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_understeer_oversteer_indicator() const
{
 if constexpr(FrontAxle_t::NTIRES == 2)
 {
    // (1) Get ideal curvature: kappa = omega/vtot         
    const auto kappa = _omega/sqrt(_u*_u + _v*_v);

    // (2) Get tires position
    const auto track = get_front_axle().get_track();  // Distance between the two front tires
    const auto x_com = -get_rear_axle().get_frame().get_position_and_velocity_in_target(_road_frame).first.x(); // Distance from rear axle to CoM
    const auto wb    = get_front_axle().get_frame().get_position_and_velocity_in_target(_road_frame).first.x() + x_com; // Wheel-base

    // (3) Get tires steering angle
    const auto delta_left  = get_front_axle().template get_tire<0>().get_frame().get_rotation_angles().front();
    const auto delta_right = get_front_axle().template get_tire<1>().get_frame().get_rotation_angles().front();

    // (4) Define the inside and outside tires: if kappa > 0, the interior tire is the right
    const auto delta_inside = ( kappa >= 0.0 ? delta_right : - delta_left );
    const auto delta_outside = ( kappa >= 0.0 ? delta_left : - delta_right );
    const auto abs_kappa = ( kappa >= 0.0 ? kappa : -kappa );

    // (5) Compute the ideal steering angles for the inside and outside tires
    const auto delta_inside_ideal = atan(wb*abs_kappa /(sqrt(1.0 - abs_kappa*abs_kappa*x_com*x_com) - 0.5*track*abs_kappa));
    const auto delta_outside_ideal = atan(wb*abs_kappa /(sqrt(1.0 - abs_kappa*abs_kappa*x_com*x_com) + 0.5*track*abs_kappa));

    // (6) Compute the difference between real and ideal steering angles
    const auto d_delta_inside = delta_inside_ideal - delta_inside;
    const auto d_delta_outside = delta_outside_ideal - delta_outside;

    // (7) Compute the oversteering parameter

    // (7.1) If they have different sign, the steering is neutral
    if ( d_delta_inside*d_delta_outside <= 0.0 )
    {
        return Timeseries_t{0.0};
    }
    else
    {
        // (7.2) If they are positive (ideal delta bigger than real, the car is oversteering)
        if ( d_delta_inside > 0.0 )
            return ( d_delta_inside > d_delta_outside ? d_delta_outside : d_delta_inside );
        else
            return ( d_delta_inside > d_delta_outside ? d_delta_inside : d_delta_outside );
    }
 }
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::update(Timeseries_t x, Timeseries_t y, Timeseries_t psi)
{
    const Timeseries_t dx = _u*cos(psi) - _v*sin(psi);
    const Timeseries_t dy = _u*sin(psi) + _v*cos(psi);

    _road_frame.set_origin({x,y,0.0},{dx,dy,0.0},false);
    _road_frame.set_rotation_angle(0,psi,_omega);
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_controls(const std::array<Timeseries_t,NSTATE>& q, const std::array<Timeseries_t,NCONTROL>& u)
{
    get_front_axle().set_state_and_controls(q,u);
    get_rear_axle().set_state_and_controls(q,u);

    // u
    _u = q[IU];

    // v
    _v = q[IV];

    // oemga
    _omega = q[IOMEGA];    
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_names(std::array<std::string,NSTATE>& q, std::array<std::string,NCONTROL>& u) 
{
    FrontAxle_t::set_state_and_control_names(q,u);
    RearAxle_t::set_state_and_control_names(q,u);

    // u
    q[IU] = "u";

    // v
    q[IV] = "v";

    // oemga
    q[IOMEGA] = "omega";    
}



template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t N>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_state_derivative(std::array<Timeseries_t,N>& dqdt) const
{
    get_front_axle().get_state_derivative(dqdt);
    get_rear_axle().get_state_derivative(dqdt);

    // dudt
    dqdt[IIDU] = _du;

    // dvdt
    dqdt[IIDV] = _dv;

    // domega
    dqdt[IIDOMEGA] = _dOmega;
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, NSTATE>& q_def     , std::array<scalar, NSTATE>& q_lb     , std::array<scalar, NSTATE>& q_ub     ,
    std::array<scalar , NCONTROL>& u_def   , std::array<scalar, NCONTROL>& u_lb   , std::array<scalar, NCONTROL>& u_ub,
    scalar velocity_x_lb, scalar velocity_x_ub, scalar velocity_y_ub, scalar omega_ub ) const
{
    get_front_axle().set_state_and_control_upper_lower_and_default_values(q_def, q_lb, q_ub, u_def, u_lb, u_ub);
    get_rear_axle().set_state_and_control_upper_lower_and_default_values(q_def, q_lb, q_ub, u_def, u_lb, u_ub);

    // State -----
    q_def[IU] = velocity_x_lb;
    q_lb[IU]  = velocity_x_lb;
    q_ub[IU]  = velocity_x_ub;

    // v
    q_def[IV] = 0.0;
    q_lb[IV]  = -velocity_y_ub;
    q_ub[IV]  =  velocity_y_ub;

    // oemga
    q_def[IOMEGA] = 0.0;    
    q_lb[IOMEGA]  = -omega_ub;
    q_ub[IOMEGA]  =  omega_ub;
}

#endif
