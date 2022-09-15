#ifndef CHASSIS_HPP
#define CHASSIS_HPP

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
    read_parameters(database, path, get_parameters(), __used_parameters);
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
  __used_parameters(other.__used_parameters),
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

    __used_parameters = other.__used_parameters;

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
        const auto found = ::set_parameter(get_parameters(), __used_parameters, parameter, "vehicle/chassis/", value); 

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
template<size_t NSTATE, size_t NCONTROL>
inline void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::transform_states_to_input_states
    (const std::array<Timeseries_t,NSTATE>& states, const std::array<Timeseries_t,NCONTROL>& controls, std::array<Timeseries_t,NSTATE>& input_states)
{
    const auto& u     = states[state_names::U];
    const auto& v     = states[state_names::V];
    const auto& omega = states[state_names::OMEGA];

    // (1) Update the frames, the tires will want them updated
    _road_frame.set_origin({0.0,0.0,0.0},{u,v,0.0},false);
    _road_frame.set_rotation_angle(0,0.0,omega);

    // (2) Transform
    get_front_axle().transform_states_to_input_states(states, controls, input_states);
    get_rear_axle().transform_states_to_input_states(states, controls, input_states);
    
    // Update input states, just as a placeholder if I ever switch to u=vtot.cos(beta), v=vtot.sin(beta)
    input_states[input_state_names::U] = u; 
    input_states[input_state_names::V] = v; 
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state(Timeseries_t u, Timeseries_t v, Timeseries_t omega)
{
    _u = u;
    _v = v;
    _omega = omega;
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Timeseries_t Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_longitudinal_acceleration() const
{
    Vector3d<Timeseries_t> velocity = {get_u(), get_v(), 0.0};

    Vector3d<Timeseries_t> acceleration = {get_du() - velocity.y()*get_omega(), 
                              get_dv() + velocity.x()*get_omega(),
                              0.0
                             };

    return dot(velocity,acceleration)/norm(velocity);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Timeseries_t Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_lateral_acceleration() const
{
    Vector3d<Timeseries_t> velocity = {get_u(), get_v(), 0.0};

    Vector3d<Timeseries_t> acceleration = {get_du() - velocity.y()*get_omega(), 
                              get_dv() + velocity.x()*get_omega(),
                              0.0
                             };

    return cross(velocity,acceleration).z()/norm(velocity);
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
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_controls
    (const std::array<Timeseries_t,NSTATE>& input_states, const std::array<Timeseries_t,NCONTROL>& controls)
{
    get_front_axle().set_state_and_controls(input_states, controls);
    get_rear_axle().set_state_and_controls(input_states, controls);

    // u
    _u = input_states[input_state_names::U];

    // v
    _v = input_states[input_state_names::V];

    // oemga
    _omega = input_states[input_state_names::OMEGA];
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_names
    (std::array<std::string,NSTATE>& input_states, std::array<std::string,NCONTROL>& controls) const 
{
    _front_axle.set_state_and_control_names(input_states,controls);
    _rear_axle.set_state_and_control_names(input_states,controls);

    // u
    input_states[input_state_names::U] = get_name() + ".velocity.x";

    // v
    input_states[input_state_names::V] = get_name() + ".velocity.y";

    // oemga
    input_states[input_state_names::OMEGA] = get_name() + ".omega.z";
}



template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t N>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_state_and_state_derivative
    (std::array<Timeseries_t,N>& state, std::array<Timeseries_t, N>& dstate_dt) const
{
    get_front_axle().get_state_and_state_derivative(state, dstate_dt);
    get_rear_axle().get_state_and_state_derivative(state, dstate_dt);

    // dudt
    state[state_names::U] = _u;
    dstate_dt[state_names::U] = _du;

    // dvdt
    state[state_names::V] = _v;
    dstate_dt[state_names::V] = _dv;

    // domega
    state[state_names::OMEGA] = _omega;
    dstate_dt[state_names::OMEGA] = _dOmega;
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, NSTATE>& input_states_def, std::array<scalar, NSTATE>& input_states_lb, 
     std::array<scalar, NSTATE>& input_states_ub, std::array<scalar , NCONTROL>& controls_def, 
     std::array<scalar, NCONTROL>& controls_lb, std::array<scalar, NCONTROL>& controls_ub,
    scalar velocity_x_lb, scalar velocity_x_ub, scalar velocity_y_ub, scalar omega_ub ) const
{
    get_front_axle().set_state_and_control_upper_lower_and_default_values(input_states_def, input_states_lb, input_states_ub,
        controls_def, controls_lb, controls_ub);
    get_rear_axle().set_state_and_control_upper_lower_and_default_values(input_states_def, input_states_lb, input_states_ub,
        controls_def, controls_lb, controls_ub);

    // State -----
    input_states_def[input_state_names::U] = velocity_x_lb;
    input_states_lb[input_state_names::U]  = velocity_x_lb;
    input_states_ub[input_state_names::U]  = velocity_x_ub;

    // v
    input_states_def[input_state_names::V] = 0.0;
    input_states_lb[input_state_names::V]  = -velocity_y_ub;
    input_states_ub[input_state_names::V]  =  velocity_y_ub;

    // oemga
    input_states_def[input_state_names::OMEGA] = 0.0;
    input_states_lb[input_state_names::OMEGA]  = -omega_ub;
    input_states_ub[input_state_names::OMEGA]  =  omega_ub;
}

#endif
