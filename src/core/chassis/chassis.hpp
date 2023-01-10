#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include "src/core/foundation/fastest_lap_exception.h"

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Chassis(
    const FrontAxle_t& front_axle, const RearAxle_t& rear_axle, const std::string& path)
: _inertial_frame(),
  _road_frame(Frame<Timeseries_t>(Vector3d<Timeseries_t>(0.0), Vector3d<Timeseries_t>(0.0), { 0.0,0.0,0.0,0.0 }, { 0.0,0.0,0.0,0.0 }, { Z,Y,X,Z }, _inertial_frame, {})),
  _chassis_frame(Frame<Timeseries_t>(Vector3d<Timeseries_t>(0.0), Vector3d<Timeseries_t>(0.0), {}, {}, {}, _road_frame, {})),
  _mass_kg(0.0),
  _I_kgm2(0.0),
  _front_axle(front_axle),
  _rear_axle(rear_axle),
  _total_force_N(Vector3d<Timeseries_t>(0.0)),
  _total_torque_Nm(Vector3d<Timeseries_t>(0.0))
{
    // Set the front and rear axles new frames parents
    _front_axle.get_frame().set_parent(_chassis_frame); 
    _rear_axle.get_frame().set_parent(_chassis_frame); 
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Chassis(
    const FrontAxle_t& front_axle, const RearAxle_t& rear_axle,
    Xml_document& database, const std::string& path)
: Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Chassis(front_axle, rear_axle, path)
{
    read_parameters(database, path, get_parameters(), __used_parameters);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Chassis(const Chassis& other)
: __used_parameters(other.__used_parameters),
  _velocity_x_mps(other._velocity_x_mps),
  _velocity_y_mps(other._velocity_y_mps),
  _yaw_rate_radps(other._yaw_rate_radps),
  _inertial_frame(),
  _road_frame(other._road_frame),
  _chassis_frame(other._chassis_frame),
  _mass_kg(other._mass_kg),
  _I_kgm2(other._I_kgm2),
  _rho(other._rho),
  _cd(other._cd),
  _cl(other._cl),
  _A(other._A),
  _northward_wind(other._northward_wind),
  _eastward_wind(other._eastward_wind),
  _front_axle(other._front_axle),
  _rear_axle(other._rear_axle),
  _com_velocity_x_mps(other._com_velocity_x_mps),
  _com_velocity_x_dot_mps2(other._com_velocity_x_dot_mps2),
  _com_velocity_y_mps(other._com_velocity_y_mps),
  _com_velocity_y_dot_mps2(other._com_velocity_y_dot_mps2),
  _yaw_rate_dot_radps2(other._yaw_rate_dot_radps2),
  _total_force_N(other._total_force_N),
  _total_torque_Nm(other._total_torque_Nm)

{
    _road_frame.set_parent(_inertial_frame);

    // Update parent of chassis frame to new road frame
    _chassis_frame.set_parent(_road_frame);

    // Set the front and rear axles new frames parents
    _front_axle.get_frame().set_parent(_chassis_frame); 
    _rear_axle.get_frame().set_parent(_chassis_frame); 
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>& Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::operator=(const Chassis& other)
{
    __used_parameters   = other.__used_parameters;
    _velocity_x_mps      = other._velocity_x_mps;
    _velocity_y_mps      = other._velocity_y_mps;
    _com_velocity_x_mps = other._com_velocity_x_mps;
    _com_velocity_y_mps = other._com_velocity_y_mps;
    _yaw_rate_radps      = other._yaw_rate_radps;
    _inertial_frame     = Frame<Timeseries_t>();
    _road_frame         = other._road_frame;
    _chassis_frame      = other._chassis_frame;
    _mass_kg            = other._mass_kg;
    _I_kgm2             = other._I_kgm2;
    _rho                = other._rho;
    _cd                 = other._cd;
    _cl                 = other._cl;
    _A                  = other._A;
    _northward_wind     = other._northward_wind;
    _eastward_wind      = other._eastward_wind;
    _front_axle         = other._front_axle;
    _rear_axle          = other._rear_axle;
    _com_velocity_x_mps      = other._com_velocity_x_mps;
    _com_velocity_x_dot_mps2 = other._com_velocity_x_dot_mps2;
    _com_velocity_y_mps      = other._com_velocity_y_mps;
    _com_velocity_y_dot_mps2 = other._com_velocity_y_dot_mps2;
    _yaw_rate_dot_radps2     = other._yaw_rate_dot_radps2;
    _total_force_N           = other._total_force_N;
    _total_torque_Nm         = other._total_torque_Nm;

    // Update parent of road frame to new inertial frame
    _road_frame.set_parent(_inertial_frame);

    // Update parent of chassis frame to new road frame
    _chassis_frame.set_parent(_road_frame);

    // Set the front and rear axles new frames parents
    _front_axle.get_frame().set_parent(_chassis_frame); 
    _rear_axle.get_frame().set_parent(_chassis_frame); 

    return *this;
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::update(const Vector3d<Timeseries_t>& ground_position_vector_m,
    const Euler_angles<scalar>& road_euler_angles_rad, const Timeseries_t& track_heading_angle_rad, const Euler_angles<Timeseries_t>& road_euler_angles_dot_radps,
    const Timeseries_t& track_heading_angle_dot_radps, const Timeseries_t& ground_velocity_z_body_mps)
{
    // Set the new position of the frame origin
    _road_frame.set_origin(ground_position_vector_m, {_velocity_x_mps, _velocity_y_mps, ground_velocity_z_body_mps}, decltype(_road_frame)::Frame_velocity_types::this_frame);

    // Set the new orientation of the frame (update only in the last call)
    _road_frame.set_rotation_angle(0, road_euler_angles_rad.yaw(),   road_euler_angles_dot_radps.yaw(),   false);
    _road_frame.set_rotation_angle(1, road_euler_angles_rad.pitch(), road_euler_angles_dot_radps.pitch(), false);
    _road_frame.set_rotation_angle(2, road_euler_angles_rad.roll(),  road_euler_angles_dot_radps.roll(),  false);
    _road_frame.set_rotation_angle(3, track_heading_angle_rad,       track_heading_angle_dot_radps,       true);
}



template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<typename T>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_parameter(const std::string& parameter, const T value)
{
    // Check if the parameter goes to this object
    if ( parameter.find("vehicle/chassis/") == 0 )
    {
        // Find the parameter in the database
        auto found = ::set_parameter(get_parameters(), __used_parameters, parameter, "vehicle/chassis/", value); 

        // If not found, look for it in the class extra parameters
        if (!found)
        {
            const auto extra_parameters_map = get_extra_parameters_map();
            const auto it_extra_parameter = extra_parameters_map.find(parameter);

            if (it_extra_parameter != extra_parameters_map.cend())
            {
                *(it_extra_parameter->second) = value;
                found = true;
            }
        }

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


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::fill_xml(Xml_document& doc) const
{
    // Write the parameters of this class
    ::write_parameters(doc, "vehicle/chassis/", get_parameters());

    // Write the parameters of the axles
    _front_axle.fill_xml(doc);
    _rear_axle.fill_xml(doc); 
}



template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
inline void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::transform_states_to_inputs
    (const std::array<Timeseries_t,number_of_inputs>& states, const std::array<Timeseries_t,number_of_controls>& controls, std::array<Timeseries_t,number_of_inputs>& inputs)
{
    const auto& u     = states[state_names::com_velocity_x_mps];
    const auto& v     = states[state_names::com_velocity_y_mps];
    const auto& omega = states[state_names::yaw_rate_radps];

    // (1) Update the frames, the tires will want them updated
    _road_frame.set_origin({0.0,0.0,0.0},{u,v,0.0}, decltype(_road_frame)::Frame_velocity_types::this_frame);
    _road_frame.set_rotation_angle(3,0.0,omega);

    // (2) Transform
    get_front_axle().transform_states_to_inputs(states, controls, inputs);
    get_rear_axle().transform_states_to_inputs(states, controls, inputs);
    
    // Update input states, just as a placeholder if I ever switch to u=vtot.cos(beta), v=vtot.sin(beta)
    inputs[input_names::velocity_x_mps] = u; 
    inputs[input_names::velocity_y_mps] = v; 
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state(Timeseries_t u, Timeseries_t v, Timeseries_t omega)
{
    // u
    _velocity_x_mps = u;

    // v
    _velocity_y_mps = v;

    // omega
    _yaw_rate_radps = omega;

}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Timeseries_t Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::get_longitudinal_acceleration() const
{
    const Vector3d<Timeseries_t> velocity = {get_u(), get_v(), 0.0};
    const Vector3d<Timeseries_t> acceleration = _total_force_N / _mass_kg;

    return dot(velocity,acceleration)/norm(velocity);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Timeseries_t Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::get_lateral_acceleration() const
{
    const Vector3d<Timeseries_t> velocity = {get_u(), get_v(), 0.0};
    const Vector3d<Timeseries_t> acceleration = _total_force_N / _mass_kg;

    return cross(velocity,acceleration).z()/norm(velocity);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline const Vector3d<Timeseries_t> Chassis<Timeseries_t, FrontAxle_t, RearAxle_t, state_start,control_start>::get_gravity_force() const
{
    // Get Euler angles
    const auto& pitch               = _road_frame.get_rotation_angles()[1];
    const auto& roll                = _road_frame.get_rotation_angles()[2];
    const auto& track_heading_angle = _road_frame.get_rotation_angles()[3];

    return {
        _mass_kg * g0 * (sin(track_heading_angle) * sin(roll) * cos(pitch) - cos(track_heading_angle) * sin(pitch)),
        _mass_kg * g0 * (sin(track_heading_angle) * sin(pitch) + cos(track_heading_angle) * sin(roll) * cos(pitch)),
        _mass_kg * g0 * cos(roll) * cos(pitch)
    };
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline auto Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::get_aerodynamic_force() const -> Aerodynamic_forces
{
    // (1) Get car's velocity in body frame
    const Vector3d<Timeseries_t> car_velocity = _road_frame.get_absolute_velocity_in_body();

    // (2) Get wind velocity in body frame: the minus sign accounts for the Z
    const Vector3d<Timeseries_t> wind_velocity_absolute = { _eastward_wind, -_northward_wind, 0.0 };
    const Vector3d<Timeseries_t> wind_velocity          = transpose(_road_frame.get_absolute_rotation_matrix()) * wind_velocity_absolute;

    // (3) Get aerodynamic velocity in body frame
    const Vector3d<Timeseries_t> aerodynamic_velocity = -car_velocity + wind_velocity;
    const Timeseries_t aerodynamic_velocity_norm_squared = aerodynamic_velocity.x() * aerodynamic_velocity.x() + aerodynamic_velocity.y() * aerodynamic_velocity.y();
    const Timeseries_t aerodynamic_velocity_norm         = sqrt(aerodynamic_velocity_norm_squared);

    const Vector3d<Timeseries_t> drag_force = 0.5 * _rho * _cd * _A * aerodynamic_velocity * aerodynamic_velocity_norm;
    const Vector3d<Timeseries_t> lift_force = { 0.0, 0.0, 0.5 * _rho * _cl * _A * aerodynamic_velocity.x()*aerodynamic_velocity.x()};

    return Aerodynamic_forces
    {
        .lift = lift_force,
        .drag = drag_force,
        .wind_velocity_body = wind_velocity,
        .aerodynamic_velocity   = aerodynamic_velocity
    };
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Timeseries_t Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::get_understeer_oversteer_indicator() const
{
 if constexpr(FrontAxle_t::NTIRES == 2)
 {
    // (1) Get ideal curvature: kappa = omega/vtot         
    const auto kappa = _yaw_rate_radps/sqrt(_velocity_x_mps*_velocity_x_mps + _velocity_y_mps*_velocity_y_mps);

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

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_controls
    (const std::array<Timeseries_t,number_of_inputs>& inputs, const std::array<Timeseries_t,number_of_controls>& controls)
{
    get_front_axle().set_state_and_controls(inputs, controls);
    get_rear_axle().set_state_and_controls(inputs, controls);

    // u
    _velocity_x_mps = inputs[input_names::velocity_x_mps];

    // v
    _velocity_y_mps = inputs[input_names::velocity_y_mps];

    // omega
    _yaw_rate_radps = inputs[input_names::yaw_rate_radps];
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_control_names
    (std::array<std::string,number_of_inputs>& inputs, std::array<std::string,number_of_controls>& controls) const 
{
    _front_axle.set_state_and_control_names(inputs,controls);
    _rear_axle.set_state_and_control_names(inputs,controls);

    // u
    inputs[input_names::velocity_x_mps] = get_name() + ".velocity.x";

    // v
    inputs[input_names::velocity_y_mps] = get_name() + ".velocity.y";

    // omega
    inputs[input_names::yaw_rate_radps] = get_name() + ".omega.z";
}



template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_states>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::get_state_and_state_derivative
    (std::array<Timeseries_t,number_of_states>& state, std::array<Timeseries_t, number_of_states>& dstate_dt) const
{
    get_front_axle().get_state_and_state_derivative(state, dstate_dt, _mass_kg);
    get_rear_axle().get_state_and_state_derivative(state, dstate_dt, _mass_kg);

    // dudt
    state    [state_names::com_velocity_x_mps] = _com_velocity_x_mps;
    dstate_dt[state_names::com_velocity_x_mps] = _com_velocity_x_dot_mps2;

    // dvdt
    state    [state_names::com_velocity_y_mps] = _com_velocity_y_mps;
    dstate_dt[state_names::com_velocity_y_mps] = _com_velocity_y_dot_mps2;

    // domega
    state    [state_names::yaw_rate_radps] = _yaw_rate_radps;
    dstate_dt[state_names::yaw_rate_radps] = _yaw_rate_dot_radps2;
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, number_of_inputs>& input_states_def, std::array<scalar, number_of_inputs>& input_states_lb, 
     std::array<scalar, number_of_inputs>& input_states_ub, std::array<scalar , number_of_controls>& controls_def, 
     std::array<scalar, number_of_controls>& controls_lb, std::array<scalar, number_of_controls>& controls_ub,
    scalar velocity_x_lb, scalar velocity_x_ub, scalar velocity_y_ub, scalar omega_ub ) const
{
    get_front_axle().set_state_and_control_upper_lower_and_default_values(input_states_def, input_states_lb, input_states_ub,
        controls_def, controls_lb, controls_ub);
    get_rear_axle().set_state_and_control_upper_lower_and_default_values(input_states_def, input_states_lb, input_states_ub,
        controls_def, controls_lb, controls_ub);

    // State -----
    input_states_def[input_names::velocity_x_mps] = velocity_x_lb;
    input_states_lb [input_names::velocity_x_mps] = velocity_x_lb;
    input_states_ub [input_names::velocity_x_mps] = velocity_x_ub;

    // v
    input_states_def[input_names::velocity_y_mps] = 0.0;
    input_states_lb [input_names::velocity_y_mps] = -velocity_y_ub;
    input_states_ub [input_names::velocity_y_mps] =  velocity_y_ub;

    // omega
    input_states_def[input_names::yaw_rate_radps] = 0.0;
    input_states_lb [input_names::yaw_rate_radps] = -omega_ub;
    input_states_ub [input_names::yaw_rate_radps] =  omega_ub;
}

#endif
