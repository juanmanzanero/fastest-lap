#ifndef CHASSIS_CAR_6DOF_HPP
#define CHASSIS_CAR_6DOF_HPP

#include "src/core/foundation/fastest_lap_exception.h"

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Chassis_car_6dof()
: base_type(FrontAxle_t("front-axle",
            typename FrontAxle_t::Tire_left_type("front-axle.left-tire", "vehicle/front-tire/"),
            typename FrontAxle_t::Tire_right_type("front-axle.right-tire", "vehicle/front-tire/"),
            "vehicle/front-axle/"),
            RearAxle_t("rear-axle", 
            typename RearAxle_t::Tire_left_type("rear-axle.left-tire", "vehicle/rear-tire/"),
            typename RearAxle_t::Tire_right_type("rear-axle.right-tire", "vehicle/rear-tire/"),
            "vehicle/rear-axle/"),
            "vehicle/chassis/") 
{
    // Chassis frame must have zero rotations
    if ( base_type::get_chassis_frame().get_rotation_angles().size() != 0 )
        throw fastest_lap_exception("Chassis frame must have cero rotations for Chassis_car_6dof");

    // Set chassis frame position and velocity (it does not matter before calling update())
    base_type::get_chassis_frame().set_origin(get_com_position(), get_com_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame); 

    // Set axles frame position and velocity
    base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
    base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Chassis_car_6dof(const FrontAxle_t& front_axle, 
                                                         const RearAxle_t& rear_axle,
                                                         Xml_document& database,
                                                         const std::string& path)
: base_type(front_axle, rear_axle, database, path),
_z(0.0), _dz(0.0), _d2z(0.0), 
_mu(0.0), _dmu(0.0), _d2mu(0.0), 
_phi(0.0), _dphi(0.0), _d2phi(0.0)
{
    read_parameters(database, path, get_parameters(), __used_parameters);
    // Chassis frame must have zero rotations
    if ( base_type::get_chassis_frame().get_rotation_angles().size() != 0 )
        throw fastest_lap_exception("Chassis frame must have cero rotations for Chassis_car_6dof");

    // Set chassis frame position and velocity (it does not matter before calling update())
    base_type::get_chassis_frame().set_origin(get_com_position(), get_com_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame); 

    // Set axles frame position and velocity
    base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
    base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Chassis_car_6dof(Xml_document& database)
: Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>(
           FrontAxle_t("front-axle",
                       typename FrontAxle_t::Tire_left_type("front-axle.left-tire", database, "vehicle/front-tire/"),
                       typename FrontAxle_t::Tire_right_type("front-axle.right-tire", database, "vehicle/front-tire/"),
                       database, "vehicle/front-axle/"),
           RearAxle_t("rear-axle", 
                      typename RearAxle_t::Tire_left_type("rear-axle.left-tire", database, "vehicle/rear-tire/"),
                      typename RearAxle_t::Tire_right_type("rear-axle.right-tire", database, "vehicle/rear-tire/"),
                      database, "vehicle/rear-axle/"),
           database, "vehicle/chassis/") 
{}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state
 (Timeseries_t u, Timeseries_t v, Timeseries_t omega, Timeseries_t z, Timeseries_t dz, Timeseries_t mu, Timeseries_t dmu, Timeseries_t phi, Timeseries_t dphi)
{
    base_type::set_state(u,v,omega);

    // Update CoG position
    _z    = z;
    _dz   = dz;

    // Update pitch attitude
    _mu   = mu;
    _dmu  = dmu;
    
    // Update roll attitude
    _phi  = phi;
    _dphi = dphi;
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::update(const Vector3d<Timeseries_t>& ground_position_vector_m,
    const Euler_angles<scalar>& road_euler_angles_rad, const Timeseries_t& track_heading_angle_rad, const Euler_angles<Timeseries_t>& road_euler_angles_dot_radps,
    const Timeseries_t& track_heading_angle_dot_radps, const Timeseries_t& ground_velocity_z_body_mps)
{
    // Update base_type
    base_type::update(ground_position_vector_m, road_euler_angles_rad, track_heading_angle_rad, road_euler_angles_dot_radps, track_heading_angle_dot_radps, ground_velocity_z_body_mps);

    auto& front_axle = base_type::get_front_axle();
    auto& rear_axle = base_type::get_rear_axle();
    
    auto& road_frame = base_type::get_road_frame();

    const auto& m = base_type::get_mass();

    // Update frame
    base_type::get_chassis_frame().set_origin(_x_com + Vector3d<Timeseries_t>(0.0, 0.0, _z), {0.0, 0.0, _dz}, Frame<Timeseries_t>::Frame_velocity_types::parent_frame);

    // Update axles
    base_type::get_front_axle().update(get_front_axle_position(),get_front_axle_velocity(), _phi, _dphi);
    base_type::get_rear_axle().update(get_rear_axle_position(), get_rear_axle_velocity(), _phi, _dphi);

    // Get the forces from the axles
    const Matrix3x3<Timeseries_t> Q_front = front_axle.get_frame().get_rotation_matrix(road_frame);
    const Matrix3x3<Timeseries_t> Q_rear = rear_axle.get_frame().get_rotation_matrix(road_frame);
    
    const Vector3d<Timeseries_t> x_front = std::get<0>(front_axle.get_frame().get_position_and_velocity_in_target(road_frame));
    const Vector3d<Timeseries_t> x_rear  = std::get<0>(rear_axle.get_frame().get_position_and_velocity_in_target(road_frame));

    const Vector3d<Timeseries_t> F_front = Q_front*front_axle.get_force();
    const Vector3d<Timeseries_t> F_rear  = Q_rear*rear_axle.get_force();

    const Vector3d<Timeseries_t> T_front = Q_front*front_axle.get_torque();
    const Vector3d<Timeseries_t> T_rear  = Q_rear* rear_axle.get_torque();

    base_type::_total_force_N = F_front + F_rear;

    base_type::_total_force_N[Z] += base_type::get_mass()*g0;

    base_type::_total_torque_Nm =   T_front + cross(x_front, F_front)
                    + T_rear + cross(x_rear, F_rear);

    // Aerodynamic drag
    const auto aerodynamic_forces = base_type::get_aerodynamic_force();
    const auto F_aero = aerodynamic_forces.lift + aerodynamic_forces.drag;
    base_type::_total_force_N += F_aero;
    base_type::_total_torque_Nm += cross(base_type::get_chassis_frame().get_origin(), F_aero);

    // Newton equations
    const Vector3d<Timeseries_t> dvdt = -Newton_lhs() + base_type::_total_force_N/m;

    base_type::_com_velocity_x_mps = base_type::get_u();
    base_type::_com_velocity_y_mps = base_type::get_v();

    base_type::_com_velocity_x_dot_mps2 = dvdt[X];
    base_type::_com_velocity_y_dot_mps2 = dvdt[Y];
    _d2z = dvdt[Z];

    // Euler equations on road contact point
    const Vector3d<Timeseries_t> d2phi = linsolve(Euler_m(), -Euler_lhs() + base_type::_total_torque_Nm);

    _d2phi = d2phi[X];
    _d2mu  = d2phi[Y];
    base_type::_yaw_rate_dot_radps2 = d2phi[Z];
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Vector3d<Timeseries_t> Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Newton_lhs() const
{
    const Frame<Timeseries_t>& road_frame = base_type::get_road_frame();
    const Vector3d<Timeseries_t> omega = road_frame.get_omega_absolute_in_body();
    const Vector3d<Timeseries_t> vel = road_frame.get_absolute_velocity_in_body(); 

    return cross(omega,vel);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Vector3d<Timeseries_t> Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Euler_lhs() const
{
    const Timeseries_t& omega = base_type::get_yaw_rate_radps();

    const Timeseries_t& u = base_type::get_u();
    const Timeseries_t& v = base_type::get_v();

    const Timeseries_t& m = base_type::get_mass();
    const sMatrix3x3& I = base_type::get_inertia();
    const scalar h = -_x_com[2];

    const Timeseries_t& du = base_type::_com_velocity_x_dot_mps2;
    const Timeseries_t& dv = base_type::_com_velocity_y_dot_mps2;
  
    return { m*(dv*h + (h - _z)*omega*u) - (I.yy()-I.zz()+I.xx())*omega*_dmu 
             - (I.yy() - I.zz())*omega*omega*_phi , 
             m*((_z - h)*du + h*omega*v) + (I.yy()-I.zz()+I.xx())*omega*_dphi
             - ((I.xx() - I.zz())*_mu + I.xz())*omega*omega,
             2.0*I.zx()*_dmu*omega
           };
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Matrix3x3<Timeseries_t> Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Euler_m() const
{
    const sMatrix3x3& I = base_type::get_inertia();

    return { I.xx(), 0.0, -(I.xx() - I.zz())*_mu - I.xz(),
             0.0, I.yy(), (I.yy() - I.zz())*_phi,
            -I.xz(), 0.0, I.zz() + 2.0*I.xz()*_mu };
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
scalar Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::get_parameter(const std::string& parameter_name) const
{
     if (parameter_name == "cog_height") return -_x_com[2];

     if (parameter_name == "front_axle_x") return _x_front_axle[0];

     if (parameter_name == "rear_axle_x") return _x_rear_axle[0];

     throw fastest_lap_exception("parameter " + parameter_name + " does not exist in Chassis_Car");
}


// ------- Handle state vector
template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_states>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::get_state_and_state_derivative
    (std::array<Timeseries_t,number_of_states>& state, std::array<Timeseries_t, number_of_states>& dstate_dt) const
{
    base_type::get_state_and_state_derivative(state,dstate_dt);
    // 1st order

    // dzdt
    state[state_names::Z]     = _z;
    dstate_dt[state_names::Z] = _dz;

    // dphidt
    state[state_names::PHI]     = _phi;
    dstate_dt[state_names::PHI] = _dphi;

    // dmudt
    state[state_names::MU]     = _mu;
    dstate_dt[state_names::MU] = _dmu;

    // 2nd order
    
    // d2zdt2
    state[state_names::DZDT]     = _dz;
    dstate_dt[state_names::DZDT] = _d2z;

    // d2phidt2
    state[state_names::DPHIDT]     = _dphi;
    dstate_dt[state_names::DPHIDT] = _d2phi;

    // d2mudt2
    state[state_names::DMUDT]     = _dmu;
    dstate_dt[state_names::DMUDT] = _d2mu;
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_control_names(std::array<std::string, number_of_inputs>& inputs,
    std::array<std::string, number_of_controls>& controls) const
{
    base_type::set_state_and_control_names(inputs,controls);

    // z
    inputs[input_names::Z] = "chassis.position.z";

    // phi
    inputs[input_names::PHI] = "chassis.attitude.phi";

    // mu
    inputs[input_names::MU] = "chassis.attitude.mu";

    // 2nd order

    // dzdt
    inputs[input_names::DZDT] = "chassis.velocity.z";

    // dphidt
    inputs[input_names::DPHIDT] = "chassis.omega.x";

    // dmudt
    inputs[input_names::DMUDT] = "chassis.omega.y";
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_controls
    (const std::array<Timeseries_t,number_of_inputs>& inputs, 
     const std::array<Timeseries_t,number_of_controls>& controls)
{
    base_type::set_state_and_controls(inputs,controls);

    // State ---

    // 1st order

    // z
    _z = inputs[input_names::Z];

    // phi
    _phi = inputs[input_names::PHI];

    // mu
    _mu = inputs[input_names::MU];

    // 2nd order

    // dz
    _dz = inputs[input_names::DZDT];

    // dphidt
    _dphi = inputs[input_names::DPHIDT];

    // dmudt
    _dmu = inputs[input_names::DMUDT];
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, number_of_inputs>& inputs_def, std::array<scalar, number_of_inputs>& inputs_lb,
        std::array<scalar, number_of_inputs>& inputs_ub,         
        std::array<scalar, number_of_controls>& controls_def, std::array<scalar, number_of_controls>& controls_lb,
        std::array<scalar, number_of_controls>& controls_ub) const
{
    // Call function for the parent class
    base_type::set_state_and_control_upper_lower_and_default_values
        (inputs_def, inputs_lb, inputs_ub,
         controls_def, controls_lb, controls_ub,
         10.0 * KMH, 200.0 * KMH, 10.0 * KMH, 10.0);
    // State ---

    // 1st order

    // z
    inputs_def[input_names::Z] = 1.0e-5;
    inputs_lb[input_names::Z]  = 1.0e-5;
    inputs_ub[input_names::Z]  = 0.139;

    // phi
    inputs_def[input_names::PHI] = 0.0;
    inputs_lb[input_names::PHI]  = -30.0*DEG;
    inputs_ub[input_names::PHI]  =  30.0*DEG;

    // mu
    inputs_def[input_names::MU] = 0.0;
    inputs_lb[input_names::MU]  = -30.0*DEG;
    inputs_ub[input_names::MU]  =  30.0*DEG;

    // 2nd order

    // dz
    inputs_def[input_names::DZDT] = 0.0;
    inputs_lb[input_names::DZDT]  = -10.0;
    inputs_ub[input_names::DZDT]  =  10.0;

    // dphidt
    inputs_def[input_names::DPHIDT] = 0.0;
    inputs_lb[input_names::DPHIDT]  = -10.0;
    inputs_ub[input_names::DPHIDT]  =  10.0;

    // dmudt
    inputs_def[input_names::DMUDT] = 0.0;
    inputs_lb[input_names::DMUDT]  = -10.0;
    inputs_ub[input_names::DMUDT]  =  10.0;
}

#endif

