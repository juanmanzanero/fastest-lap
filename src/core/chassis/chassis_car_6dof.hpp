#ifndef CHASSIS_CAR_6DOF_HPP
#define CHASSIS_CAR_6DOF_HPP

#include "src/core/foundation/fastest_lap_exception.h"

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis_car_6dof()
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
    base_type::get_chassis_frame().set_origin(get_com_position(), get_com_velocity()); 

    // Set axles frame position and velocity
    base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity());
    base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity());
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis_car_6dof(const FrontAxle_t& front_axle, 
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
    base_type::get_chassis_frame().set_origin(get_com_position(), get_com_velocity()); 

    // Set axles frame position and velocity
    base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity());
    base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity());
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis_car_6dof(Xml_document& database)
: Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>(
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


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state
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


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::update(const Vector3d<Timeseries_t>& ground_position_vector_m,
    const Euler_angles<scalar>& road_euler_angles_rad, const Timeseries_t& track_heading_angle_rad, const Euler_angles<Timeseries_t>& road_euler_angles_dot_radps,
    const Timeseries_t& track_heading_angle_dot_radps, const Timeseries_t& ground_velocity_z_body_mps)
{
    // Update base_type
    base_type::update(ground_position_vector_m.x(), ground_position_vector_m.y(), road_euler_angles_rad.yaw() + track_heading_angle_rad);

    FrontAxle_t& front_axle = base_type::get_front_axle();
    RearAxle_t& rear_axle = base_type::get_rear_axle();
    
    Frame<Timeseries_t>& road_frame = base_type::get_road_frame();

    const Timeseries_t& m = base_type::get_mass();

    // Update frame
    base_type::get_chassis_frame().set_origin(_x_com + Vector3d<Timeseries_t>(0.0, 0.0, _z), {0.0, 0.0, _dz});

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

    base_type::_F = F_front + F_rear;

    base_type::_F[Z] += base_type::get_mass()*g0;

    base_type::_T =   T_front + cross(x_front, F_front)
                    + T_rear + cross(x_rear, F_rear);

    // Aerodynamic drag
    const auto aerodynamic_forces = base_type::get_aerodynamic_force();
    const auto F_aero = aerodynamic_forces.lift + aerodynamic_forces.drag;
    base_type::_F += F_aero;
    base_type::_T += cross(base_type::get_chassis_frame().get_origin(), F_aero);

    // Add the external forces
    base_type::_F += base_type::_Fext;

    base_type::_T += base_type::_Text;

    // Newton equations
    const Vector3d<Timeseries_t> dvdt = -Newton_lhs() + base_type::_F/m;

    base_type::_du = dvdt[X];
    base_type::_dv = dvdt[Y];
    _d2z = dvdt[Z];

    // Euler equations on road contact point
    const Vector3d<Timeseries_t> d2phi = linsolve(Euler_m(), -Euler_lhs() + base_type::_T);

    _d2phi = d2phi[X];
    _d2mu  = d2phi[Y];
    base_type::_dOmega = d2phi[Z];
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Vector3d<Timeseries_t> Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Newton_lhs() const
{
    const Frame<Timeseries_t>& road_frame = base_type::get_road_frame();
    const Vector3d<Timeseries_t> omega = road_frame.get_omega_absolute_in_body();
    const Vector3d<Timeseries_t> vel = road_frame.get_absolute_velocity_in_body(); 

    return cross(omega,vel);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Vector3d<Timeseries_t> Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Euler_lhs() const
{
    const Frame<Timeseries_t>& road_frame = base_type::get_road_frame();
    const Timeseries_t& omega = road_frame.get_rotation_angles_derivative()[0];
    const Vector3d<Timeseries_t> vel = road_frame.get_absolute_velocity_in_body(); 

    const Timeseries_t& u = vel[X];
    const Timeseries_t& v = vel[Y];

    const Timeseries_t& m = base_type::get_mass();
    const sMatrix3x3& I = base_type::get_inertia();
    const scalar h = -_x_com[2];

    const Timeseries_t& du = base_type::_du;
    const Timeseries_t& dv = base_type::_dv;
  
    return { m*(dv*h + (h - _z)*omega*u) - (I.yy()-I.zz()+I.xx())*omega*_dmu 
             - (I.yy() - I.zz())*omega*omega*_phi , 
             m*((_z - h)*du + h*omega*v) + (I.yy()-I.zz()+I.xx())*omega*_dphi
             - ((I.xx() - I.zz())*_mu + I.xz())*omega*omega,
             2.0*I.zx()*_dmu*omega
           };
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Matrix3x3<Timeseries_t> Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Euler_m() const
{
    const sMatrix3x3& I = base_type::get_inertia();

    return { I.xx(), 0.0, -(I.xx() - I.zz())*_mu - I.xz(),
             0.0, I.yy(), (I.yy() - I.zz())*_phi,
            -I.xz(), 0.0, I.zz() + 2.0*I.xz()*_mu };
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
scalar Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_parameter(const std::string& parameter_name) const
{
     if (parameter_name == "cog_height") return -_x_com[2];

     if (parameter_name == "front_axle_x") return _x_front_axle[0];

     if (parameter_name == "rear_axle_x") return _x_rear_axle[0];

     throw fastest_lap_exception("parameter " + parameter_name + " does not exist in Chassis_Car");
}


// ------- Handle state vector
template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t N>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_state_and_state_derivative
    (std::array<Timeseries_t,N>& state, std::array<Timeseries_t, N>& dstate_dt) const
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


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_names(std::array<std::string, NSTATE>& input_states,
    std::array<std::string, NALGEBRAIC>& algebraic_states, std::array<std::string, NCONTROL>& controls) const
{
    base_type::set_state_and_control_names(input_states,controls);

    // z
    input_states[input_state_names::Z] = "chassis.position.z";

    // phi
    input_states[input_state_names::PHI] = "chassis.attitude.phi";

    // mu
    input_states[input_state_names::MU] = "chassis.attitude.mu";

    // 2nd order

    // dzdt
    input_states[input_state_names::DZDT] = "chassis.velocity.z";

    // dphidt
    input_states[input_state_names::DPHIDT] = "chassis.omega.x";

    // dmudt
    input_states[input_state_names::DMUDT] = "chassis.omega.y";
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_controls
    (const std::array<Timeseries_t,NSTATE>& input_states, 
     const std::array<Timeseries_t,NALGEBRAIC>& algebraic_states, 
     const std::array<Timeseries_t,NCONTROL>& controls)
{
    base_type::set_state_and_controls(input_states,controls);

    // State ---

    // 1st order

    // z
    _z = input_states[input_state_names::Z];

    // phi
    _phi = input_states[input_state_names::PHI];

    // mu
    _mu = input_states[input_state_names::MU];

    // 2nd order

    // dz
    _dz = input_states[input_state_names::DZDT];

    // dphidt
    _dphi = input_states[input_state_names::DPHIDT];

    // dmudt
    _dmu = input_states[input_state_names::DMUDT];
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, NSTATE>& input_states_def, std::array<scalar, NSTATE>& input_states_lb,
        std::array<scalar, NSTATE>& input_states_ub, std::array<scalar, NALGEBRAIC>& algebraic_states_def,
        std::array<scalar, NALGEBRAIC>& algebraic_states_lb, std::array<scalar, NALGEBRAIC>& algebraic_states_ub,
        std::array<scalar, NCONTROL>& controls_def, std::array<scalar, NCONTROL>& controls_lb,
        std::array<scalar, NCONTROL>& controls_ub) const
{
    // Call function for the parent class
    base_type::set_state_and_control_upper_lower_and_default_values
        (input_states_def, input_states_lb, input_states_ub,
         controls_def, controls_lb, controls_ub,
         10.0 * KMH, 200.0 * KMH, 10.0 * KMH, 10.0);
    // State ---

    // 1st order

    // z
    input_states_def[input_state_names::Z] = 1.0e-5;
    input_states_lb[input_state_names::Z]  = 1.0e-5;
    input_states_ub[input_state_names::Z]  = 0.139;

    // phi
    input_states_def[input_state_names::PHI] = 0.0;
    input_states_lb[input_state_names::PHI]  = -30.0*DEG;
    input_states_ub[input_state_names::PHI]  =  30.0*DEG;

    // mu
    input_states_def[input_state_names::MU] = 0.0;
    input_states_lb[input_state_names::MU]  = -30.0*DEG;
    input_states_ub[input_state_names::MU]  =  30.0*DEG;

    // 2nd order

    // dz
    input_states_def[input_state_names::DZDT] = 0.0;
    input_states_lb[input_state_names::DZDT]  = -10.0;
    input_states_ub[input_state_names::DZDT]  =  10.0;

    // dphidt
    input_states_def[input_state_names::DPHIDT] = 0.0;
    input_states_lb[input_state_names::DPHIDT]  = -10.0;
    input_states_ub[input_state_names::DPHIDT]  =  10.0;

    // dmudt
    input_states_def[input_state_names::DMUDT] = 0.0;
    input_states_lb[input_state_names::DMUDT]  = -10.0;
    input_states_ub[input_state_names::DMUDT]  =  10.0;
}

#endif

