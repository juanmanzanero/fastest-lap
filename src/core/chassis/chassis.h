#ifndef CHASSIS_H
#define CHASSIS_H

#include "lion/math/matrix3x3.h"
#include "lion/frame/frame.h"
#include "lion/math/euler_angles.h"
#include <map>
#include <unordered_map>
#include "lion/io/database_parameters.h"
#include "lion/io/Xml_document.h"

//!   Generic chassis type
//!   --------------------
//!
//! This class stores and provides interface with the front and rear axles
//!  @param FrontAxle_t: type of the front axle
//!  @param RearAxle_t: type of the rear axle
//!  @param state_start: index of the first state variable defined here
//!  @param control_start: index of the first control variable defined here
template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
class Chassis
{
 public:
    using Timeseries_type = Timeseries_t;

    //! Type of the front axle
    typedef FrontAxle_t front_axle_type;

    //! Type of the rear axle
    typedef RearAxle_t rear_axle_type;

    struct input_names
    {
        enum
        {
            velocity_x_mps = state_start,   //! Longitudinal velocity (in road frame) [m/s]
            velocity_y_mps,                 //! Lateral velocity (in road frame) [m/s]
            yaw_rate_radps,                 //! Yaw rate [rad/s]
            end
        };
    };

    //! State variables: the basic 3 dof - longitudinal/lateral velocity + yaw
    struct state_names
    {
        enum
        {
            com_velocity_x_mps = state_start,
            com_velocity_y_mps,
            yaw_rate_radps,
            end
        };
    };

    //! Control variables: none
    struct control_names
    {
        enum { end = control_start };
    };

    static_assert(input_names::end == state_names::end);

    //! Default constructor
    Chassis() = default;

    //! Empty constructor
    Chassis(const FrontAxle_t& front_axle, 
            const RearAxle_t& rear_axle,
            const std::string& path=""
           );

    //! Constructor from axles and mechanical/geometrical parameters
    //! @param[in] front_axle: Front axle object
    //! @param[in] rear_axle: Rear axle object
    //! @param[in] parameters: map with mechanical/geometrical parameters by name
    //! @param[in] path: path to the parameters of the chassis in the map
    Chassis(const FrontAxle_t& front_axle, 
            const RearAxle_t& rear_axle,
            Xml_document& database,
            const std::string& path=""
           );

    //! Copy constructor
    Chassis(const Chassis& other);

    //! Copy assignment
    Chassis& operator=(const Chassis& other);

    //! Perform an update of the class: move the road frame to the new position
    //! @param[in] x: x-position of the road frame [m]
    //! @param[in] y: y-position of the road frame [m]
    //! @param[in] psi: yaw angle of the road frame [rad]
    void update(const Vector3d<Timeseries_t>& ground_position_vector_m,
                const Euler_angles<scalar>& road_euler_angles_rad,
                const Timeseries_t& track_heading_angle_rad,
                const Euler_angles<Timeseries_t>& road_euler_angles_dot_radps,
                const Timeseries_t& track_heading_angle_dot_radps,
                const Timeseries_t& ground_velocity_z_body_mps);

    //! Modifyer to set a parameter
    template<typename T>
    void set_parameter(const std::string& parameter, const T value);

    //! Fill the corresponding nodes of an xml document
    void fill_xml(Xml_document& doc) const;

    template<size_t number_of_inputs, size_t number_of_controls>
    void transform_states_to_inputs(const std::array<Timeseries_t,number_of_inputs>& states,
                                          const std::array<Timeseries_t,number_of_controls>& controls,
                                          std::array<Timeseries_t,number_of_inputs>& input_states);

    //! Set state: longitudinal/lateral velocities and yaw speed
    //! @param[in] u: longitudinal velocity in road frame [m/s]
    //! @param[in] v: lateral velocity in road frame [m/s]
    //! @param[in] omega: yaw speed [rad/s]
    void set_state(Timeseries_t u, Timeseries_t v, Timeseries_t omega);

    //! Get the longitudinal velocity [m/s]
    const Timeseries_t& get_u() const { return _velocity_x_mps; }

    //! Get the lateral velocity [m/s]
    const Timeseries_t& get_v() const { return _velocity_y_mps; }

    //! Get the longitudinal acceleration [m/s2]
    Timeseries_t get_longitudinal_acceleration() const;

    //! Get the lateral acceleration [m/s2]
    Timeseries_t get_lateral_acceleration() const;

    constexpr const Timeseries_t& get_com_velocity_x_dot_mps2() const { return _com_velocity_x_dot_mps2; }

    constexpr const Timeseries_t& get_com_velocity_y_dot_mps2() const { return _com_velocity_y_dot_mps2; }

    //! Get the yaw speed [rad/s]
    const Timeseries_t& get_yaw_rate_radps() const { return _yaw_rate_radps; }

    //! Get the yaw acceleration [rad/s2]
    const Timeseries_t& get_yaw_rate_dot_radps2() const { return _yaw_rate_dot_radps2; }

    //! Get the chassis mass [kg]
    constexpr const Timeseries_t& get_mass() const { return _mass_kg; } 

    //! Get the chassis inertia matrix [kg.m2]
    constexpr const sMatrix3x3& get_inertia() const { return _I_kgm2; }

    //! Get the front axle
    constexpr const FrontAxle_t& get_front_axle() const { return _front_axle; }
    constexpr       FrontAxle_t& get_front_axle()       { return _front_axle; }

    //! Get the rear axle
    constexpr const RearAxle_t& get_rear_axle() const { return _rear_axle; }
    constexpr       RearAxle_t& get_rear_axle()       { return _rear_axle; }

    //! Get a reference to the inertial frame
    constexpr const Frame<Timeseries_t>& get_inertial_frame() const { return _inertial_frame; }

    //! Get a reference to the road frame
    constexpr const Frame<Timeseries_t>& get_road_frame() const { return _road_frame; }

    //! Get a reference to the chassis frame
    constexpr const Frame<Timeseries_t>& get_chassis_frame() const { return _chassis_frame; }

    //! Get the total force acting on the chassis CoM [N]
    constexpr const Vector3d<Timeseries_t>& get_force() const { return _total_force_N; }

    //! Get the total torque acting on the chassis CoM [Nm]
    constexpr const Vector3d<Timeseries_t>& get_torque() const { return _total_torque_Nm; }

    //! Compute the gravity force
    const Vector3d<Timeseries_t> get_gravity_force() const;

    //! Get the aerodynamic force [N]
    struct Aerodynamic_forces
    {
        Vector3d<Timeseries_t> lift;
        Vector3d<Timeseries_t> drag;
        Vector3d<Timeseries_t> wind_velocity_body;
        Vector3d<Timeseries_t> aerodynamic_velocity;
    };

    auto get_aerodynamic_force() const -> Aerodynamic_forces; 

    //! Get understeer(<0) or oversteer(>0) indicator
    Timeseries_t get_understeer_oversteer_indicator() const;

    //! Get the drag coefficient
    const Timeseries_t& get_drag_coefficient() const { return _cd; }

    //! Load the time derivative of the state variables computed herein to the dqdt
    //! @param[out] dqdt: the vehicle state vector time derivative
    template<size_t number_of_states>
    void get_state_and_state_derivative(std::array<Timeseries_t,number_of_states>& state, 
                                        std::array<Timeseries_t, number_of_states>& dstate_dt
                                        ) const;

    //! Set the state variables of this class
    //! @param[in] q: the vehicle state vector 
    //! @param[in] u: the vehicle control vector
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_controls(const std::array<Timeseries_t,number_of_inputs>& input_states, 
                                const std::array<Timeseries_t,number_of_controls>& controls);

    //! Set the state and controls upper, lower, and default values
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_upper_lower_and_default_values(std::array<scalar,number_of_inputs>& input_states_def,
                                                               std::array<scalar,number_of_inputs>& input_states_lb,
                                                               std::array<scalar,number_of_inputs>& input_states_ub,
                                                               std::array<scalar,number_of_controls>& controls_def,
                                                               std::array<scalar,number_of_controls>& controls_lb,
                                                               std::array<scalar,number_of_controls>& controls_ub,
                                                               scalar velocity_x_lb, 
                                                               scalar velocity_x_ub, 
                                                               scalar velocity_y_ub, 
                                                               scalar omega_ub
                                                              ) const;


    //! Get the names of the state and control varaibles of this class
    //! @param[out] q: the vehicle state names
    //! @param[out] u: the vehicle control names
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_names(std::array<std::string,number_of_inputs>& input_states, 
                                     std::array<std::string,number_of_controls>& controls) const;


    bool is_ready() const { return _front_axle.is_ready() && _rear_axle.is_ready() && 
        std::all_of(__used_parameters.begin(), __used_parameters.end(), [](const auto& v) -> auto { return v; }); }

    static std::string type() { return "chassis"; }

    static std::string get_name() { return "chassis"; }


    std::unordered_map<std::string,Timeseries_t> get_outputs_map() const
    {
        auto map = get_outputs_map_self();
        const auto front_axle_map = _front_axle.get_outputs_map();
        const auto rear_axle_map = _rear_axle.get_outputs_map();

        map.insert(front_axle_map.cbegin(), front_axle_map.cend());
        map.insert(rear_axle_map.cbegin(), rear_axle_map.cend());

        return map;
    }

 private:
    DECLARE_PARAMS(
        { "mass", _mass_kg },
        { "inertia/Ixx", _I_kgm2.xx() },
        { "inertia/Ixy", _I_kgm2.xy() },
        { "inertia/Ixz", _I_kgm2.xz() },
        { "inertia/Iyx", _I_kgm2.yx() },
        { "inertia/Iyy", _I_kgm2.yy() },
        { "inertia/Iyz", _I_kgm2.yz() },
        { "inertia/Izx", _I_kgm2.zx() },
        { "inertia/Izy", _I_kgm2.zy() },
        { "inertia/Izz", _I_kgm2.zz() },
        { "aerodynamics/rho", _rho },
        { "aerodynamics/cd", _cd },
        { "aerodynamics/cl", _cl },
        { "aerodynamics/area", _A }
    ); 

    std::unordered_map<std::string, Timeseries_t*> get_extra_parameters_map()
    {
        return
        {
            {"vehicle/chassis/aerodynamics/wind_velocity/northward", &_northward_wind},
            {"vehicle/chassis/aerodynamics/wind_velocity/eastward", &_eastward_wind}
        };
    }


    std::unordered_map<std::string,Timeseries_t> get_outputs_map_self() const
    {
        return
        {
            {get_name() + ".acceleration.x", get_longitudinal_acceleration()},
            {get_name() + ".acceleration.y", get_lateral_acceleration()},
            {get_name() + ".force.x", _total_force_N.x()},
            {get_name() + ".force.y", _total_force_N.y()},
            {get_name() + ".force.z", _total_force_N.z()},
            {get_name() + ".torque.x", _total_torque_Nm.x()},
            {get_name() + ".torque.y", _total_torque_Nm.y()},
            {get_name() + ".torque.z", _total_torque_Nm.z()},
            {get_name() + ".acceleration.yaw", _yaw_rate_dot_radps2},
            {get_name() + ".position.x", _road_frame.get_origin().x()},
            {get_name() + ".position.y", _road_frame.get_origin().y()},
            {get_name() + ".attitude.yaw", _road_frame.get_rotation_angles().front()},
            {get_name() + ".understeer_oversteer_indicator", get_understeer_oversteer_indicator()},
            {get_name() + ".aerodynamics.cd", _cd},
            {get_name() + ".aerodynamics.lift", get_aerodynamic_force().lift.z()},
            {get_name() + ".aerodynamics.drag", norm(get_aerodynamic_force().drag)},
            {get_name() + ".aerodynamics.drag.x", get_aerodynamic_force().drag.x()},
            {get_name() + ".aerodynamics.drag.y", get_aerodynamic_force().drag.y()},
            {get_name() + ".aerodynamics.wind_velocity_body.x", get_aerodynamic_force().wind_velocity_body.x()},
            {get_name() + ".aerodynamics.wind_velocity_body.y", get_aerodynamic_force().wind_velocity_body.y()},
            {get_name() + ".aerodynamics.aerodynamic_velocity.x", get_aerodynamic_force().aerodynamic_velocity.x()},
            {get_name() + ".aerodynamics.aerodynamic_velocity.y", get_aerodynamic_force().aerodynamic_velocity.y()}
        };
    };


    // Road frame velocities
    Timeseries_t _velocity_x_mps = 0.0;         //! [in] Longitudinal velocity of the road CoM shadow (in road frame) [m/s]
    Timeseries_t _velocity_y_mps = 0.0;         //! [in] Lateral velocity of the road CoM shadow (in road frame) [m/s]

    Timeseries_t _yaw_rate_radps = 0.0;     //! [in] Yaw speed [rad/s]

    Frame<Timeseries_t> _inertial_frame; //! The inertial frame
    Frame<Timeseries_t> _road_frame;     //! Frame<Timeseries_t> with center on the road projection of the CoG, 
                                         //! aligned with the chassis
    Frame<Timeseries_t> _chassis_frame;  //! Frame<Timeseries_t> with center on the CoG and parallel axes to the road frame

    // Mass properties
    Timeseries_t _mass_kg = 0.0;   //! [c] chassis mass [kg]
    sMatrix3x3 _I_kgm2 = {};      //! [c] chassis inertia matrix [kg.m2]

    // Aerodynamic properties
    scalar _rho      = 0.0;                //! [c] air density [kg/m3]
    Timeseries_t _cd = 0.0;                //! [c] drag coefficient [-]      
    Timeseries_t _cl = 0.0;                //! [c] lift coefficient [-]      
    scalar _A        = 0.0;                //! [c] frontal area [m2]
    Timeseries_t _northward_wind = 0.0;    //! [c] northward wind velocity [m/s]
    Timeseries_t _eastward_wind  = 0.0;    //! [c] eastward wind velocity [m/s]

    FrontAxle_t _front_axle; //! Front axle
    RearAxle_t  _rear_axle;  //! Rear axle

 protected:

    // Non-const version of getters
    constexpr Frame<Timeseries_t>& get_inertial_frame() { return _inertial_frame; }
    constexpr Frame<Timeseries_t>& get_road_frame()     { return _road_frame; }
    constexpr Frame<Timeseries_t>& get_chassis_frame()  { return _chassis_frame; }


    // Longitudinal dynamics
    Timeseries_t _com_velocity_x_mps = 0.0; //! [out] Longitudinal velocity of the center of mass [m/s]
    Timeseries_t _com_velocity_x_dot_mps2 = 0.0;    //! [out] Longitudinal acceleration [m/s2]
    
    // Lateral dynamics
    Timeseries_t _com_velocity_y_mps = 0.0; //! [out] Lateral velocity of the center of mass [m/s]
    Timeseries_t _com_velocity_y_dot_mps2 = 0.0;    //! [out] Lateral acceleration [m/s2]

    // Attitude: yaw
    Timeseries_t _yaw_rate_dot_radps2 = 0.0; //! [out] Yaw acceleration [rad/s2]

    Vector3d<Timeseries_t> _total_force_N;   //! [out] Total forces at road frame
    Vector3d<Timeseries_t> _total_torque_Nm; //! [out] Total torque at road frame


};

#include "chassis.hpp"

#endif
