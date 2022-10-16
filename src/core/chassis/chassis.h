#ifndef CHASSIS_H
#define CHASSIS_H

#include "lion/math/matrix3x3.h"
#include "lion/frame/frame.h"
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
//!  @param STATE0: index of the first state variable defined here
//!  @param CONTROL0: index of the first control variable defined here
template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
class Chassis
{
 public:
    using Timeseries_type = Timeseries_t;

    //! Type of the front axle
    typedef FrontAxle_t front_axle_type;

    //! Type of the rear axle
    typedef RearAxle_t rear_axle_type;

    //! State variables: the basic 3 dof - longitudinal/lateral velocity + yaw
    struct input_state_names
    {
        enum
        {
            U = STATE0,    //! Longitudinal velocity (in road frame) [m/s]
            V,             //! Lateral velocity (in road frame) [m/s]
            OMEGA,         //! Yaw speed [rad/s]
            end
        };
    };

    struct state_names
    {
        enum
        {
            U = input_state_names::U,
            V = input_state_names::V,
            OMEGA = input_state_names::OMEGA
        };
    };

    //! Control variables: none
    struct control_names
    {
        enum { end = CONTROL0 };
    };

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

    //! Modifyer to set a parameter
    template<typename T>
    void set_parameter(const std::string& parameter, const T value);

    //! Fill the corresponding nodes of an xml document
    void fill_xml(Xml_document& doc) const;

    template<size_t NSTATE, size_t NCONTROL>
    void transform_states_to_input_states(const std::array<Timeseries_t,NSTATE>& states,
                                          const std::array<Timeseries_t,NCONTROL>& controls,
                                          std::array<Timeseries_t,NSTATE>& input_states);

    //! Set state: longitudinal/lateral velocities and yaw speed
    //! @param[in] u: longitudinal velocity in road frame [m/s]
    //! @param[in] v: lateral velocity in road frame [m/s]
    //! @param[in] omega: yaw speed [rad/s]
    void set_state(Timeseries_t u, Timeseries_t v, Timeseries_t omega);

    //! Get the longitudinal velocity [m/s]
    const Timeseries_t& get_u() const { return _u; }

    //! Get the longitudinal acceleration in chassis frame [m/s2]
    const Timeseries_t& get_du() const { return _du; }

    //! Get the lateral velocity [m/s]
    const Timeseries_t& get_v() const { return _v; }

    //! Get the lateral acceleration in chassis frame [m/s2]
    const Timeseries_t& get_dv() const { return _dv; }

    //! Get the longitudinal acceleration [m/s2]
    Timeseries_t get_longitudinal_acceleration() const;

    //! Get the lateral acceleration [m/s2]
    Timeseries_t get_lateral_acceleration() const;

    //! Get the yaw speed [rad/s]
    const Timeseries_t& get_omega() const { return _omega; }

    //! Get the yaw acceleration [rad/s2]
    const Timeseries_t& get_domega() const { return _dOmega; }

    //! Get the chassis mass [kg]
    constexpr const Timeseries_t& get_mass() const { return _m; } 

    //! Get the chassis inertia matrix [kg.m2]
    constexpr const sMatrix3x3& get_inertia() const { return _I; }

    //! Get the front axle
    constexpr const FrontAxle_t& get_front_axle() const { return _front_axle; }
    constexpr       FrontAxle_t& get_front_axle()       { return _front_axle; }

    //! Get the rear axle
    constexpr const RearAxle_t& get_rear_axle() const { return _rear_axle; }
    constexpr       RearAxle_t& get_rear_axle()       { return _rear_axle; }

    //! Get a reference to the inertial frame
    constexpr const Frame<Timeseries_t>& get_inertial_frame() const { return _inertial_frame; }
    constexpr       Frame<Timeseries_t>& get_inertial_frame()       { return _inertial_frame; }

    //! Get a reference to the road frame
    constexpr const Frame<Timeseries_t>& get_road_frame() const { return _road_frame; }
    constexpr       Frame<Timeseries_t>& get_road_frame()       { return _road_frame; }

    //! Get a reference to the chassis frame
    constexpr const Frame<Timeseries_t>& get_chassis_frame() const { return _chassis_frame; }
    constexpr       Frame<Timeseries_t>& get_chassis_frame()       { return _chassis_frame; }

    //! Get the total force acting on the chassis CoM [N]
    constexpr const Vector3d<Timeseries_t>& get_force() const { return _F; }

    //! Get the total torque acting on the chassis CoM [Nm]
    constexpr const Vector3d<Timeseries_t>& get_torque() const { return _T; }

    //! Get the aerodynamic force [N]
    Vector3d<Timeseries_t> get_aerodynamic_force() const; 

    //! Get understeer(<0) or oversteer(>0) indicator
    Timeseries_t get_understeer_oversteer_indicator() const;

    //! Get the drag coefficient
    const Timeseries_t& get_drag_coefficient() const { return _cd; }

    //! Add an additional external force
    //! @param[in] F: force to be added [N]
    constexpr void set_external_force(const Vector3d<Timeseries_t>& F) { _Fext = F; } 

    //! Add an additional external torque
    //! @param[in] T: torque to be added [Nm]
    constexpr void set_external_torque(const Vector3d<Timeseries_t>& T) { _Text = T; } 

    //! Perform an update of the class: move the road frame to the new position
    //! @param[in] x: x-position of the road frame [m]
    //! @param[in] y: y-position of the road frame [m]
    //! @param[in] psi: yaw angle of the road frame [rad]
    void update(Timeseries_t x, Timeseries_t y, Timeseries_t psi);

    //! Load the time derivative of the state variables computed herein to the dqdt
    //! @param[out] dqdt: the vehicle state vector time derivative
    template<size_t N>
    void get_state_and_state_derivative(std::array<Timeseries_t,N>& state, 
                                        std::array<Timeseries_t, N>& dstate_dt) const;

    //! Set the state variables of this class
    //! @param[in] q: the vehicle state vector 
    //! @param[in] u: the vehicle control vector
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_controls(const std::array<Timeseries_t,NSTATE>& input_states, 
                                const std::array<Timeseries_t,NCONTROL>& controls);

    //! Set the state and controls upper, lower, and default values
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_control_upper_lower_and_default_values(std::array<scalar,NSTATE>& input_states_def,
                                                               std::array<scalar,NSTATE>& input_states_lb,
                                                               std::array<scalar,NSTATE>& input_states_ub,
                                                               std::array<scalar,NCONTROL>& controls_def,
                                                               std::array<scalar,NCONTROL>& controls_lb,
                                                               std::array<scalar,NCONTROL>& controls_ub,
                                                               scalar velocity_x_lb, 
                                                               scalar velocity_x_ub, 
                                                               scalar velocity_y_ub, 
                                                               scalar omega_ub
                                                              ) const;


    //! Get the names of the state and control varaibles of this class
    //! @param[out] q: the vehicle state names
    //! @param[out] u: the vehicle control names
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_control_names(std::array<std::string,NSTATE>& input_states, 
                                     std::array<std::string,NCONTROL>& controls) const;


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

    // Road frame velocities
    Timeseries_t _u = 0.0;         //! [in] Longitudinal velocity (in road frame) [m/s]
    Timeseries_t _v = 0.0;         //! [in] Lateral velocity (in road frame) [m/s]
    Timeseries_t _omega = 0.0;     //! [in] Yaw speed [rad/s]

    Frame<Timeseries_t> _inertial_frame; //! The inertial frame
    Frame<Timeseries_t> _road_frame;     //! Frame<Timeseries_t> with center on the road projection of the CoG, 
                           //! aligned with the chassis
    Frame<Timeseries_t> _chassis_frame;  //! Frame<Timeseries_t> with center on the CoG and parallel axes to the road frame

    // Mass properties
    Timeseries_t _m;     //! [c] chassis mass [kg]
    sMatrix3x3 _I; //! [c] chassis inertia matrix [kg.m2]

    // Aerodynamic properties
    scalar _rho;   //! [c] air density [kg/m3]
    Timeseries_t _cd;    //! [c] drag coefficient [-]      
    Timeseries_t _cl;    //! [c] lift coefficient [-]      
    scalar _A;     //! [c] frontal area [m2]
    
    FrontAxle_t _front_axle; //! Front axle
    RearAxle_t  _rear_axle;  //! Rear axle

    DECLARE_PARAMS(
        { "mass", _m },
        { "inertia/Ixx", _I.xx() },
        { "inertia/Ixy", _I.xy() },
        { "inertia/Ixz", _I.xz() },
        { "inertia/Iyx", _I.yx() },
        { "inertia/Iyy", _I.yy() },
        { "inertia/Iyz", _I.yz() },
        { "inertia/Izx", _I.zx() },
        { "inertia/Izy", _I.zy() },
        { "inertia/Izz", _I.zz() },
        { "aerodynamics/rho", _rho },
        { "aerodynamics/cd", _cd },
        { "aerodynamics/cl", _cl },
        { "aerodynamics/area", _A }
    ); 


    std::unordered_map<std::string,Timeseries_t> get_outputs_map_self() const
    {
        return
        {
            {get_name() + ".acceleration.x", get_longitudinal_acceleration()},
            {get_name() + ".acceleration.y", get_lateral_acceleration()},
            {get_name() + ".force.x", _F.x()},
            {get_name() + ".force.y", _F.y()},
            {get_name() + ".force.z", _F.z()},
            {get_name() + ".torque.x", _T.x()},
            {get_name() + ".torque.y", _T.y()},
            {get_name() + ".torque.z", _T.z()},
            {get_name() + ".acceleration.yaw", _dOmega},
            {get_name() + ".position.x", _road_frame.get_origin().x()},
            {get_name() + ".position.y", _road_frame.get_origin().y()},
            {get_name() + ".attitude.yaw", _road_frame.get_rotation_angles().front()},
            {get_name() + ".understeer_oversteer_indicator", get_understeer_oversteer_indicator()},
            {get_name() + ".aerodynamics.cd", _cd}
        };
    }

 protected:


    // Longitudinal dynamics
    Timeseries_t _du = 0.0;    //! [out] Longitudinal acceleration [m/s2]
    
    // Lateral dynamics
    Timeseries_t _dv = 0.0;    //! [out] Lateral acceleration [m/s2]

    // Attitude: yaw
    Timeseries_t _dOmega = 0.0; //! [out] Yaw acceleration [rad/s2]

    Vector3d<Timeseries_t> _F; //! [out] Total forces at road frame
    Vector3d<Timeseries_t> _T; //! [out] Total torque at road frame

    Vector3d<Timeseries_t> _Fext = Vector3d<Timeseries_t>(0.0); //! [in] An external force [N]
    Vector3d<Timeseries_t> _Text = Vector3d<Timeseries_t>(0.0); //! [in] An external torque [Nm]
};

#include "chassis.hpp"

#endif
