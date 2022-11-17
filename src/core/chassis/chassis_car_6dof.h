#ifndef CHASSIS_CAR_6DOF_H
#define CHASSIS_CAR_6DOF_H

#include "chassis.h"
#include <array>
#include "lion/math/euler_angles.h"

//!     Car chassis model with two small pitch and roll rotations
//!     ---------------------------------------------------------
//!
//!     The chassis frame is parallel to the road projection of CoM frame.
//! Therefore, the small rotations are not included in the frame motions
//! and will be manually included through updates of the positions of the 
//! axles frames.
//!  * The chassis frame coordinates are (0,0,z-h) and velocity (0,0,dz). Parallel to the road frame
//!  * The axles are located at (x_ax,0,z_ax + mu.x_ax) with velocity (0,0,dmu.x_ax)
//!  @param FrontAxle_t: type of the front axle
//!  @param RearAxle_t: type of the rear axle
//!  @param STATE0: index of the first state variable defined here
//!  @param CONTROL0: index of the first control variable defined here
template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
class Chassis_car_6dof : public Chassis<Timeseries_t,FrontAxle_t, RearAxle_t, STATE0,CONTROL0>
{
 public:

    //! Type of the chassic of which this class derives
    using base_type             = Chassis<Timeseries_t,FrontAxle_t, RearAxle_t,STATE0,CONTROL0>;

    //! Type of the front axle
    using Front_axle_type       = FrontAxle_t;

    //! Type of the rear axle
    using Rear_axle_type        = RearAxle_t;

    //! Type of the front left tyre
    using Front_left_tire_type  = typename FrontAxle_t::Tire_left_type;

    //! Type of the front right tire
    using Front_right_tire_type = typename FrontAxle_t::Tire_right_type;

    //! Type of the rear left tire
    using Rear_left_tire_type   = typename RearAxle_t::Tire_left_type;

    //! Type of the rear right tire
    using Rear_right_tire_type  = typename RearAxle_t::Tire_right_type;
    
    //! The two axles: FRONT and REAR
    enum Axles { FRONT, REAR }; 

    // State variables: six dof rigid body motion - add the 3 remaining dofs
    struct input_state_names : public base_type::input_state_names
    {
        enum
        {
            Z = base_type::input_state_names::end,  //! Vertical displacement of the chassis [m]
            PHI,                                    //! Roll angle (assumed small) [rad]
            MU,                                     //! Pitch angle (assumed small) [rad]
            DZDT,                                   //! Vertical displacement derivative [m/s]
            DPHIDT,                                 //! Roll angular speed [rad/s]
            DMUDT,                                  //! Pitch angular speed [rad/s]
            end
        };
    };

    struct state_names : public base_type::state_names
    {
        enum
        {
            Z      = input_state_names::Z,
            PHI    = input_state_names::PHI,
            MU     = input_state_names::MU,
            DZDT   = input_state_names::DZDT,
            DPHIDT = input_state_names::DPHIDT,
            DMUDT  = input_state_names::DMUDT
        };
    };

    //! Control variables: none
    struct control_names : public base_type::control_names
    {
        enum 
        {
            end = base_type::control_names::end
        };
    };

    constexpr static size_t NALGEBRAIC = 0;  //! Number of algebraic equations

    //! Default constructor
    Chassis_car_6dof();

    //! Constructor from axles and extra parameters
    //! @param[in] front_axle: Front axle
    //! @param[in] rear_axle: Rear axle
    //! @param[in] parameters: map containing mechanical/geometrical parameters by name
    //! @param[in] path: path to find its parameters on the map
    Chassis_car_6dof(const FrontAxle_t& front_axle, 
                const RearAxle_t& rear_axle,
                Xml_document& database,
                const std::string& path=""
               );

    //! Constructor only from parameter map and tire types
    //! @param[in] parameters: map with all mechanical/geometrical parameters by name.
    //!                        It must contain the parameters to define the tires and axles
    //! @param[in] front_left_tire_type: Type of the front left tire
    //! @param[in] front_right_tire_type: Type of the front right tire type
    //! @param[in] rear_left_tire_type: Type of the rear left tire
    //! @param[in] rear_right_tire_type: Type of the rear right tire type
    Chassis_car_6dof(Xml_document& database);

    //! Update the chassis: update the axles to get forces and compute accelerations
    //! @param[in] x: x-coordinate of the road frame [m]
    //! @param[in] y: y-coordinate of the road frame [m]
    //! @param[in] psi: yaw angle of the road frame [rad]
    void update(const Vector3d<Timeseries_t>& ground_position_vector_m,
                const Euler_angles<scalar>& road_euler_angles_rad,
                const Timeseries_t& track_heading_angle_rad,
                const Euler_angles<Timeseries_t>& road_euler_angles_dot_radps,
                const Timeseries_t& track_heading_angle_dot_radps,
                const Timeseries_t& ground_velocity_z_body_mps);



    //! Set the chassis state variables from direct values
    //      --- Set by the parent class ---
    //! @param[in] u: road frame x-velocity (in road frame) [m/s]
    //! @param[in] v: road frame y-velocity (in road frame) [m/s]
    //! @param[in] omega: road frame yaw speed [rad/s]
    //!     --- Set in here ---
    //! @param[in] z: chassis vertical displacement [m]
    //! @param[in] dz: chassis vertical velocity [m/s]
    //! @param[in] mu: chassis pitch angle [rad]
    //! @param[in] dmu: chassis pitch speed [rad/s]
    //! @param[in] phi: chassis roll angle [rad]
    //! @param[in] dphi: chassis roll speed [rad/s]
    void set_state(Timeseries_t u, Timeseries_t v, Timeseries_t omega, 
                   Timeseries_t z, Timeseries_t dz, Timeseries_t mu, 
                   Timeseries_t dmu, Timeseries_t phi, Timeseries_t dphi);

    //! Get the CoM position in road frame
    Vector3d<Timeseries_t> get_com_position() const { return _x_com + Vector3d<Timeseries_t>(0.0, 0.0, _z); }

    //! Get the CoM velocity in road frame
    Vector3d<Timeseries_t> get_com_velocity() const { return { 0.0, 0.0, _dz }; }

    //! Computes the axle position in chassis frame. It would be constant, but
    //! due to the small pitch angle , the Z position moves proportional to its x coordinate
    //!
    //! @param[in] axle: which axle to be computed
    Vector3d<Timeseries_t> get_front_axle_position() const 
        { return _x_front_axle + Vector3d<Timeseries_t>(0.0, 0.0, -_mu*_x_front_axle[0]); }

    Vector3d<Timeseries_t> get_rear_axle_position() const 
        { return _x_rear_axle + Vector3d<Timeseries_t>(0.0, 0.0, -_mu*_x_rear_axle[0]); }

    //! Get the chassis CoM absolute acceleration in road frame
    Vector3d<Timeseries_t> get_acceleration() const { return {base_type::_du, base_type::_dv, _d2z}; }

    //! Get the chassis angular acceleration
    Vector3d<Timeseries_t> get_angles_acceleration() const { return {_d2phi, _d2mu, base_type::_dOmega}; }

    //! Computes the axle velocity in chassis frame. It would be zero, but
    //! the small pitch angle (not captured in the chassis frame) induces a vertical velocity
    //! @param[in] axle: which axle to be computed
    Vector3d<Timeseries_t> get_front_axle_velocity() const { return {0.0, 0.0, - _dmu*_x_front_axle[0] }; }

    Vector3d<Timeseries_t> get_rear_axle_velocity() const { return {0.0, 0.0, - _dmu*_x_rear_axle[0] }; }

    //! Return a mechanical/geometrical parameter by name
    //! @param[in] parameter_name: name of the parameter
    scalar get_parameter(const std::string& parameter_name) const;

    //! Load the time derivative of the state variables computed herein to the dqdt
    //! @param[out] dqdt: the vehicle state vector time derivative
    template<size_t N>
    void get_state_and_state_derivative(std::array<Timeseries_t, N>& state, 
                                        std::array<Timeseries_t,N>& dstate_dt
                                       ) const;

    //! Set the state variables of this class
    //! @param[in] q: the vehicle state vector 
    //! @param[in] u: the vehicle control vector
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_controls(const std::array<Timeseries_t,NSTATE>& input_states, 
                                const std::array<Timeseries_t,NALGEBRAIC>& algebraic_states,
                                const std::array<Timeseries_t,NCONTROL>& controls);

    //! Set the state and controls upper, lower, and default values
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_control_upper_lower_and_default_values(std::array<scalar, NSTATE>& input_states_def,
        std::array<scalar, NSTATE>& input_states_lb,
        std::array<scalar, NSTATE>& input_states_ub,
        std::array<scalar, NALGEBRAIC>& algebraic_states_def,
        std::array<scalar, NALGEBRAIC>& algebraic_states_lb,
        std::array<scalar, NALGEBRAIC>& algebraic_states_ub,
        std::array<scalar, NCONTROL>& control_def,
        std::array<scalar, NCONTROL>& control_lb,
        std::array<scalar, NCONTROL>& control_ub
    ) const;

    //! Get the names of the state and control varaibles of this class
    //! @param[out] q: the vehicle state names
    //! @param[out] u: the vehicle control names
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_control_names(std::array<std::string, NSTATE>& input_states,
        std::array<std::string, NALGEBRAIC>& algebraic_states,
        std::array<std::string, NCONTROL>& control_states) const;

    bool is_ready() const { return base_type::is_ready() && 
        std::all_of(__used_parameters.begin(), __used_parameters.end(), [](const auto& v) -> auto { return v; }); }

    static std::string type() { return "chassis_car_6dof"; }

    std::unordered_map<std::string,Timeseries_t> get_outputs_map() const
    {
        auto map = get_outputs_map_self();
        const auto base_type_map = base_type::get_outputs_map();

        map.insert(base_type_map.cbegin(), base_type_map.cend());

        return map;
    }


 private:
    //! Compute the left hand side of Newton's equations (linear momentum derivative)
    Vector3d<Timeseries_t> Newton_lhs() const;

    //! Compute all terms of the left hand side of Euler's equations except the second order derivatives
    Vector3d<Timeseries_t> Euler_lhs() const;

    //! Compute the mass matrix of Euler's equations: only the second order derivatives
    Matrix3x3<Timeseries_t> Euler_m() const;

    // Geometrical properties
    sVector3d _x_com;        //! [c] Center of mass position[m]
    sVector3d _x_front_axle; //! [c] Center of front axle [m]
    sVector3d _x_rear_axle;  //! [c] Center of the rear axle [m]

    // Vertical dynamics
    Timeseries_t _z = 0.0;      //! [in] Vertical displacement of CoG [m]
    Timeseries_t _dz = 0.0;     //! [in] Vertical velocity of CoG [m/s]
    Timeseries_t _d2z = 0.0;    //! [out] Vertical acceleration of CoG [m/s2]

    // Small angle attitude: pitch
    Timeseries_t _mu = 0.0;     //! [in] Pitch angle [rad]
    Timeseries_t _dmu = 0.0;    //! [in] Pitch omega [rad/s]
    Timeseries_t _d2mu = 0.0;   //! [out] Pitch acceleration [rad/s2]

    // Small angle attitude: roll
    Timeseries_t _phi = 0.0;    //! [in] Roll angle [rad]
    Timeseries_t _dphi = 0.0;   //! [in] Roll omega [rad/s]
    Timeseries_t _d2phi = 0.0;  //! [out] Roll acceleration [rad/s2]

    DECLARE_PARAMS(
        { "com", _x_com },
        { "front_axle", _x_front_axle },
        { "rear_axle", _x_rear_axle },
    ); 

    std::unordered_map<std::string,Timeseries_t> get_outputs_map_self() const
    {
        return
        {
        };
    }
};

#include "chassis_car_6dof.hpp"


#endif
