#ifndef CHASSIS_CAR_3DOF_H
#define CHASSIS_CAR_3DOF_H

#include "src/core/foundation/fastest_lap_exception.h"
#include "lion/math/euler_angles.h"
#include "chassis.h"
#include <array>
#include <vector>

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
//!  @param state_start: index of the first state variable defined here
//!  @param control_start: index of the first control variable defined here
template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
class Chassis_car_3dof : public Chassis<Timeseries_t,FrontAxle_t, RearAxle_t, state_start, control_start>
{
 public:

    //! Type of the chassic of which this class derives
    using base_type             = Chassis<Timeseries_t,FrontAxle_t, RearAxle_t, state_start, control_start>;

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

    // Inputs: the four vertical forces at each wheel contact point
    struct input_names : public base_type::input_names
    {
        enum { force_z_fl_g = base_type::input_names::end, force_z_fr_g, force_z_rl_g, force_z_rr_g, end };
    };

    //! States: the vertical velocity, and the two small rotation angular momentum
    struct state_names : public base_type::state_names
    {
        enum { com_velocity_z_mps = base_type::state_names::end,
               roll_angular_momentum_Nms, 
               pitch_angular_momentum_Nms,
               roll_balance_equation_g,
               end};
    };

    //! Control variables:: throttle/brake, and brake-bias
    struct control_names : public base_type::control_names
    {
        enum { throttle = base_type::control_names::end, brake_bias, end};
    };

    static_assert(static_cast<size_t>(input_names::end) == static_cast<size_t>(state_names::end));

    //! Default constructor
    Chassis_car_3dof();

    //! Constructor from axles and extra parameters
    //! @param[in] front_axle: Front axle
    //! @param[in] rear_axle: Rear axle
    //! @param[in] parameters: map containing mechanical/geometrical parameters by name
    //! @param[in] path: path to find its parameters on the map
    Chassis_car_3dof(const FrontAxle_t& front_axle, 
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
    Chassis_car_3dof(Xml_document& database);

    template<size_t number_of_inputs, size_t number_of_controls>
    void transform_states_to_inputs(const std::array<Timeseries_t,number_of_inputs>& states,
                                          const std::array<Timeseries_t,number_of_controls>& controls,
                                          std::array<Timeseries_t,number_of_inputs>& input_states);

    //! Modifyer to set a parameter from the database
    template<typename T>
    void set_parameter(const std::string& parameter, const T value);

    //! Fill the corresponding nodes of an xml document
    void fill_xml(Xml_document& doc) const;

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
    void set_state(const Timeseries_t& u, const Timeseries_t& v, const Timeseries_t& omega); 

    //! Get the CoM position in road frame
    Vector3d<Timeseries_t> get_com_position() const { return _x_com; }

    //! Get the CoM velocity in road frame
    Vector3d<Timeseries_t> get_com_velocity() const { return { 0.0, 0.0, 0.0 }; }

    //! Get the CoM absolute vertical in body axes
    constexpr const Timeseries_t& get_com_velocity_z_mps() const { return _com_velocity_z_mps; }

    //! Get the derivative of the CoM absolute vertical in body axes
    constexpr const Timeseries_t& get_com_velocity_z_dot_mps2() const { return _com_velocity_z_dot_mps2; }

    //! Get the derivative of the pitch angular momentum
    constexpr const Timeseries_t& get_pitch_angular_momentum_dot_Nm() const { return _pitch_angular_momentum_dot_Nm; }

    //! Get the derivative of the roll angular momentum
    constexpr const Timeseries_t& get_roll_angular_momentum_dot_Nm() const { return _roll_angular_momentum_dot_Nm; }

    //! Get the roll balance equation
    constexpr const Timeseries_t& get_roll_balance_equation_N() const { return _roll_balance_equation_N; }

    //! Computes the front axle position in chassis frame. It is constant in the 3DOF model
    Vector3d<Timeseries_t> get_front_axle_position() const { return {_x_front_axle[0], _x_front_axle[1], _x_front_axle[2]}; }

    //! Computes the rear axle position in chassis frame. It is constant in the 3DOF model
    Vector3d<Timeseries_t> get_rear_axle_position() const { return {_x_rear_axle[0], _x_rear_axle[1], _x_rear_axle[2]}; }

    //! Get the chassis CoM absolute acceleration in road frame
    Vector3d<Timeseries_t> get_acceleration() const { return {base_type::_du, base_type::_dv, 0.0}; }

    //! Get the chassis angular acceleration
    Vector3d<Timeseries_t> get_angles_acceleration() const { return {0.0, 0.0, base_type::_dOmega}; }

    //! Computes the front axle velocity in chassis frame. Zero in the 3DOF model
    Vector3d<Timeseries_t> get_front_axle_velocity() const { return {0.0, 0.0, 0.0}; }

    //! Computes the rear axle velocity in chassis frame. Zero in the 3DOF model
    Vector3d<Timeseries_t> get_rear_axle_velocity() const { return {0.0, 0.0, 0.0}; }

    //! Get a negative normal force
    const Timeseries_t& get_negative_normal_force(const size_t& id) const
    {
        switch(id)
        {
         case (input_names::force_z_fl_g): return _neg_force_z_fl_N; break; 
         case (input_names::force_z_fr_g): return _neg_force_z_fr_N; break; 
         case (input_names::force_z_rl_g): return _neg_force_z_rl_N; break; 
         case (input_names::force_z_rr_g): return _neg_force_z_rr_N; break; 
         default:      throw fastest_lap_exception("Id is incorrect");
        }
    }

    //! Get the throttle
    const Timeseries_t& get_throttle() const { return _throttle; }

    //! Get the brake bias
    const Timeseries_t& get_brake_bias() const { return _brake_bias; }

    //! Return a mechanical/geometrical parameter by name
    //! @param[in] parameter_name: name of the parameter
    scalar get_parameter(const std::string& parameter_name) const;

    //! Load the time derivative of the state variables computed herein to the dqdt
    //! @param[out] dqdt: the vehicle state vector time derivative
    template<size_t number_of_states>
    void get_state_and_state_derivative(std::array<Timeseries_t, number_of_states>& state,
                                        std::array<Timeseries_t, number_of_states>& dstate_dt
                                        ) const;

    //! Set the state variables of this class
    //! @param[in] q: the vehicle state vector 
    //! @param[in] u: the vehicle control vector
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_controls(const std::array<Timeseries_t,number_of_inputs>& inputs, 
                                const std::array<Timeseries_t,number_of_controls>& controls);


    //! Set the state and controls upper, lower, and default values
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_upper_lower_and_default_values(std::array<scalar,number_of_inputs>& inputs_def,
                                                              std::array<scalar,number_of_inputs>& inputs_lb,
                                                              std::array<scalar,number_of_inputs>& inputs_ub,
                                                              std::array<scalar,number_of_controls>& control_def,
                                                              std::array<scalar,number_of_controls>& control_lb,
                                                              std::array<scalar,number_of_controls>& control_ub
                                                              ) const;

    //! Get the names of the state and control varaibles of this class
    //! @param[out] q: the vehicle state names
    //! @param[out] u: the vehicle control names
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_names(std::array<std::string,number_of_inputs>& inputs, 
                                     std::array<std::string,number_of_controls>& controls) const;

    bool is_ready() const { return base_type::is_ready() && 
        std::all_of(__used_parameters.begin(), __used_parameters.end(), [](const auto& v) -> auto { return v; }); }

    static std::string type() { return "chassis_car_3dof"; }

    std::unordered_map<std::string,Timeseries_t> get_outputs_map() const
    {
        auto map = get_outputs_map_self();
        const auto base_type_map = base_type::get_outputs_map();

        map.insert(base_type_map.cbegin(), base_type_map.cend());

        return map;
    }


 private:

    // Geometrical properties
    Vector3d<Timeseries_t> _x_com;              //! [c] Center of mass position[m]
    Vector3d<Timeseries_t> _x_front_axle;       //! [c] Center of front axle [m]
    Vector3d<Timeseries_t> _x_rear_axle;        //! [c] Center of the rear axle [m]
    Vector3d<Timeseries_t> _x_aero;             //! [c] Aerodynamic center [m]

    // Mechanical properties
    Timeseries_t   _roll_balance_coeff; //! [c] Coefficient in [0,1], usually 1/2, such that: Fz_fr − Fz_fl = D(Fz_fr + Fz_rr − Fz_fl − Fz_rl)
    scalar         _force_z_max_ref2;        //! [c] Square of the parasitic smooth positive force when Fz = 0

    // Variables    ----------------------------------------------------------------

    // Setteable variables
    Timeseries_t _brake_bias_0; //! [in] Initial value for the brake bias (to be read from database)

    // Control variables
    Timeseries_t _throttle   = 0.0;   //! [in] throttle: 1-full throttle, -1-hard brake
    Timeseries_t _brake_bias = 0.0;   //! [in] Brake bias: 1-only front, 0-only rear

    // Algebraic variables
    Timeseries_t _force_z_fl_N = 0.0;    //! [in] Vertical load for FL tire
    Timeseries_t _force_z_fr_N = 0.0;    //! [in] Vertical load for FR tire
    Timeseries_t _force_z_rl_N = 0.0;    //! [in] Vertical load for RL tire
    Timeseries_t _force_z_rr_N = 0.0;    //! [in] Vertical load for RR tire

    Timeseries_t _neg_force_z_fl_N = 0.0;    //! negative vertical load for FL tire
    Timeseries_t _neg_force_z_fr_N = 0.0;    //! negative vertical load for FR tire
    Timeseries_t _neg_force_z_rl_N = 0.0;    //! negative vertical load for RL tire
    Timeseries_t _neg_force_z_rr_N = 0.0;    //! negative vertical load for RR tire

    // Algebraic constraints
    Timeseries_t _com_velocity_z_mps         = 0.0;
    Timeseries_t _roll_angular_momentum_Nms  = 0.0;
    Timeseries_t _pitch_angular_momentum_Nms = 0.0;

    Timeseries_t _com_velocity_z_dot_mps2       = 0.0;
    Timeseries_t _roll_angular_momentum_dot_Nm  = 0.0;
    Timeseries_t _pitch_angular_momentum_dot_Nm = 0.0;

    Timeseries_t _roll_balance_equation_N = 0.0;  //! [out] roll balance equation

    DECLARE_PARAMS(
        { "com/x", _x_com.x() },
        { "com/y", _x_com.y() },
        { "com/z", _x_com.z() },
        { "front_axle/x", _x_front_axle.x() },
        { "front_axle/y", _x_front_axle.y() },
        { "front_axle/z", _x_front_axle.z() },
        { "rear_axle/x", _x_rear_axle.x() },
        { "rear_axle/y", _x_rear_axle.y() },
        { "rear_axle/z", _x_rear_axle.z() }, 
        { "pressure_center/x", _x_aero.x() },
        { "pressure_center/y", _x_aero.y() },
        { "pressure_center/z", _x_aero.z() },
        { "brake_bias", _brake_bias_0 },
        { "roll_balance_coefficient", _roll_balance_coeff},
        { "Fz_max_ref2", _force_z_max_ref2 }
    );  

    std::unordered_map<std::string,Timeseries_t> get_outputs_map_self() const
    {
        return
        {
        };
    }
};

#include "chassis_car_3dof.hpp"

#endif
