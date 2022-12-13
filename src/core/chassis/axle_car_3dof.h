#ifndef AXLE_CAR_3DOF_H
#define AXLE_CAR_3DOF_H

#include "axle.h"
#include "src/core/actuators/engine.h"
#include "src/core/actuators/brake.h"
#include "lion/io/Xml_document.h"
#include "lion/io/database_parameters.h"

//!     Car axle model with small roll rotation
//!     ---------------------------------------
//!
//! The frame is parallel to the chassis frame. The small roll rotation
//! is manually included when the position and velocity of the tires is updated
//!
//! * The position of the tires are (0,y_tire,0), 
//!   where y_tire is the constant y position of the tires
//! * The velocity of the tires is (0,0,0).
//!
//! Types of car axles implemented:
//!
//! 1) Powered with differential: both tires have different rotation speed the same rotation speed,
//!     and a torque T is imposed by the driver
//!
//! 2) Steering with free roll: we don't impose any condition on omega (i.e.
//!     the tires are free to roll), and the tires plane can rotate around Z (steering)
//!

//! State and control variables for Powered (with differential) axles
//!  @param state_start: index of the first state variable defined here
//!  @param control_start: index of the first control variable defined here
template<size_t state_start, size_t control_start>
struct POWERED
{
    //! State variables: kappa of the two wheels
    struct input_names
    {
        enum
        {
            KAPPA_LEFT = state_start,  //! Left tire longitudinal slip [-]
            KAPPA_RIGHT,               //! Right tire longitudinal slip [-]
            end
        };
    };

    struct state_names
    {
        enum
        {
            angular_momentum_left = state_start,
            angular_momentum_right,
            end
        };
    };

    static_assert(static_cast<size_t>(input_names::end) == static_cast<size_t>(state_names::end));

    //! Control variables: boost
    struct control_names
    {
        enum Controls
        {
            boost = control_start, end
        };
    };
};

//! State and control variables for Steering with free roll axles
//!  @param state_start: index of the first state variable defined here
//!  @param control_start: index of the first control variable defined here
template<size_t state_start, size_t control_start>
struct STEERING
{
    //! State variables: kappa of the two wheels
    struct input_names
    {
        enum
        {
            KAPPA_LEFT = state_start,  //! Left tire longitudinal slip [-]
            KAPPA_RIGHT,               //! Right tire longitudinal slip [-]
            end
        };
    };

    struct state_names
    {
        enum
        {
            angular_momentum_left = state_start,
            angular_momentum_right,
            end
        };
    };

    static_assert(static_cast<size_t>(input_names::end) == static_cast<size_t>(state_names::end));

    //! Control variables: steering angle
    struct control_names
    {
        enum Controls
        {
            STEERING = control_start,    //! Steering angle [rad]
            end
        };
    };
};

//! Car axle class
//!  @param Tire_left_t: type of the left tire
//!  @param Tire_right_t: type of the right tire
//!  @param Axle_mode: POWERED_WITHOUT_DIFFERENTIAL or STEERING_FREE_ROLL
//!  @param state_start: index of the first state variable defined here
//!  @param control_start: index of the first control variable defined here
template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
class Axle_car_3dof : public Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>, 
    public Axle_mode<Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::state_names::end, 
                     Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::control_names::end> 
{
 public:

    //! The parent axle class
    using base_type       = Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>;

    //! The axle sub type (POWERED_WITHOUT_DIFFERENTIAL/STEERING_FREE_ROLL)
    using Axle_type       = Axle_mode<Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::state_names::end,
                                      Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::control_names::end>;

    struct input_names           : public Axle_type::input_names,   base_type::input_names {};
    struct state_names           : public Axle_type::state_names,   base_type::state_names {};
    struct control_names         : public Axle_type::control_names, base_type::control_names {};

    //! The left tire type
    using Tire_left_type  = Tire_left_t;

    //! The right tire type
    using Tire_right_type = Tire_right_t;

    //! The two tires: left and right
    enum Tires : size_t { LEFT, RIGHT };

    //! Default constructor
    Axle_car_3dof(const std::string& name, 
             const Tire_left_t& tire_l,
             const Tire_right_t& tire_r,
             const std::string& path=""
            );

    //! Constructor
    //! @param[in] name: name given to the axle for identification (e.g. front, rear)
    //! @param[in] tire_l: left tire
    //! @param[in] tire_r: right tire
    //! @param[in] parameters: map containing the mechanical and geometrical parameters
    //! @param[in] path: path of this axle in the parameters map
    Axle_car_3dof(const std::string& name, 
             const Tire_left_t& tire_l,
             const Tire_right_t& tire_r,
             Xml_document& database,
             const std::string& path=""
            );

    template<typename T>
    bool set_parameter(const std::string& parameter, const T value);

    template<size_t number_of_inputs, size_t number_of_controls>
    void transform_states_to_inputs(const std::array<Timeseries_t,number_of_inputs>& states,
                                    const std::array<Timeseries_t,number_of_controls>& controls,
                                    std::array<Timeseries_t,number_of_inputs>& inputs);

    //! Updates the axle: compute angular_momentum_left and angular_momentum_right, plus the equivalent force+torque at the axle center
    //! @param[in] Fz_left: the normal force of the left tire
    //! @param[in] Fz_right: the normal force of the right tire
    //! @param[in] throttle: the throttle/brake percentage in [-1,1]
    //! @param[in] brake_bias: the brake bias in [0,1]
    void update(Timeseries_t Fz_left, Timeseries_t Fz_right, Timeseries_t throttle, Timeseries_t brake_bias);

    //! Get the track
    const scalar& get_track() const { return _track; }

    //! Get the steering angle [rad]
    const Timeseries_t& get_steering_angle() const { return _delta; }

    //! Get the angular speed of the left tire [rad/s]
    const Timeseries_t& get_kappa_dimensionless_left() const { return _kappa_dimensionless_left; }

    //! Get the angular speed of the right tire [rad/s]
    const Timeseries_t& get_kappa_dimensionless_right() const { return _kappa_dimensionless_right; }

    //! Get the axle angular acceleration [rad/s2]
    const Timeseries_t& get_dangular_momentum_dt_left() const { return _dangular_momentum_dt_left; } 

    //! Get the axle angular acceleration [rad/s2]
    const Timeseries_t& get_dangular_momentum_dt_right() const { return _dangular_momentum_dt_right; } 

    //! Get the left wheel torque [N.m]
    const Timeseries_t& get_torque_left() const { return _torque_left; }

    //! Get the right wheel torque [N.m]
    const Timeseries_t& get_torque_right() const { return _torque_right; }

    //! Get the tire position (in axle frame)
    //! @param[in] tire: which tire (LEFT/RIGHT)
    const Vector3d<Timeseries_t> get_tire_position(Tires tire) const { return {0.0, _y_tire[tire], 0.0}; }

    //! Get the tire position y-position (in axle frame)
    //! @param[in] tire: which tire (LEFT/RIGHT)
    const scalar get_tire_y_position(Tires tire) const { return _y_tire[tire]; }

    //! Get the tire relative velocity (in axle frame)
    //! @param[in] tire: which tire (LEFT/RIGHT)
    const Vector3d<Timeseries_t> get_tire_velocity(Tires tire) const { return {0.0,0.0,0.0}; }

    //! Get the engine
    const Engine<Timeseries_t>& get_engine() const { return _engine; }

    const Engine<Timeseries_t>& get_engine_boost() const { return _engine_boost; }

    const Timeseries_t& get_boost() const { return _boost; }

    //! Get a certain tire mechanical or geometrical parameter by name
    //! @param[in] parameter_name: name of the parameter
    scalar get_parameter(const std::string& parameter_name) const;

    //! Load the time derivative of the state variables computed herein to the dqdt
    //! @param[out] dqdt: the vehicle state vector time derivative
    template<size_t number_of_states>
    void get_state_and_state_derivative(std::array<Timeseries_t,number_of_states>& state,
                                        std::array<Timeseries_t,number_of_states>& dstate_dt,
                                        const Timeseries_t& mass_kg) const;

    //! Set the state variables of this class
    //! @param[in] q: the vehicle state vector 
    //! @param[in] u: the vehicle control vector
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_controls(const std::array<Timeseries_t,number_of_inputs>& inputs,
                                const std::array<Timeseries_t,number_of_controls>& controls
                               );

    //! Set the state and controls upper, lower, and default values
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_upper_lower_and_default_values(std::array<scalar,number_of_inputs>& inputs_def,
                                                               std::array<scalar,number_of_inputs>& inputs_lb,
                                                               std::array<scalar,number_of_inputs>& inputs_ub,
                                                               std::array<scalar,number_of_controls>& controls_def,
                                                               std::array<scalar,number_of_controls>& controls_lb,
                                                               std::array<scalar,number_of_controls>& controls_ub 
                                                              ) const;

    //! Get the names of the state and control variables of this class
    //! @param[out] q: the vehicle state names
    //! @param[out] u: the vehicle control names
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_names(std::array<std::string,number_of_inputs>& inputs, 
                                     std::array<std::string,number_of_controls>& controls) const;

    bool is_ready() const { return base_type::is_ready() && 
        std::all_of(__used_parameters.begin(), __used_parameters.end(), [](const auto& v) -> auto { return v; }); }

    static std::string type() { return "axle_car_3dof"; }

    void fill_xml(Xml_document& doc) const;

    std::unordered_map<std::string,Timeseries_t> get_outputs_map() const
    {
        auto map = get_outputs_map_self();
        const auto base_type_map = base_type::get_outputs_map();

        map.insert(base_type_map.cbegin(), base_type_map.cend());

        return map;
    }

 private:
    // Geometry
    scalar _track;                      //! [c] Distance between the two tires [m]
    std::array<scalar,2> _y_tire;       //! [c] y position of the two tires [m]

    // Mechanical
    scalar _I;                          //! [c] Inertia
    Timeseries_t _differential_stiffness;     //! [c] Differential stiffness [N.m.s/rad]

    // Numerical
    scalar _throttle_smooth_pos;        //! [c] Coefficient used to smooth throttle/brake

    // State Variables
    Timeseries_t _kappa_dimensionless_left;           //! [in] Angular speed of the left tire [rad/s]
    Timeseries_t _kappa_dimensionless_right;          //! [in] Angular speed of the right tire [rad/s]

    // State variables time derivatives
    Timeseries_t _dangular_momentum_dt_left;          //! [out] Angular acceleration of the left tire [rad/s2]
    Timeseries_t _dangular_momentum_dt_right;         //! [out] Angular acceleration of the right tire [rad/s2]

    // Engine/brake torque
    Timeseries_t _torque_left;          //! [out] Torque applied to the left tire
    Timeseries_t _torque_right;         //! [out] Torque applied to the right tire

    // Control variables
    Timeseries_t _throttle;             //! [in] Throttle/brake value in [-1/1]
    Timeseries_t _boost;                //! [in] Boost value in [-1/1]

    // Actuators
    Brake<Timeseries_t>  _brakes;       //! [c] Brakes model

    // Extra members for POWERED
    Engine<Timeseries_t> _engine;       //! [c] Engine model
    Engine<Timeseries_t> _engine_boost; //! [c] Engine model for the boost

    // Extra members for STEERING
    Timeseries_t _delta;                //! [in] Steering angle [rad]

    template<typename T = Axle_mode<0,0>>
    std::enable_if_t<std::is_same<T,POWERED<0,0>>::value,std::vector<Database_parameter_mutable>> 
    get_parameters() { return 
    { 
        { "track", _track },
        { "inertia", _I },
        { "smooth_throttle_coeff", _throttle_smooth_pos },
        { "differential_stiffness", _differential_stiffness }
    };}

    template<typename T = Axle_mode<0,0>>
    std::enable_if_t<std::is_same<T,POWERED<0,0>>::value,std::vector<Database_parameter_const>> 
    get_parameters() const { return 
    { 
        { "track", _track },
        { "inertia", _I },
        { "smooth_throttle_coeff", _throttle_smooth_pos },
        { "differential_stiffness", _differential_stiffness }
    };}

    template<typename T = Axle_mode<0,0>>
    std::enable_if_t<std::is_same<T,STEERING<0,0>>::value,std::vector<Database_parameter_mutable>> 
    get_parameters() { return 
    { 
        { "track", _track },
        { "inertia", _I },
        { "smooth_throttle_coeff", _throttle_smooth_pos }
    };}

    template<typename T = Axle_mode<0,0>>
    std::enable_if_t<std::is_same<T,STEERING<0,0>>::value,std::vector<Database_parameter_const>> 
    get_parameters() const { return 
    { 
        { "track", _track },
        { "inertia", _I },
        { "smooth_throttle_coeff", _throttle_smooth_pos }
    };}

    std::vector<bool> __used_parameters = std::vector<bool>(get_parameters().size(), false);

    std::unordered_map<std::string,Timeseries_t> get_outputs_map_self() const
    {
        return
        {
        };
    }
};

#include "axle_car_3dof.hpp"

#endif
