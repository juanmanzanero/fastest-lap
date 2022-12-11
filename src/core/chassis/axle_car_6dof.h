#ifndef AXLE_CAR_6DOF_H
#define AXLE_CAR_6DOF_H

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
//! * The position of the tires are (0,y_tire,s + phi.y_tire + beta.delta), 
//!   where y_tire is the constant y position of the tires, s is the chassis deformation
//!   (i.e. tires center can have a displacement due to chassis compliance),
//!   phi is the small roll angle, and beta.delta is a dz as a result of the steering mechanism
//! * The velocity of the tires is (0,0,ds + dphi.y_tire). Movement induced due to ddelta has
//!   not been included yet
//! 

//! Types of car axles implemented:
//!
//! 1) Powered without differential: both tires have the same rotation speed,
//!     and a torque T is imposed by the driver
//!
//! 2) Steering with free roll: we don't impose any condition on omega (i.e.
//!     the tires are free to roll), and the tires plane can rotate around Z (steering)
//!

//! State and control variables for Powered (without differential) axles
//!  @param state_start: index of the first state variable defined here
//!  @param control_start: index of the first control variable defined here
template<size_t state_start, size_t control_start>
struct POWERED_WITHOUT_DIFFERENTIAL
{
    //! State variables
    struct input_names 
    {
        enum
        {
            OMEGA_AXLE = state_start,     //! Axle angular speed [rad/s]
            end
        };    
    } ;

    struct state_names
    {
        enum
        {
            OMEGA_AXLE = input_names::OMEGA_AXLE,
            end
        };
    };

    //! Control variables
    struct control_names
    {
        enum
        {
            TORQUE = control_start,    //! Torque at the axle [Nm]
            end
        };
    };
};

//! State and control variables for Steering with free roll axles
//!  @param state_start: index of the first state variable defined here
//!  @param control_start: index of the first control variable defined here
template<size_t state_start, size_t control_start>
struct STEERING_FREE_ROLL
{
    //! State variables: none
    struct input_names
    {
        enum { end = state_start };
    };
    //! Control variables
    struct control_names  
    { 
        enum
        {
            STEERING = control_start,    //! Steering angle [rad]
            end
        };
    };

    struct state_names
    {
        enum
        {
            end = state_start
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
class Axle_car_6dof : public Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>, 
    public Axle_mode<Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::input_names::end, 
                                  Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::control_names::end>
{
 public:

    //! The parent axle class
    using base_type       = Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>;

    //! The axle sub type (POWERED_WITHOUT_DIFFERENTIAL/STEERING_FREE_ROLL)
    using Axle_type = Axle_mode<Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::input_names::end,
                                Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::control_names::end>;

    struct input_names : public Axle_type::input_names, base_type::input_names 
    {
        constexpr const static size_t end = Axle_type::input_names::end;
    };

    struct control_names : public Axle_type::control_names, base_type::control_names 
    {
        constexpr const static size_t end = Axle_type::control_names::end;
    };

    struct state_names : public Axle_type::state_names, base_type::state_names 
    {
        constexpr const static size_t end = Axle_type::state_names::end;
    };

    static_assert(input_names::end == state_names::end);

    //! The left tire type
    using Tire_left_type  = Tire_left_t;

    //! The right tire type
    using Tire_right_type = Tire_right_t;

    //! The two tires: left and right
    enum Tires : size_t { LEFT, RIGHT };

    //! Default constructor
    Axle_car_6dof() = default;

    Axle_car_6dof(const std::string& name, 
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
    Axle_car_6dof(const std::string& name, 
             const Tire_left_t& tire_l,
             const Tire_right_t& tire_r,
             Xml_document& database,
             const std::string& path=""
            );


    template<size_t number_of_inputs, size_t number_of_controls>
    void transform_states_to_inputs(const std::array<Timeseries_t,number_of_inputs>& states,
                                          const std::array<Timeseries_t,number_of_controls>& controls,
                                          std::array<Timeseries_t,number_of_inputs>& inputs) {}

    // Functions for POWERED_WITHOUT_DIFFERENTIAL
    
    //! Set torque and omega: _T_ax = T_ax, _omega = omega
    //! @param[in] T_ax: torque applied to the axle by engine/brake [Nm]
    //! @param[in] omega: angular speed of the axle [rad/s]
    template<typename T = Axle_mode<0,0>>
    std::enable_if_t<std::is_same<T,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value,void> 
        set_torque_and_omega(Timeseries_t T_ax, Timeseries_t omega);

    //! Set throttle and omega
    //! @param[in] throttle: throttle/brake percentage [-]
    //! @param[in] omega: angular speed of the axle [rad/s]
    template<typename T = Axle_mode<0,0>>
    std::enable_if_t<std::is_same<T,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value,void> 
        set_throttle_and_omega(Timeseries_t throttle, Timeseries_t omega);

    // Functions for STEERING_FREE_ROLL

    //! Set the steering angle
    //!
    //! @param[in] delta: the steering angle [rad]
    template<typename T = Axle_mode<0,0>>
    std::enable_if_t<std::is_same<T,STEERING_FREE_ROLL<0,0>>::value,void> 
        set_steering_angle(Timeseries_t delta);

    //! Updates the axle: 
    //! Pre-requisites: call to set_torque_and_omega and set_steering_angle
    //! --------------
    //!     -> delta, T, and omega are up to date
    //!
    //!     1) Moves the frame to (x0,v0)
    //!     2) Computes new tire deformations. This is the result of solving the system:
    //!         wl = zsym - zasym + sl
    //!         wr = zsym + zasym + sr
    //!         k_chassis sl + k_antiroll (sl - sr) + k_tire wl = 0
    //!         k_chassis sr + k_antiroll (sr - sl) + k_tire wr = 0
    //!         
    //!         Where zsym is a symmetric vertical displacement (of the whole axle), e.g. zsym = z + a.mu
    //!         and zasym is an assymmetric vertical displacement (one tire rises and the other lowers), 
    //!         e.g. zasym = 0.5.phi.track + beta.delta
    //!
    //!     3) Updates the tires (gets the new forces)
    //!     4) Updates the axle equation if POWERED_WITHOUT_DIFFERENTIAL
    //!     5) Reduces the tire forces to force+torque at the axle frame origin
    //!
    //! @param[in] x0: new frame origin [m]
    //! @param[in] v0: new frame velocity [m/s]
    //! @param[in] phi: roll angle, assumed small [rad]
    //! @param[in] dphi: roll angle derivative [rad/s]
    void update(const Vector3d<Timeseries_t>& x0, const Vector3d<Timeseries_t>& v0, Timeseries_t phi, Timeseries_t dphi);

    //! Performs an update (see above) skipping the frame movement step (i.e. keeps its
    //! previous position and velocity and only updates roll angle)
    //!
    //! @param[in] phi: roll angle, assumed small [rad]
    //! @param[in] dphi: roll angle derivative [rad/s]
    void update(Timeseries_t phi, Timeseries_t dphi);

    //! Get the steering angle [rad]
    const Timeseries_t& get_steering_angle() const { return _delta; }

    //! Get the chassis deformation [m]
    //! @param[in] i: which tire (LEFT/RIGHT)
    const Timeseries_t& get_chassis_deformation(Tires i) const { return _s.at(i); }

    //! Get the chassis deformation velocity [m/s]
    //! @param[in] i: which tire (LEFT/RIGHT)
    const Timeseries_t& get_chassis_deformation_velocity(size_t i) const { return _ds.at(i); }

    //! Get the axle torque [Nm]
    const Timeseries_t& get_axle_torque() const { return _T_ax; }

    //! Get the axle angular speed [rad/s]
    const Timeseries_t& get_omega() const { return _omega; }

    //! Get the axle angular acceleration [rad/s2]
    const Timeseries_t& get_omega_derivative() const { return _domega; } 

    //! Get the tire position (in axle frame)
    //! @param[in] tire: which tire (LEFT/RIGHT)
    const Vector3d<Timeseries_t> get_tire_position(Tires tire) const { 
                        return {0.0, _y_tire[tire], _s[tire] + _phi*_y_tire[tire] + _beta[tire]*_delta }; }

    //! Get the tire position y-position (in axle frame)
    //! @param[in] tire: which tire (LEFT/RIGHT)
    const scalar get_tire_y_position(Tires tire) const { return _y_tire[tire]; }

    //! Get the tire relative velocity (in axle frame)
    //! @param[in] tire: which tire (LEFT/RIGHT)
    const Vector3d<Timeseries_t> get_tire_velocity(Tires tire) const { 
                        return {0.0, 0.0, _ds[tire] + _dphi*_y_tire[tire]  }; }

    //! Return the track (the distance between the two tires)
    const scalar& get_track() const { return _track; }

    //! Get a certain tire mechanical or geometrical parameter by name
    //! @param[in] parameter_name: name of the parameter
    scalar get_parameter(const std::string& parameter_name) const;

    //! Load the time derivative of the state variables computed herein to the dqdt
    //! @param[out] dqdt: the vehicle state vector time derivative
    template<size_t number_of_states>
    void get_state_and_state_derivative(std::array<Timeseries_t,number_of_states>& state, std::array<Timeseries_t, number_of_states>& dstate_dt, const Timeseries_t& mass_kg) const;

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
                                                               std::array<scalar,number_of_controls>& controls_def,
                                                               std::array<scalar,number_of_controls>& controls_lb,
                                                               std::array<scalar,number_of_controls>& controls_ub 
                                                              ) const;

    //! Get the names of the state and control varaibles of this class
    //! @param[out] q: the vehicle state names
    //! @param[out] u: the vehicle control names
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_names(std::array<std::string,number_of_inputs>& inputs, 
                                     std::array<std::string,number_of_controls>& controls) const;

    static std::string type() { return "axle_car"; }

    //! Enable direct torque
    void enable_direct_torque() { _engine.direct_torque() = true; }

    constexpr bool is_direct_torque() const { return _engine.direct_torque() == true; }

    bool is_ready() const { return base_type::is_ready() && 
        std::all_of(__used_parameters.begin(), __used_parameters.end(), [](const auto& v) -> auto { return v; }); }

    std::unordered_map<std::string,Timeseries_t> get_outputs_map() const
    {
        auto map = get_outputs_map_self();
        const auto base_type_map = base_type::get_outputs_map();

        map.insert(base_type_map.cbegin(), base_type_map.cend());

        return map;
    }

 private:
    // Geometry
    scalar _track;                //! [c] Distance between the two tires [m]
    std::array<scalar,2> _y_tire; //! [c] y position of the two tires [m]

    // Mechanical
    scalar _k_chassis;  //! [c] Chassis stiffness [N/m]
    scalar _k_antiroll; //! [c] Antiroll bar stiffness [N/m]

    // Numerical
    scalar _throttle_smooth_pos; //! [c] Coefficient used to smooth throttle/brake

    Timeseries_t _phi;   //! [in] Angle rotated by the axle in the X direction [rad]
    Timeseries_t _dphi;  //! [in] Omega of the angle rotated by the axle in the X direction [rad/s]

    std::array<Timeseries_t,2> _s;  //! [out] Chassis deformations [m]
    std::array<Timeseries_t,2> _ds; //! [out] Chassis deformations velocity [m/s]

    // Parameters for POWERED_WITHOUT_DIFFERENTIAL
    Timeseries_t _omega;    //! [in] Angular speed of the shaft [rad/s]
    Timeseries_t _throttle; //! [in] Throttle/brake value in [-1/1]
    Timeseries_t _T_ax;     //! [in] Torque applied to the axle [Nm]
    Timeseries_t _domega;   //! [out] Angular acceleration of the shaft [rad/s2]
    scalar _I;            //! [c] Inertia of all parts connected to the shaft [kg m2]
    Engine<Timeseries_t> _engine;       //! [c] Engine model
    Brake<Timeseries_t>  _brakes;       //! [c] Brakes model

    // Parameters for STEERING_FREE_ROLL
    Timeseries_t _delta;           //! [in] Steering angle [rad]
    std::array<scalar,2> _beta;  //! [c] dz in the tires due to delta (dz/d(delta)) [m/rad]. 
                                 //! Basically, interior tire lowers and the exterior rises

    template<typename T = Axle_mode<0,0>>
    std::enable_if_t<std::is_same<T,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value,std::vector<Database_parameter_mutable>> 
    get_parameters() { return 
    { 
        { "track", _track },
        { "stiffness/chassis", _k_chassis },
        { "stiffness/antiroll", _k_antiroll },
        { "inertia", _I },
        { "smooth_throttle_coeff", _throttle_smooth_pos }
    };}

    template<typename T = Axle_mode<0,0>>
    std::enable_if_t<std::is_same<T,STEERING_FREE_ROLL<0,0>>::value,std::vector<Database_parameter_mutable>> 
    get_parameters() { return 
    { 
        { "track", _track },
        { "stiffness/chassis", _k_chassis },
        { "stiffness/antiroll", _k_antiroll },
        { "beta-steering/left", _beta[LEFT] },
        { "beta-steering/right", _beta[RIGHT] }
    };}

    std::vector<bool> __used_parameters = std::vector<bool>(get_parameters().size(), false);

    template<typename T = Axle_mode<0,0>>
    std::enable_if_t<std::is_same<T,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value,std::unordered_map<std::string,Timeseries_t>> 
    get_outputs_map_self() const
    {
        return
        {
        };
    }

    template<typename T = Axle_mode<0,0>>
    std::enable_if_t<std::is_same<T,STEERING_FREE_ROLL<0,0>>::value,std::unordered_map<std::string,Timeseries_t>> 
    get_outputs_map_self() const
    {
        return {};
    }
};

#include "axle_car_6dof.hpp"

#endif
