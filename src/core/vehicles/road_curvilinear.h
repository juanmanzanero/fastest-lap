#ifndef ROAD_CURVILINEAR_H
#define ROAD_CURVILINEAR_H

#include "road.h"
#include "lion/math/polynomial.h"
#include "lion/math/matrix_extensions.h"
#include "lion/math/euler_angles.h"

template<typename Timeseries_t,typename Track_t,size_t state_start, size_t control_start>
class Road_curvilinear : public Road<Timeseries_t,state_start,control_start>
{
 public:
    using base_type  = Road<Timeseries_t,state_start,control_start>;
    using Track_type = Track_t;

    struct input_names : public base_type::input_names
    {
        enum { time = base_type::input_names::end, lateral_displacement, track_heading_angle, end };
    };

    struct state_names : public base_type::state_names
    {
        enum { time = base_type::state_names::end, lateral_displacement, track_heading_angle, end };
    };

    static_assert(static_cast<size_t>(input_names::end) == static_cast<size_t>(state_names::end));

    struct control_names : public base_type::control_names {};

    //! Default constructor
    Road_curvilinear() = default;

    //! Constructor from given track object
    Road_curvilinear(const Track_t& track) : _track(track) {}

    //! Change the track to an existing road object
    void change_track(const Track_t& track) { _track = track; }

    //! Get the track length
    constexpr const scalar& track_length() const { return _track.get_total_length(); } 

    //! Get the value of the left track limit at a given arclenght
    constexpr const scalar get_left_track_limit(scalar s) const { return _track.get_left_track_limit(s); }

    //! Get the value of the right track limit at a given arclenght
    constexpr const scalar get_right_track_limit(scalar s) const { return _track.get_right_track_limit(s); }

    //! Compute the 3D track curvature (in body axes) 
    constexpr sVector3d curvature(const Euler_angles<scalar>& euler_angles, const Euler_angles<scalar>& deuler_angles_ds) const
    {
        const auto& pitch     = euler_angles.pitch();
        const auto& roll      = euler_angles.roll();
        const auto& dyaw_ds   = deuler_angles_ds.yaw();
        const auto& dpitch_ds = deuler_angles_ds.pitch();
        const auto& droll_ds  = deuler_angles_ds.roll();

        return { droll_ds - sin(pitch) * dyaw_ds,
                 cos(roll) * dpitch_ds + cos(pitch) * sin(roll) * dyaw_ds,
                 -sin(roll) * dpitch_ds + cos(pitch) * cos(roll) * dyaw_ds };
    }

    constexpr const auto& get_lateral_displacement() const { return _lateral_displacement_mps; } 

    constexpr const auto& get_track_heading_angle() const { return _track_heading_angle_rad; } 

    constexpr const auto& get_track_heading_angle_dot() const { return _track_heading_angle_dot_radps; } 

    constexpr const auto& get_curvature() const { return _curvature; }

    constexpr const auto& get_tangent_vector() const { return _tangent_vector; }

    constexpr const auto& get_normal_vector() const { return _normal_vector; }

    constexpr const auto& get_binormal_vector() const { return _binormal_vector; }

    constexpr const scalar& get_heading_angle() const { return _theta; }

    const Euler_angles<scalar> get_euler_angles() const { return { _theta, _mu, _phi }; }

    const Euler_angles<Timeseries_t> get_euler_angles_dot() const { return { _dtheta_ds/base_type::get_dtimeds(), _dmu_ds/base_type::get_dtimeds(), _dphi_ds/base_type::get_dtimeds()}; }

    constexpr const Track_t& get_track() const { return _track; }

    //! Compute the time derivatives
    //! @param[in] velocity_x_body: x-velocity of the body frame
    //! @param[in] velocity_y_body: y-velocity of the body frame
    //! @param[in] yaw_rate: z-component of the body frame rate
    void update(const Timeseries_t& velocity_x_body, const Timeseries_t& velocity_y_body, const Timeseries_t& yaw_rate);

    template<size_t number_of_states>
    void get_state_and_state_derivative(std::array<Timeseries_t,number_of_states>& states, 
                                        std::array<Timeseries_t,number_of_states>& dstates_dt
                                        ) const;


    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_controls(const scalar t, 
                                const std::array<Timeseries_t,number_of_inputs>& inputs, 
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

    template<size_t number_of_inputs, size_t number_of_controls>
    static void set_state_and_control_names(std::string& key_name, 
                                            std::array<std::string,number_of_inputs>& input_names, 
                                            std::array<std::string,number_of_controls>& control_names
                                           );

    void update_track(const scalar t);

 private:
    Track_t _track;     //! [in] Vectorial polynomial with track coordinates

    sVector3d _position = { 0.0, 0.0, 0.0 }; //! Position along the centerline [m]

    scalar _theta = 0.0;   //! Yaw angle of the Frenet frame [rad]
    scalar _mu = 0.0;      //! Pitch angle of the Frenet frame [rad]
    scalar _phi = 0.0;     //! Roll angle of the Frenet frame [rad]

    scalar _dtheta_ds = 0.0; //! Spatial derivative of the yaw angle [rad/m]
    scalar _dmu_ds = 0.0; //! Spatial derivative of the pitch angle [rad/m]
    scalar _dphi_ds = 0.0; //! Spatial derivative of the roll angle [rad/m]

    sVector3d _tangent_vector = { 0.0, 0.0, 0.0 };      //! Tangent vector of the Frenet frame
    sVector3d _normal_vector = { 0.0, 0.0, 0.0 };       //! Normal vector of the Frenet frame
    sVector3d _binormal_vector = { 0.0, 0.0, 0.0 };     //! Binormal vector of the Frenet frame

    sVector3d _curvature = { 0.0, 0.0, 0.0 };           //! 3D curvature vector

    Timeseries_t _time = 0.0;                     //! The simulation time [s]
    Timeseries_t _lateral_displacement_mps = 0.0; //! The normal distance to the road centerline [m]
    Timeseries_t _track_heading_angle_rad = 0.0;  //! Vehicle angle wrt the centerline [rad]

    Timeseries_t _lateral_displacement_dot_mps = 0.0;  //! Time derivative of the lateral displacement [m/s]
    Timeseries_t _track_heading_angle_dot_radps = 0.0; //! Time derivative of the track heading angle [rad/s]
};

template<typename Timeseries_t,typename Track_t,size_t state_start, size_t control_start>
struct road_is_curvilinear<Road_curvilinear<Timeseries_t, Track_t, state_start, control_start>>
{
    const static inline bool value = true;
};

#include "road_curvilinear.hpp"

#endif
