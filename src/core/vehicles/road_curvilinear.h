#ifndef ROAD_CURVILINEAR_H
#define ROAD_CURVILINEAR_H

#include "road.h"
#include "lion/math/polynomial.h"
#include "lion/math/matrix_extensions.h"

template<typename Timeseries_t,typename Track_t,size_t STATE0, size_t CONTROL0>
class Road_curvilinear : public Road<Timeseries_t,STATE0,CONTROL0>
{
 public:
    using base_type  = Road<Timeseries_t,STATE0,CONTROL0>;
    using Track_type = Track_t;

    Road_curvilinear() = default;
    Road_curvilinear(const Track_t& track);

    struct input_state_names
    {
        enum { TIME = base_type::input_state_names::end, N, ALPHA, end };
    };

    struct state_names
    {
        enum { TIME  = input_state_names::TIME, 
               N     = input_state_names::N,
               ALPHA = input_state_names::ALPHA 
             };
    };

    struct control_names
    {
        enum { end = base_type::control_names::end };
    };

    void change_track(const Track_t& track) { _track = track; }

    constexpr const scalar& track_length() const { return _track.get_total_length(); } 

    constexpr const scalar get_left_track_limit(scalar s) const { return _track.get_left_track_limit(s); }

    constexpr const scalar get_right_track_limit(scalar s) const { return _track.get_right_track_limit(s); }

    constexpr scalar curvature(const sVector3d& dr, const sVector3d& d2r, const scalar drnorm) const
                                                        { return cross(dr,d2r)[Z]/(drnorm*drnorm*drnorm); } 

    constexpr const Timeseries_t& get_n() const { return _n; } 

    constexpr const Timeseries_t& get_alpha() const { return _alpha; } 

    constexpr const scalar& get_curvature() const { return _k; }

    constexpr const sVector3d& get_tangent() const { return _tan; }

    constexpr const sVector3d& get_normal() const { return _nor; }

    constexpr const scalar& get_heading_angle() const { return _theta; }

    constexpr const Track_t& get_track() const { return _track; }

    void update(const Timeseries_t u, const Timeseries_t v, const Timeseries_t omega);

    template<size_t N>
    void get_state_and_state_derivative(std::array<Timeseries_t,N>& states, std::array<Timeseries_t,N>& dstates_dt) const;

    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_controls(const scalar t, 
                                const std::array<Timeseries_t,NSTATE>& input_states, 
                                const std::array<Timeseries_t,NCONTROL>& controls);

    //! Set the state and controls upper, lower, and default values
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_control_upper_lower_and_default_values(std::array<scalar,NSTATE>& input_states_def,
                                                              std::array<scalar,NSTATE>& input_states_lb,
                                                              std::array<scalar,NSTATE>& input_states_ub,
                                                              std::array<scalar,NCONTROL>& controls_def,
                                                              std::array<scalar,NCONTROL>& controls_lb,
                                                              std::array<scalar,NCONTROL>& controls_ub
                                                              ) const;

    template<size_t NSTATE, size_t NCONTROL>
    static void set_state_and_control_names(std::string& key_name, 
                                            std::array<std::string,NSTATE>& input_state_names, 
                                            std::array<std::string,NCONTROL>& control_names
                                           );

    void update_track(const scalar t);

 private:
    Track_t _track;     //! [in] Vectorial polynomial with track coordinates

    sVector3d _r;
    sVector3d _dr;
    sVector3d _d2r;

    scalar _rnorm;
    scalar _drnorm;

    sVector3d _tan;
    sVector3d _bi;
    sVector3d _nor;

    scalar _k;
    scalar _theta;


    Timeseries_t _time;  //! The simulation time
    Timeseries_t _n;     //! The normal distance to the road centerline
    Timeseries_t _alpha; //! Vehicle angle wrt the centerline

    Timeseries_t _dtime;
    Timeseries_t _dn;
    Timeseries_t _dalpha;
};

#include "road_curvilinear.hpp"

#endif
