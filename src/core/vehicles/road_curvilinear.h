#ifndef __ROAD_CURVILINEAR_H__
#define __ROAD_CURVILINEAR_H__

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
    Road_curvilinear(const Track_t& track, scalar w);

    enum State { ITIME = base_type::STATE_END, IN, IALPHA, STATE_END };
    enum Controls { CONTROL_END = base_type::CONTROL_END };
    
    constexpr static size_t IIDTIME  = ITIME;
    constexpr static size_t IIDN     = IN;
    constexpr static size_t IIDALPHA = IALPHA;

    void change_track(const Track_t& track, scalar w) { _track = track; _w = w; }

    constexpr const scalar& track_length() const { return _track.get_total_length(); } 

    constexpr const scalar& track_width() const { return _w; }

    constexpr scalar curvature(const sVector3d& dr, const sVector3d& d2r, const scalar drnorm) const
                                                        { return cross(dr,d2r)[Z]/(drnorm*drnorm*drnorm); } 

    constexpr const Timeseries_t& get_n() const { return _n; } 

    constexpr const scalar& get_curvature() const { return _k; }

    constexpr const sVector3d& get_tangent() const { return _tan; }

    constexpr const sVector3d& get_normal() const { return _nor; }

    constexpr const scalar& get_heading_angle() const { return _theta; }

    constexpr const Track_t& get_track() const { return _track; }

    void update(const Timeseries_t u, const Timeseries_t v, const Timeseries_t omega);

    template<size_t N>
    void get_state_derivative(std::array<Timeseries_t,N>& dqdt) const;

    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_controls(const scalar t, const std::array<Timeseries_t,NSTATE>& q, const std::array<Timeseries_t,NCONTROL>& u);

    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_control_names(std::array<std::string,NSTATE>& q, std::array<std::string,NCONTROL>& u) const;

    void update_track(const scalar t);

 private:
    Track_t _track;     //! [in] Vectorial polynomial with track coordinates
    scalar _w;          //! [in] Track width

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


inline vPolynomial construct_ninety_degrees_bend(double L_end = 20.0)
{
    // Part 1: 20m straight
    scalar L1 = 20.0;
    std::vector<sVector3d> y1 = { {0.0,0.0,0.0}, {10.0, 0.0, 0.0}, {20.0,0.0,0.0} };
 
    // Part 2: 90 degrees bend with 10m radius
    scalar L2 = 10*pi/2.0;
    std::vector<sVector3d> y2;
    const size_t N2 = 10;
 
    const auto xi2 = std::get<0>(gauss_legendre_lobatto_nodes_and_weights(N2));

    for (size_t i = 0; i <= N2; ++i)
        y2.push_back(sVector3d(20.0+10.0*sin(pi/4.0*(xi2[i]+1.0)),10.0-10.0*cos(pi/4.0*(xi2[i]+1.0)),0.0));

    // Part 3: 20m straight
    scalar L3 = L_end;
    std::vector<sVector3d> y3 = { y2.back(), {30.0, 10.0+L_end*0.5, 0.0} , {30.0,10.0+L_end,0.0} };
 
    return {0.0, {L1,L2,L3}, {y1,y2,y3}};
}

static inline vPolynomial ninety_degrees_bend = construct_ninety_degrees_bend();




#include "road_curvilinear.hpp"


#endif
