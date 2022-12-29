#ifndef TRACK_BY_POLYNOMIAL_H
#define TRACK_BY_POLYNOMIAL_H

#include "lion/io/Xml_document.h"
#include "src/core/applications/circuit_preprocessor.h"
#include "lion/math/euler_angles.h"

class Track_by_polynomial
{
 public:

    Track_by_polynomial() = default;

    Track_by_polynomial(Xml_document& doc);

    //! Constructs a planar track from the position vector and track limits vectors
    Track_by_polynomial(const vPolynomial& position, const sPolynomial& wl, const sPolynomial& wr);

    Track_by_polynomial(const vPolynomial& position, const sPolynomial& yaw, const sPolynomial& pitch, const sPolynomial& roll,
        const sPolynomial& dyaw_ds, const sPolynomial& dpitch_ds, const sPolynomial& droll_ds, const sPolynomial& wl, const sPolynomial& wr) : _r(position), _theta(yaw), _mu(pitch), _phi(roll),
                                                                                                               _dtheta_ds(dyaw_ds), _dmu_ds(dpitch_ds), _dphi_ds(droll_ds), _wl(wl), _wr(wr) {}

    Track_by_polynomial(const Circuit_preprocessor& circuit_preprocessor);

    void set_left_track_limit_correction(const sPolynomial& wl_correction) { _wl_correction = wl_correction;  }

    void set_right_track_limit_correction(const sPolynomial& wr_correction) { _wr_correction = wr_correction;  }

    scalar get_left_track_limit(scalar s) const { return _wl(s) + _wl_correction(s); }

    scalar get_right_track_limit(scalar s) const { return _wr(s) + _wr_correction(s); }

    constexpr bool has_elevation() const;

    constexpr const scalar& get_total_length() const { return _r.get_right_bound(); } 

    struct Frenet_frame
    {
        sVector3d position;
        Euler_angles<scalar> euler_angles;
        Euler_angles<scalar> deuler_angles_ds;
    };

    Frenet_frame operator()(const scalar& t) const { return { _r(t), {_theta(t),_mu(t),_phi(t)}, {_dtheta_ds(t),_dmu_ds(t), _dphi_ds(t)} };  }

    template<typename Timeseries_t>
    Vector3d<Timeseries_t> position_at(const scalar t, const Timeseries_t& w)
    {
        const auto r = _r(t);
        const auto theta = _theta(t);
        const auto mu = _mu(t);
        const auto phi = _phi(t);

        const auto normal_vector = { cos(theta) * sin(mu) * sin(phi) - sin(theta) * cos(phi),
                                     sin(theta) * sin(mu) * sin(phi) + cos(theta) * cos(phi),
                                     cos(mu) * sin(phi) };
                
        return 
        {  
            r + w*normal_vector,
        };
    }

    const Circuit_preprocessor& get_preprocessor() const { return _preprocessor; }

 private:
    // Position vector
    vPolynomial _r;     //! Position vector polynomial

    // Frenet frame orientation
    sPolynomial _theta; //! Track heading angle [rad]
    sPolynomial _mu;    //! Track pitch angle [rad]
    sPolynomial _phi;   //! Track roll angle [rad]

    // Curvature
    sPolynomial _dtheta_ds; //! Kappa (yaw-rate) [rad/m]
    sPolynomial _dmu_ds;    //! pitch-rate [rad/m]
    sPolynomial _dphi_ds;   //! roll-rate [rad/m]

    sPolynomial _wl;    //! Distance to the left track limit [m]
    sPolynomial _wr;    //! Distance to the right track limit [m]

    sPolynomial _wl_correction; //! Apply a correction to the left track limit [m]
    sPolynomial _wr_correction; //! Apply a correction to the right track limit [m]

    Circuit_preprocessor _preprocessor;     //! Save the preprocessor used to compute this track
};

#include "track_by_polynomial.hpp"

#endif
