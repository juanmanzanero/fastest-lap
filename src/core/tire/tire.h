#ifndef __TIRE_H__
#define __TIRE_H__

#include "lion/foundation/types.h"
#include "lion/math/vector3d.h"
#include "lion/frame/frame.h"
#include "lion/io/Xml_document.h"
#include "lion/io/database_parameters.h"

//! Basic class defining tires
template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
class Tire
{
 public:
    using Timeseries_type = Timeseries_t;

    //! Indices of the state variables of this class: none
    enum State     { STATE_END    = STATE0 } ;

    //! Indices of the control variables of this class: none
    enum Controls  { CONTROL_END  = CONTROL0 } ;

    // Two tire types implemented: NORMAL (both lateral and longitudinal) and ONLY_LATERAL (only lateral)
    enum Type { NORMAL, ONLY_LATERAL };

    static Type get_type(const std::string& s){ return ( (s == "only lateral") ? ONLY_LATERAL : NORMAL ); }

    //! Default constructor
    Tire() = default;

    //! Constructor
    //! @param[in] name: name of the tire
    //! @param[in] parameters: map containing tire parameters
    //! @param[in] path: path to the tire on the parameters map
    //! @param[in] type: type of the tire (NORMAL or ONLY_LATERAL)
    Tire(const std::string& name, 
         Xml_document& database,
         const std::string& path="");

    //! Return the tire carcass radial stiffness [N/m]
    //! To be implemented by children classes
    constexpr scalar get_radial_stiffness() const;

    //! Updates omega, deformation, the contact point velocity, kappa and lambda
    //! But first, updates the position and velocity of the frame
    //! @param[in] x0: new position of the tire frame
    //! @param[in] v0: new velocity of the tire frame
    //! @param[in] omega: new value for tire angular speed
    void update(const Vector3d<Timeseries_t>& x0, const Vector3d<Timeseries_t>& v0, Timeseries_t omega);

    //! Updates omega, deformation, the contact point velocity, kappa and lambda
    //! @param[in] omega: new value for tire angular speed
    void update(Timeseries_t omega);

    //! Return the nominal radius of the tire [N]
    constexpr const scalar& get_radius() const { return _R0; }

    //! Returns the longitudinal slip [-]
    const Timeseries_t& get_kappa() const { return _kappa; }

    //! Returns the lateral slip [-]
    const Timeseries_t& get_lambda() const { return _lambda; }

    //! Returns the contact point velocity [m/s]
    const Vector3d<Timeseries_t>& get_velocity() const { return _v; }

    //! Get the forces torque at the wheel enter [N.m]
    const Timeseries_t get_longitudinal_torque_at_wheel_center() const { return -_F[X]*(_R0-_w); } 

    //! Get the contact point position [m]
    const Vector3d<Timeseries_t> get_contact_point() const { return {0.0, 0.0, _R0-_w}; }

    //! Get a reference to the frame
    const Frame<Timeseries_t>& get_frame() const { return _frame; }
          Frame<Timeseries_t>& get_frame()       { return _frame; }

    //! Get the vertical tire deformation [m]
    const Timeseries_t& get_vertical_deformation() const { return _w; }

    //! Get the vertical tire deformation velocity [m/s]
    const Timeseries_t& get_vertical_deformation_velocity() const { return _dw; }

    //! Get the tire forces [N]
    const Vector3d<Timeseries_t>& get_force() const { return _F; }

    //! Get the tire torques (applied at the wheel center) [N.m]
    const Vector3d<Timeseries_t>& get_torque() const { return _T; }

    //! Get the forces projected on the parent frame
    Vector3d<Timeseries_t> get_force_in_parent() const { return _frame.get_rotation_matrix()*_F; } 

    //! Get the torque projected on the parent frame
    Vector3d<Timeseries_t> get_torque_in_parent() const { return _frame.get_rotation_matrix()*_T; }

    //! Print the tire
    //! @param[inout] os: output stream
    std::ostream& print(std::ostream& os) const;

    static std::string type() { return "tire"; }

 private:
   
    //! Compute the longitudinal slip
    Timeseries_t kappa() const { return (_omega*_R0-_v[0])/_v[0]; } 

    //! Compute the lateral slip
    Timeseries_t lambda() const { return -_v[1]/_v[0]; }

    std::string _name;  //! [c] A string to identify the tire (e.g. front left)
    Type        _type;  //! [c] Tire type: normal or lateral_only

    Frame<Timeseries_t>  _frame;  //! [in] A frame with origin in the tire road contact point 
    scalar _R0;     //! [c] Tire nominal radius [m]

    scalar _um = 0.2;   //! [c] Marginal velocity, the denominator of kappa and lambda for low velocities

    std::vector<Database_parameter> get_parameters() { return 
    { 
        { "radius", _R0 }
    };}

 protected:

    // Inputs
    Timeseries_t _omega;  //! [in] Tire angular velocity [rad/s]

    // Outputs
    Timeseries_t _w;      //! [out] Tire vertical deformation: w = R0 + zC [m]
    Timeseries_t _dw;     //! [out] Tire vertical deformation derivative   [m/s]
    Vector3d<Timeseries_t>  _v;      //! [out] Absolute velocity of the contact point in body frame [m/s].

    Timeseries_t _kappa;  //! [out] Longitudinal slip  ((omega.R0)/v[0] - 1)
    Timeseries_t _lambda; //! [out] Sideslip angle = -atan(v[1]/v[0])

    Vector3d<Timeseries_t> _F;       //! [out] Tire forces [N]
    Vector3d<Timeseries_t> _T;       //! [out] Tire torques at wheel centre [N.m]
};

template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
inline std::ostream& operator<<(std::ostream& os, const Tire<Timeseries_t,STATE0,CONTROL0>& tire){return tire.print(os);}

#include "tire.hpp"

#endif
