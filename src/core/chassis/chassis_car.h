#ifndef __CHASSIS_CAR_H__
#define __CHASSIS_CAR_H__

#include "chassis.h"
#include <array>

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
class Chassis_car : public Chassis<Timeseries_t,FrontAxle_t, RearAxle_t, STATE0,CONTROL0>
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
    enum State     
    { 
        IZ = base_type::STATE_END,  //! Vertical displacement of the chassis [m]
        IPHI,                       //! Roll angle (assumed small) [rad]
        IMU,                        //! Pitch angle (assumed small) [rad]
        IDZ,                        //! Vertical displacement derivative [m/s]
        IDPHI,                      //! Roll angular speed [rad/s]
        IDMU,                       //! Pitch angular speed [rad/s]
        STATE_END
    };

    //! Control variables: none
    enum Controls  { CONTROL_END = base_type::CONTROL_END};

    constexpr static size_t IIDZ    = IZ;    //! Vertical velocity [m/s]
    constexpr static size_t IIDPHI  = IPHI;  //! Roll angular speed [rad/s]
    constexpr static size_t IIDMU   = IMU;   //! Pitch angular speed [rad/s]
    constexpr static size_t IID2Z   = IDZ;   //! Vertical acceleration [m/s2]
    constexpr static size_t IID2PHI = IDPHI; //! Roll acceleration [rad/s2]
    constexpr static size_t IID2MU  = IDMU;  //! Pitch acceleration [rad/s2]

    //! Default constructor
    Chassis_car() = default;

    //! Constructor from axles and extra parameters
    //! @param[in] front_axle: Front axle
    //! @param[in] rear_axle: Rear axle
    //! @param[in] parameters: map containing mechanical/geometrical parameters by name
    //! @param[in] path: path to find its parameters on the map
    Chassis_car(const FrontAxle_t& front_axle, 
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
    Chassis_car(Xml_document& database);

    //! Update the chassis: update the axles to get forces and compute accelerations
    //! @param[in] x: x-coordinate of the road frame [m]
    //! @param[in] y: y-coordinate of the road frame [m]
    //! @param[in] psi: yaw angle of the road frame [rad]
    void update(Timeseries_t x, Timeseries_t y, Timeseries_t psi);

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
    void get_state_derivative(std::array<Timeseries_t,N>& dqdt) const;

    //! Set the state variables of this class
    //! @param[in] q: the vehicle state vector 
    //! @param[in] u: the vehicle control vector
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_controls(const std::array<Timeseries_t,NSTATE>& q, 
                                const std::array<Timeseries_t,NCONTROL>& u);

    //! Get the names of the state and control varaibles of this class
    //! @param[out] q: the vehicle state names
    //! @param[out] u: the vehicle control names
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_control_names(std::array<std::string,NSTATE>& q, 
                                     std::array<std::string,NCONTROL>& u) const;

    static std::string type() { return "chassis_car"; }
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
    Timeseries_t _z;      //! [in] Vertical displacement of CoG [m]
    Timeseries_t _dz;     //! [in] Vertical velocity of CoG [m/s]
    Timeseries_t _d2z;    //! [out] Vertical acceleration of CoG [m/s2]

    // Small angle attitude: pitch
    Timeseries_t _mu;     //! [in] Pitch angle [rad]
    Timeseries_t _dmu;    //! [in] Pitch omega [rad/s]
    Timeseries_t _d2mu;   //! [out] Pitch acceleration [rad/s2]

    // Small angle attitude: roll
    Timeseries_t _phi;    //! [in] Roll angle [rad]
    Timeseries_t _dphi;   //! [in] Roll omega [rad/s]
    Timeseries_t _d2phi;  //! [out] Roll acceleration [rad/s2]


    std::vector<Database_parameter> get_parameters() { return 
    { 
        { "com", _x_com },
        { "front_axle", _x_front_axle },
        { "rear_axle", _x_rear_axle },
    };}
};

#include "chassis_car.hpp"


#endif
