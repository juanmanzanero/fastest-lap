#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "lion/math/matrix3x3.h"
#include "lion/frame/frame.h"
#include <map>
#include "lion/io/database_parameters.h"

//!   Generic chassis type
//!   --------------------
//!
//! This class stores and provides interface with the front and rear axles
//!  @param FrontAxle_t: type of the front axle
//!  @param RearAxle_t: type of the rear axle
//!  @param STATE0: index of the first state variable defined here
//!  @param CONTROL0: index of the first control variable defined here
template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
class Chassis
{
 public:
    using Timeseries_type = Timeseries_t;

    //! Type of the front axle
    typedef FrontAxle_t front_axle_type;

    //! Type of the rear axle
    typedef RearAxle_t rear_axle_type;

    //! State variables: the basic 3 dof - longitudinal/lateral velocity + yaw
    enum State     
    { 
        IU = STATE0,    //! Longitudinal velocity (in road frame) [m/s]
        IV,             //! Lateral velocity (in road frame) [m/s]
        IOMEGA,         //! Yaw speed [rad/s]
        STATE_END    
    } ;

    //! Control variables: none
    enum Controls  { CONTROL_END  = CONTROL0  } ;
    
    constexpr static size_t IIDU = IU;          //! Longitudinal acceleration (in road frame) [m/s2]
    constexpr static size_t IIDV = IV;          //! Lateral acceleration (in road frame) [m/s2]
    constexpr static size_t IIDOMEGA = IOMEGA;  //! Yaw acceleration [rad/s2]

    //! Default constructor
    Chassis() = default;

    //! Empty constructor
    Chassis(const FrontAxle_t& front_axle, 
            const RearAxle_t& rear_axle,
            const std::string& path=""
           );

    //! Constructor from axles and mechanical/geometrical parameters
    //! @param[in] front_axle: Front axle object
    //! @param[in] rear_axle: Rear axle object
    //! @param[in] parameters: map with mechanical/geometrical parameters by name
    //! @param[in] path: path to the parameters of the chassis in the map
    Chassis(const FrontAxle_t& front_axle, 
            const RearAxle_t& rear_axle,
            Xml_document& database,
            const std::string& path=""
           );

    //! Copy constructor
    Chassis(const Chassis& other);

    //! Copy assignment
    Chassis& operator=(const Chassis& other);

    //! Modifyer to set a parameter
    template<typename T>
    void set_parameter(const std::string& parameter, const T value);

    //! Set state: longitudinal/lateral velocities and yaw speed
    //! @param[in] u: longitudinal velocity in road frame [m/s]
    //! @param[in] v: lateral velocity in road frame [m/s]
    //! @param[in] omega: yaw speed [rad/s]
    void set_state(Timeseries_t u, Timeseries_t v, Timeseries_t omega);

    //! Get the longitudinal velocity [m/s]
    const Timeseries_t& get_u() const { return _u; }

    //! Get the longitudinal acceleration [m/s2]
    const Timeseries_t& get_du() const { return _du; }

    //! Get the lateral velocity [m/s]
    const Timeseries_t& get_v() const { return _v; }

    //! Get the lateral acceleration [m/s2]
    const Timeseries_t& get_dv() const { return _dv; }

    //! Get the yaw speed [rad/s]
    const Timeseries_t& get_omega() const { return _omega; }

    //! Get the yaw acceleration [rad/s2]
    const Timeseries_t& get_domega() const { return _dOmega; }

    //! Get the chassis mass [kg]
    constexpr const scalar& get_mass() const { return _m; } 

    //! Get the chassis inertia matrix [kg.m2]
    constexpr const sMatrix3x3& get_inertia() const { return _I; }

    //! Get the front axle
    constexpr const FrontAxle_t& get_front_axle() const { return _front_axle; }
    constexpr       FrontAxle_t& get_front_axle()       { return _front_axle; }

    //! Get the rear axle
    constexpr const RearAxle_t& get_rear_axle() const { return _rear_axle; }
    constexpr       RearAxle_t& get_rear_axle()       { return _rear_axle; }

    //! Get a reference to the inertial frame
    constexpr const Frame<Timeseries_t>& get_inertial_frame() const { return _inertial_frame; }
    constexpr       Frame<Timeseries_t>& get_inertial_frame()       { return _inertial_frame; }

    //! Get a reference to the road frame
    constexpr const Frame<Timeseries_t>& get_road_frame() const { return _road_frame; }
    constexpr       Frame<Timeseries_t>& get_road_frame()       { return _road_frame; }

    //! Get a reference to the chassis frame
    constexpr const Frame<Timeseries_t>& get_chassis_frame() const { return _chassis_frame; }
    constexpr       Frame<Timeseries_t>& get_chassis_frame()       { return _chassis_frame; }

    //! Get the total force acting on the chassis CoM [N]
    constexpr const Vector3d<Timeseries_t>& get_force() const { return _F; }

    //! Get the total torque acting on the chassis CoM [Nm]
    constexpr const Vector3d<Timeseries_t>& get_torque() const { return _T; }

    //! Get the aerodynamic force [N]
    Vector3d<Timeseries_t> get_aerodynamic_force() const; 

    //! Add an additional external force
    //! @param[in] F: force to be added [N]
    constexpr void set_external_force(const Vector3d<Timeseries_t>& F) { _Fext = F; } 

    //! Add an additional external torque
    //! @param[in] T: torque to be added [Nm]
    constexpr void set_external_torque(const Vector3d<Timeseries_t>& T) { _Text = T; } 

    //! Perform an update of the class: move the road frame to the new position
    //! @param[in] x: x-position of the road frame [m]
    //! @param[in] y: y-position of the road frame [m]
    //! @param[in] psi: yaw angle of the road frame [rad]
    void update(Timeseries_t x, Timeseries_t y, Timeseries_t psi);

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
    static void set_state_and_control_names(std::array<std::string,NSTATE>& q, 
                                     std::array<std::string,NCONTROL>& u);

    static std::string type() { return "chassis"; }


 private:

    // Road frame velocities
    Timeseries_t _u;         //! [in] Longitudinal velocity (in road frame) [m/s]
    Timeseries_t _v;         //! [in] Lateral velocity (in road frame) [m/s]
    Timeseries_t _omega;     //! [in] Yaw speed [rad/s]

    Frame<Timeseries_t> _inertial_frame; //! The inertial frame
    Frame<Timeseries_t> _road_frame;     //! Frame<Timeseries_t> with center on the road projection of the CoG, 
                           //! aligned with the chassis
    Frame<Timeseries_t> _chassis_frame;  //! Frame<Timeseries_t> with center on the CoG and parallel axes to the road frame

    // Mass properties
    scalar _m;     //! [c] chassis mass [kg]
    sMatrix3x3 _I; //! [c] chassis inertia matrix [kg.m2]

    // Aerodynamic properties
    scalar _rho;   //! [c] air density [kg/m3]
    scalar _cd;    //! [c] drag coefficient [-]      
    scalar _cl;    //! [c] lift coefficient [-]      
    scalar _A;     //! [c] frontal area [m2]
    
    FrontAxle_t _front_axle; //! Front axle
    RearAxle_t  _rear_axle;  //! Rear axle

    std::vector<Database_parameter> get_parameters() { return 
    { 
        { "mass", _m },
        { "inertia", _I },
        { "aerodynamics/rho", _rho },
        { "aerodynamics/cd", _cd },
        { "aerodynamics/cl", _cl },
        { "aerodynamics/area", _A }
    };}

 protected:

    // Longitudinal dynamics
    Timeseries_t _du;    //! [out] Longitudinal acceleration [m/s2]
    
    // Lateral dynamics
    Timeseries_t _dv;    //! [out] Lateral acceleration [m/s2]

    // Attitude: yaw
    Timeseries_t _dOmega; //! [out] Yaw acceleration [rad/s2]

    Vector3d<Timeseries_t> _F; //! [out] Total forces at road frame
    Vector3d<Timeseries_t> _T; //! [out] Total torque at road frame

    Vector3d<Timeseries_t> _Fext = Vector3d<Timeseries_t>(0.0); //! [in] An external force [N]
    Vector3d<Timeseries_t> _Text = Vector3d<Timeseries_t>(0.0); //! [in] An external torque [Nm]
};

#include "chassis.hpp"

#endif
