#ifndef __TIRE_PACEJKA_H__
#define __TIRE_PACEJKA_H__

#include "tire.h"
#include "lion/math/matrix_extensions.h"

//! Implementation of the complete Pacejka tire model
struct Pacejka_standard_model
{
    //! Initialise the model (i.e. compute extra parameters)
    void initialise();

    //! Applies the magic formula for the case of pure longitudinal slip (lambda=0)
    //! Fx0 = Dx sin(Cx atan(Bx kappa - Ex(Bx kappa - atan(Bx kappa))))
    //! with:
    //!     Dx = pDx1 Fz > 0
    //!     Cx = pCx1    > 0
    //!     Ex = pEx1    <= 1
    //!           (pKx1+pKx2 dfz)exp(pKx3 dfz)
    //!     Bx = ------------------------------
    //!                 pCx1 pDx1
    //!     dfz0 = (Fz - lambdaFz0 Fz0)/(lambda Fz0 Fz0)
    //! @param[in] kappa: instantaneous longitudinal slip
    //! @param[in] Fz: Vertical load [N]
    template<typename Timeseries_t>
    Timeseries_t force_pure_longitudinal_magic(Timeseries_t kappa, Timeseries_t Fz) const;

    //! Applies the magic formula for the case of pure lateral slip (kappa=0)
    //! Fy0 = Dy sin(Cy atan(By lambda - Ey(By lambda - atan(By lambda))))
    //! with:
    //!     Dy = pDy1 Fz  > 0
    //!     Cy = pCy1 
    //!     Ey = pEy1     <= 1
    //!           pKy1 lambdaFz0 Fz0 sin(pKy4 atan(Fz / (lambdaFz0 Fz0 pKy2)))
    //!     By = --------------------------------------------------------------
    //!                                pCy1 pDy1
    //!     (if Fz=0, By = pKy1 pKy4 / (pKy2 pCy1 pDy1)
    //! @param[in] lambda: instantaneous lateral slip
    //! @param[in] Fz: Vertical load [N]
    template<typename Timeseries_t>
    Timeseries_t force_pure_lateral_magic(Timeseries_t lambda, Timeseries_t Fz) const;

    //! Applies the magic formula for the case of longitudinal combined slip
    //! Fx = Gx Fx0
    //! Gx = cos(rCx1 atan(rBx1 lambda))
    //! @param[in] kappa: instantaneous longitudinal slip
    //! @param[in] kappa: instantaneous lateral slip     
    //! @param[in] Fz: Vertical load [N]
    template<typename Timeseries_t>
    Timeseries_t force_combined_longitudinal_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const;

    //! Applies the magic formula for the case of lateral combined slip
    //!
    //! Fy = Gy Fy0
    //! Gy = cos(rCy1 atan(rBy1 lambda))
    //! 
    //! @param[in] kappa: instantaneous longitudinal slip
    //! @param[in] kappa: instantaneous lateral slip     
    //! @param[in] Fz: Vertical load [N]
    template<typename Timeseries_t>
    Timeseries_t force_combined_lateral_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const;

    DECLARE_PARAMS(
        { "nominal-vertical-load", _Fz0 },
        { "lambdaFz0", _lambdaFz0 },
        { "longitudinal/pure/pCx1", _pCx1 },
        { "longitudinal/pure/pDx1", _pDx1 },
        { "longitudinal/pure/pEx1", _pEx1 },
        { "longitudinal/pure/pKx1", _pKx1 },
        { "longitudinal/pure/pKx2", _pKx2 },
        { "longitudinal/pure/pKx3", _pKx3 },
        { "lateral/pure/pCy1", _pCy1 },
        { "lateral/pure/pDy1", _pDy1 },
        { "lateral/pure/pEy1", _pEy1 },
        { "lateral/pure/pKy1", _pKy1 },
        { "lateral/pure/pKy2", _pKy2 },
        { "lateral/pure/pKy4", _pKy4 },
        { "longitudinal/combined/rBx1", _rBx1 },
        { "longitudinal/combined/rCx1", _rCx1 },
        { "lateral/combined/rBy1", _rBy1 },
        { "lateral/combined/rCy1", _rCy1 } 
    ); 

    std::ostream& print(std::ostream& os) const;

    // Parameters of the magic formula
    // ===============================
    // At the moment I will only include the parameters from Lot, Roberto, and Nicola Dal Bianco. 
    // "Lap time optimisation of a racing go-kart."
    scalar _Fz0 = 0.0;        //! [c] Nominal load [N]
    scalar _lambdaFz0 = 0.0;  //! [c] Nominal load scaling
    scalar _Fz0prime = 0.0;   //! [c] Fz0.lambdaFz0

    // Pure slip longitudinal parameters
    scalar _pCx1 = 0.0;    //! [c] Cx
    scalar _pDx1 = 0.0;    //! [c] mu_x
    scalar _pEx1 = 0.0;    //! [c] Ex
    scalar _pKx1 = 0.0;    //! [c] Bx base parameter
    scalar _pKx2 = 0.0;    //! [c] Linear effect of delta(Fz) in Bx
    scalar _pKx3 = 0.0;    //! [c] Exponential effect of delta(Fz) in Bx
    
    // Pure slip lateral parameters
    scalar _pCy1 = 0.0;    //! [c] Cy
    scalar _pDy1 = 0.0;    //! [c] mu_y
    scalar _pEy1 = 0.0;    //! [c] Ey
    scalar _pKy1 = 0.0;    //! [c] By base parameter
    scalar _pKy2 = 0.0;    //! [c] Effect of Fz/Fz0
    scalar _pKy4 = 0.0;    //! [c] Always 2.0
     
    // Combined slip longitudinal parameters
    scalar _rBx1 = 0.0;   //! [c] Coefficient inside the atan 
    scalar _rCx1 = 0.0;   //! [c] Coefficient outside the atan

    // Combined slip lateral parameters
    scalar _rBy1 = 0.0;   //! [c] Coefficient inside the atan
    scalar _rCy1 = 0.0;   //! [c] Coefficient outside the atan

};


//! Implementation of a simpler pacejka model with fewer parameters
template<typename Timeseries_t>
struct Pacejka_simple_model
{
    //! Compute extra constants from the inputs
    void initialise();

    //! Compute the combined longitudinal force
    Timeseries_t force_combined_longitudinal_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const;

    //! Compute the combined lateral force
    Timeseries_t force_combined_lateral_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const;

    //! Compute the maximum kappa
    Timeseries_t maximum_kappa(Timeseries_t Fz) const { return (Fz - _Fz1)*(_kappa_max2 - _kappa_max1)/(_Fz2 - _Fz1) + _kappa_max1; }

    //! Compute the maximum lambda
    Timeseries_t maximum_lambda(Timeseries_t Fz) const { return (Fz - _Fz1)*(_lambda_max2 - _lambda_max1)/(_Fz2 - _Fz1) + _lambda_max1; }

    // Constants
    scalar _Fz1 = 0.0;            //! [c] Reference load 1
    scalar _Fz2 = 0.0;            //! [c] Reference load 2
    
    Timeseries_t _mu_x_max1 = 0.0;      //! [c] peak longitudinal friction coefficient at load 1
    Timeseries_t _mu_x_max2 = 0.0;      //! [c] peak longitudinal friction coefficient at load 2

    Timeseries_t _mu_y_max1 = 0.0;      //! [c] peak lateral friction coefficient at load 1
    Timeseries_t _mu_y_max2 = 0.0;      //! [c] peak lateral friction coefficient at load 2

    scalar _kappa_max1 = 0.0;     //! [c] slip coefficient for the friction peak at load 1
    scalar _kappa_max2 = 0.0;     //! [c] slip coefficient for the friction peak at load 2

    scalar _lambda_max1_deg = 0.0;     //! [c] slip angle for the friction peak at load 1 [deg]
    scalar _lambda_max2_deg = 0.0;     //! [c] slip angle for the friction peak at load 2 [deg]

    scalar _lambda_max1 = 0.0;     //! [c] slip angle for the friction peak at load 1
    scalar _lambda_max2 = 0.0;     //! [c] slip angle for the friction peak at load 2

    scalar _Qx = 0.0;             //! [c] longitudinal shape factor
    scalar _Qy = 0.0;             //! [c] lateral shape factor

    scalar _Sx = 0.0;             //! [c] pi/(2.atan(Qx))
    scalar _Sy = 0.0;             //! [c] pi/(2.atan(Qy))

    scalar _mu_min = 1.0;         //! [c] Minimum value for any friction coefficient

    //! Database parameters to be read from an XML element
    DECLARE_PARAMS(
        { "reference-load-1", _Fz1 },
        { "reference-load-2", _Fz2 },
        { "mu-x-max-1", _mu_x_max1 },
        { "mu-x-max-2", _mu_x_max2 },
        { "mu-y-max-1", _mu_y_max1 },
        { "mu-y-max-2", _mu_y_max2 },
        { "kappa-max-1", _kappa_max1 },
        { "kappa-max-2", _kappa_max2 },
        { "lambda-max-1", _lambda_max1_deg },
        { "lambda-max-2", _lambda_max2_deg },
        { "Qx", _Qx },
        { "Qy", _Qy } 
    ); 


};

template<typename Timeseries_t, typename Pacejka_model, size_t STATE0, size_t CONTROL0>
class Tire_pacejka : public Tire<Timeseries_t, STATE0,CONTROL0>
{
 public:
    //! The parent class type
    using base_type = Tire<Timeseries_t,STATE0,CONTROL0>;

    //! Indices of the state variables of this class: none
    enum State     { STATE_END    = base_type::STATE_END } ;

    //! Indices of the control variables of this class: none
    enum Controls  { CONTROL_END  = base_type::CONTROL_END  } ;

    //! Default constructor
    Tire_pacejka() = default;

    //! Empty constructor
    Tire_pacejka(const std::string& name, const std::string& path) : base_type(name,path), _model(), _kt(0.0),
        _ct(0.0), _Fz_max_ref2(1.0), _Smagic(0.0), _Fmagic(0.0) {}

    //! Constructor
    //! @param[in] name: name of the tire
    //! @param[in] parameters: map(string,scalar) with the tire parameters
    //! @param[in] path: path of this tire on the parameters map
    //! @param[in] type: type of the tire: NORMAL or ONLY_LATERAL
    Tire_pacejka(const std::string& name, 
                 Xml_document& database,
                 const std::string& path="" 
                );

    template<typename T>
    void set_parameter(const std::string& parameter, const T value);

    void fill_xml(Xml_document& doc) const;

    //! Calls Tire::update(x0,v0,omega) of the base class, and calls update_self()
    //! This updates the tire dynamics and tire forces
    //! @param[in] x0: new frame origin position [m]
    //! @param[in] v0: new frame origin velocity [m/s]
    //! @param[in] omega: new value for tire angular speed [rad/s]
    void update(const Vector3d<Timeseries_t>& x0, const Vector3d<Timeseries_t>& v0, Timeseries_t omega);

    //! Calls Tire::update(omega) of the base class, and calls update_self(Fz)
    //! This updates the tire dynamics and tire forces
    //! In this function, the normal load is provided externally
    //! @param[in] Fz: the normal load
    //! @param[in] kappa: new value for tire longitudinal slip [-]
    void update(Timeseries_t Fz, Timeseries_t kappa);

    //! Calls update(omega) of the base class, and calls update_self()
    //! @param[in] omega: new value for tire angular speed [rad/s]
    void update(Timeseries_t omega);

    //! Returns the tire carcass radial stiffness [N/m]
    constexpr const scalar& get_radial_stiffness() const { return _kt; }

    //! Print the tire parameters
    //! @param[in] os: output stream
    std::ostream& print(std::ostream& os) const;
 
    //! Load the time derivative of the state variables computed herein to the dqdt
    //! @param[out] dqdt: the vehicle state vector time derivative
    template<size_t N>
    void get_state_derivative(std::array<Timeseries_t,N>& dqdt) const {};

    //! Set the state variables of this class
    //! @param[in] q: the vehicle state vector 
    //! @param[in] u: the vehicle control vector
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_controls(const std::array<Timeseries_t,NSTATE>& q, 
                                const std::array<Timeseries_t,NCONTROL>& u) {};


    //! Set the state and controls upper, lower, and default values
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_control_upper_lower_and_default_values(const std::array<scalar,NSTATE>& q_def,
                                                               const std::array<scalar,NSTATE>& q_lb,
                                                               const std::array<scalar,NSTATE>& q_ub,
                                                               const std::array<scalar,NCONTROL>& u_def,
                                                               const std::array<scalar,NCONTROL>& u_lb,
                                                               const std::array<scalar,NCONTROL>& u_ub 
                                                              ) const {};

    //! Get the names of the state and control varaibles of this class
    //! @param[out] q: the vehicle state names
    //! @param[out] u: the vehicle control names
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_control_names(std::array<std::string,NSTATE>& q, 
                                     std::array<std::string,NCONTROL>& u) const {};

    static std::string type() { return "tire_pacejka"; }

    const Pacejka_model& get_model() const { return _model; }

    bool is_ready() const { return base_type::is_ready() && 
            std::all_of(__used_parameters.begin(), __used_parameters.end(), [](const auto& v) -> auto { return v; }) &&
            std::all_of(get_model().__used_parameters.begin(), get_model().__used_parameters.end(), [](const auto& v) -> auto { return v; }); }

    std::unordered_map<std::string,Timeseries_t> get_outputs_map() const
    {
        auto map = get_outputs_map_self();
        const auto base_type_map = base_type::get_outputs_map();

        map.insert(base_type_map.cbegin(), base_type_map.cend());

        return map;
    }

 private:
    //! Performs an update of the tire: computes tire forces from kappa, lambda, and tire 
    //! deformations
    void update_self();

    //! Performs an update with given normal load
    void update_self(const Timeseries_t Fz);

    Pacejka_model _model;

    scalar _kt = 0.0; //! [c] Radial stiffness of the carcass
    scalar _ct = 0.0; //! [c] Radial damping of the carcass
    scalar _Fz_max_ref2 = 0.0;    //! [c] Reference normal force for the smooth max function

    // Forces as given by the magic formula
    Timeseries_t _Smagic = 0.0;     //! [out] Longitudinal force given by the magic formula
    Timeseries_t _Fmagic = 0.0;     //! [out] Lateral force given by the magic formula

    DECLARE_PARAMS(
        { "radial-stiffness", _kt },
        { "radial-damping", _ct },
        { "Fz-max-ref2", _Fz_max_ref2 }
    ); 

    std::unordered_map<std::string,Timeseries_t> get_outputs_map_self() const
    {
        return
        {
        };
    }

};


//! Display the properties of a tire
//! @param[in] os: out stream
//! @param[in] tire: the tire
template<typename Timeseries_t,typename Pacejka_model,size_t STATE0, size_t CONTROL0>
inline std::ostream& operator<<(std::ostream& os, const Tire_pacejka<Timeseries_t,Pacejka_model,STATE0,CONTROL0>& tire){return tire.print(os);}

template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
using Tire_pacejka_std = Tire_pacejka<Timeseries_t,Pacejka_standard_model,STATE0,CONTROL0>;

template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
using Tire_pacejka_simple = Tire_pacejka<Timeseries_t,Pacejka_simple_model<Timeseries_t>,STATE0,CONTROL0>;

#include "tire_pacejka.hpp"

#endif
