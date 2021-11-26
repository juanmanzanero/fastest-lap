#ifndef __TIRE_PACEJKA_H__
#define __TIRE_PACEJKA_H__

#include "tire.h"

template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
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

    //! Constructor
    //! @param[in] name: name of the tire
    //! @param[in] parameters: map(string,scalar) with the tire parameters
    //! @param[in] path: path of this tire on the parameters map
    //! @param[in] type: type of the tire: NORMAL or ONLY_LATERAL
    Tire_pacejka(const std::string& name, 
                 Xml_document& database,
                 const std::string& path="" 
                );

    //! Calls update(x0,v0,omega) of the base class, and calls update()
    //! This updates the tire dynamics and tire forces
    //! @param[in] x0: new frame origin position [m]
    //! @param[in] v0: new frame origin velocity [m/s]
    //! @param[in] omega: new value for tire angular speed [rad/s]
    void update(const Vector3d<Timeseries_t>& x0, const Vector3d<Timeseries_t>& v0, Timeseries_t omega);

    //! Calls update(omega) of the base class, and calls update()
    //! @param[in] omega: new value for tire angular speed [rad/s]
    void update(Timeseries_t omega);

    //! Returns the tire carcass radial stiffness [N/m]
    constexpr const scalar& get_radial_stiffness() const { return _kt; }

    //! Print the tire parameters
    //! @param[in] os: output stream
    std::ostream& print(std::ostream& os) const;
 
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
    Timeseries_t force_pure_lateral_magic(Timeseries_t lambda, Timeseries_t Fz) const;

    //! Applies the magic formula for the case of longitudinal combined slip
    //! Fx = Gx Fx0
    //! Gx = cos(rCx1 atan(rBx1 lambda))
    //! @param[in] kappa: instantaneous longitudinal slip
    //! @param[in] kappa: instantaneous lateral slip     
    //! @param[in] Fz: Vertical load [N]
    Timeseries_t force_combined_longitudinal_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const;

    //! Applies the magic formula for the case of lateral combined slip
    //!
    //! Fy = Gy Fy0
    //! Gy = cos(rCy1 atan(rBy1 lambda))
    //! 
    //! @param[in] kappa: instantaneous longitudinal slip
    //! @param[in] kappa: instantaneous lateral slip     
    //! @param[in] Fz: Vertical load [N]
    Timeseries_t force_combined_lateral_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const;

    struct Max_force
    {
        double Fx;
        double Fy;
        double kappa;
        double lambda;
    };

    Max_force compute_maximum_force(double theta, double Fz) const;

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

    //! Get the names of the state and control varaibles of this class
    //! @param[out] q: the vehicle state names
    //! @param[out] u: the vehicle control names
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_control_names(std::array<std::string,NSTATE>& q, 
                                     std::array<std::string,NCONTROL>& u) const {};

    static std::string type() { return "tire_pacejka"; }

 private:
    //! Performs an update of the tire: computes tire forces from kappa, lambda, and tire 
    //! deformations
    void update();

    // Parameters of the magic formula
    // ===============================
    // At the moment I will only include the parameters from Lot, Roberto, and Nicola Dal Bianco. 
    // "Lap time optimisation of a racing go-kart."
    scalar _Fz0;        //! [c] Nominal load [N]
    scalar _lambdaFz0;  //! [c] Nominal load scaling
    scalar _Fz0prime;   //! [c] Fz0.lambdaFz0

    // Pure slip longitudinal parameters
    scalar _pCx1;    //! [c] Cx
    scalar _pDx1;    //! [c] mu_x
    scalar _pEx1;    //! [c] Ex
    scalar _pKx1;    //! [c] Bx base parameter
    scalar _pKx2;    //! [c] Linear effect of delta(Fz) in Bx
    scalar _pKx3;    //! [c] Exponential effect of delta(Fz) in Bx
    
    // Pure slip lateral parameters
    scalar _pCy1;    //! [c] Cy
    scalar _pDy1;    //! [c] mu_y
    scalar _pEy1;    //! [c] Ey
    scalar _pKy1;    //! [c] By base parameter
    scalar _pKy2;    //! [c] Effect of Fz/Fz0
    scalar _pKy4;    //! [c] Always 2.0
     
    // Combined slip longitudinal parameters
    scalar _rBx1;   //! [c] Coefficient inside the atan 
    scalar _rCx1;   //! [c] Coefficient outside the atan

    // Combined slip lateral parameters
    scalar _rBy1;   //! [c] Coefficient inside the atan
    scalar _rCy1;   //! [c] Coefficient outside the atan

    scalar _kt; //! [c] Radial stiffness of the carcass
    scalar _ct; //! [c] Radial damping of the carcass
    scalar _Fz_max_ref2;    //! [c] Reference normal force for the smooth max function

    // Forces as given by the magic formula
    Timeseries_t _Smagic;     //! [out] Longitudinal force given by the magic formula
    Timeseries_t _Fmagic;     //! [out] Lateral force given by the magic formula

    std::vector<Database_parameter> get_parameters() { return 
    { 
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
        { "lateral/combined/rCy1", _rCy1 },
        { "radial-stiffness", _kt },
        { "radial-damping", _ct },
        { "Fz-max-ref2", _Fz_max_ref2 }
    };}

    struct optimise
    {
        struct fitness
        {
            using argument_type = std::array<Timeseries_t,2>;
            fitness(double theta, double Fz, const Tire_pacejka& tire) : _theta(wrap_to_pi(theta)), _Fz(Fz), _tire(&tire) {}
            double operator() (const argument_type& x) const
            { 
                double Fx = _tire->force_combined_longitudinal_magic(x[0], x[1], _Fz);
                double Fy = _tire->force_combined_lateral_magic(x[0], x[1], _Fz);

                return -(Fx*Fx + Fy*Fy);
            }

            double _theta;
            double _Fz;
            const Tire_pacejka* _tire;
        };

        struct constraints
        {
            using argument_type = std::array<Timeseries_t,2>;
            constraints(double theta, double Fz, const Tire_pacejka& tire) : _theta(wrap_to_pi(theta)), _Fz(Fz), _tire(&tire) {}

            std::array<Timeseries_t,1> operator() (const argument_type& x) const
            {
                double Fx = _tire->force_combined_longitudinal_magic(x[0], x[1], _Fz) ;
                double Fy = _tire->force_combined_lateral_magic(x[0], x[1], _Fz) ;

                return {_theta - std::atan2(Fy,Fx)};
            }

            double _theta;
            double _Fz;
            const Tire_pacejka* _tire;
        };
    };

};


//! Display the properties of a tire
//! @param[in] os: out stream
//! @param[in] tire: the tire
template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
inline std::ostream& operator<<(std::ostream& os, const Tire_pacejka<Timeseries_t,STATE0,CONTROL0>& tire){return tire.print(os);}

#include "tire_pacejka.hpp"

#endif
