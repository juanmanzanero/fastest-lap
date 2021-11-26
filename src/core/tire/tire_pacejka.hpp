#ifndef __TIRE_PACEJKA_HPP__
#define __TIRE_PACEJKA_HPP__

#include "lion/math/optimise.h"
#include "lion/thirdparty/include/logger.hpp"

template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
inline Tire_pacejka<Timeseries_t,STATE0,CONTROL0>::Tire_pacejka(const std::string& name, Xml_document& database, 
                                  const std::string& path)
: Tire<Timeseries_t,STATE0,CONTROL0>(name, database, path),
 _Smagic(0.0),
 _Fmagic(0.0)
{
    read_parameters(database, path, get_parameters());
    _Fz0prime = _lambdaFz0*_Fz0;
}


template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
inline void Tire_pacejka<Timeseries_t,STATE0,CONTROL0>::update(const Vector3d<Timeseries_t>& x0, const Vector3d<Timeseries_t>& v0, Timeseries_t omega)
{
    base_type::update(x0, v0, omega);

    update();
}


template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
inline void Tire_pacejka<Timeseries_t,STATE0,CONTROL0>::update(Timeseries_t omega)
{
    base_type::update(omega);

    update();
}


template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
inline void Tire_pacejka<Timeseries_t,STATE0,CONTROL0>::update()
{
    // Compute the magic formula forces
    // For now, lets use a steady-state version
    const Timeseries_t Fz = smooth_pos<Timeseries_t>(_kt*base_type::_w + _ct*base_type::_dw, _Fz_max_ref2);

    _Smagic =  force_combined_longitudinal_magic(base_type::_kappa,base_type::_lambda,Fz);
    _Fmagic = force_combined_lateral_magic(base_type::_kappa, base_type::_lambda, Fz);

    base_type::_F[X] = _Smagic; 
    base_type::_F[Y] = _Fmagic;
    base_type::_F[Z] = -Fz;

    // Compute the torque at the wheel centre
    base_type::_T = cross(base_type::get_contact_point(), base_type::_F);
}


template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
inline Timeseries_t Tire_pacejka<Timeseries_t,STATE0,CONTROL0>::force_pure_longitudinal_magic(Timeseries_t kappa, Timeseries_t Fz) const
{
    if ( _pDx1 == 0 )
        return 0.0; 

    const Timeseries_t& Cx = _pCx1;
    const Timeseries_t& mu_x = _pDx1;
    const Timeseries_t& Ex = _pEx1;
    const Timeseries_t  dfz = (Fz - _Fz0prime)/_Fz0prime;
    const Timeseries_t  Bx = (_pKx1 + _pKx2*dfz)*exp(_pKx3*dfz)/(_pCx1*_pDx1);

    return mu_x*Fz*sin(Cx*atan(Bx*kappa - Ex*(Bx*kappa - atan(Bx*kappa))));
}   


template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
inline Timeseries_t Tire_pacejka<Timeseries_t,STATE0,CONTROL0>::force_pure_lateral_magic(Timeseries_t lambda, Timeseries_t Fz) const
{
    const Timeseries_t& Cy = _pCy1;
    const Timeseries_t& mu_y = _pDy1;
    const Timeseries_t& Ey = _pEy1;
    Timeseries_t By(0.0);

    if ( abs(Fz) > eps ) 
        By = (_pKy1*_Fz0prime*sin(_pKy4*atan(Fz/(_Fz0prime*_pKy2)))) / (_pCy1*_pDy1*Fz);
    else
        By = _pKy1*_pKy4 / (_pKy2*_pCy1*_pDy1);

    return mu_y*Fz*sin(Cy*atan(By*lambda - Ey*(By*lambda - atan(By*lambda))));
}


template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
inline Timeseries_t Tire_pacejka<Timeseries_t,STATE0,CONTROL0>::force_combined_longitudinal_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const
{
    const Timeseries_t Fx0 = force_pure_longitudinal_magic(kappa, Fz);
    const Timeseries_t Gxlambda = cos(_rCx1*atan(_rBx1*lambda));

    return Gxlambda*Fx0;
}


template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
inline Timeseries_t Tire_pacejka<Timeseries_t,STATE0,CONTROL0>::force_combined_lateral_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const
{
    const Timeseries_t Fy0 = force_pure_lateral_magic(lambda,Fz);
    const Timeseries_t Gykappa = cos(_rCy1*atan(_rBy1*kappa));

    return Gykappa*Fy0;
}


template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
typename Tire_pacejka<Timeseries_t,STATE0,CONTROL0>::Max_force Tire_pacejka<Timeseries_t,STATE0,CONTROL0>::compute_maximum_force(double theta, double Fz) const
{
    // Construct fitness function and constraints
    typename optimise::fitness     f(theta, Fz, *this);
    typename optimise::constraints c(theta, Fz, *this);

    std::vector<Timeseries_t> x0 = {0.05*cos(theta),0.05*sin(theta)};

    std::vector<scalar> x_lb(2,-0.15);
    std::vector<scalar> x_ub(2,0.15);
    std::vector<scalar> c_lb(1,0.0);
    std::vector<scalar> c_ub(1,0.0);

    auto result = Optimise<typename optimise::fitness,typename optimise::constraints>::optimise(2,1,x0,f,c,x_lb,x_ub, c_lb, c_ub);
    
    const double& kappa = result.x[0];
    const double& lambda = result.x[1];
    const double Fx = force_combined_longitudinal_magic(kappa,lambda,Fz);
    const double Fy = force_combined_lateral_magic(kappa,lambda,Fz);
    
    return { Fx,Fy,kappa,lambda};
}


template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
inline std::ostream& Tire_pacejka<Timeseries_t,STATE0,CONTROL0>::print(std::ostream& os) const
{
    base_type::print(os);
    out(2) << std::left << std::setw(16) << "   * kt: "  << std::right << std::setw(5) << _kt << std::endl;
    out(2) << std::left << std::setw(16) << "   * ct: "  << std::right << std::setw(5) << _ct << std::endl;
    out(2) << std::left << std::setw(16) << "   * Fz0: "  << std::right << std::setw(5) << _Fz0 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pCx1: "  << std::right << std::setw(5) << _pCx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pDx1: "  << std::right << std::setw(5) << _pDx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pEx1: "  << std::right << std::setw(5) << _pEx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKx1: "  << std::right << std::setw(5) << _pKx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKx2: "  << std::right << std::setw(5) << _pKx2 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKx3: "  << std::right << std::setw(5) << _pKx3 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pCy1: "  << std::right << std::setw(5) << _pCy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pDy1: "  << std::right << std::setw(5) << _pDy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pEy1: "  << std::right << std::setw(5) << _pEy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKy1: "  << std::right << std::setw(5) << _pKy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKy2: "  << std::right << std::setw(5) << _pKy2 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKy4: "  << std::right << std::setw(5) << _pKy4 << std::endl;
    out(2) << std::left << std::setw(16) << "   * rBx1: "  << std::right << std::setw(5) << _rBx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * rCx1: "  << std::right << std::setw(5) << _rCx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * rBy1: "  << std::right << std::setw(5) << _rBy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * rCy1: "  << std::right << std::setw(5) << _rCy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * lambdaFz0: " <<  std::right << std::setw(5) << _lambdaFz0 << std::endl;

    return os;
}


#endif
