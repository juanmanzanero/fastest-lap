#ifndef __TRACK_BY_POLYNOMIAL_H__
#define __TRACK_BY_POLYNOMIAL_H__

#include "lion/io/Xml_document.h"
#include "src/core/applications/circuit_preprocessor.h"

class Track_by_polynomial
{
 public:

    Track_by_polynomial() = default;

    Track_by_polynomial(Xml_document& doc);

    Track_by_polynomial(const vPolynomial& r, const sPolynomial& wl, const sPolynomial& wr) 
        : _r(r), _dr(_r.derivative()), _d2r(_dr.derivative()), _wl(wl), _wr(wr), _wl_correction(), _wr_correction() {}

    Track_by_polynomial(const vPolynomial& r, const vPolynomial& dr, const vPolynomial& d2r, const sPolynomial& wl, const sPolynomial& wr)
        : _r(r), _dr(dr), _d2r(d2r), _wl(wl), _wr(wr), _wl_correction(), _wr_correction() {}

    Track_by_polynomial(const Circuit_preprocessor& circuit_preprocessor);

    Track_by_polynomial(std::tuple<vPolynomial,sPolynomial,sPolynomial> p) : Track_by_polynomial(std::get<0>(p), std::get<1>(p), std::get<2>(p)) {}

    void set_left_track_limit_correction(const sPolynomial& wl_correction) { _wl_correction = wl_correction;  }

    void set_right_track_limit_correction(const sPolynomial& wr_correction) { _wr_correction = wr_correction;  }

    scalar get_left_track_limit(scalar s) const { return _wl(s) + _wl_correction(s); }

    scalar get_right_track_limit(scalar s) const { return _wr(s) + _wr_correction(s); }

    constexpr const scalar& get_total_length() const { return _r.get_right_bound(); } 

    std::tuple<sVector3d,sVector3d,sVector3d> operator()(const scalar& t) { return std::make_tuple(_r(t),_dr(t),_d2r(t)); }

    template<typename Timeseries_t>
    Vector3d<Timeseries_t> position_at(const scalar t, const Timeseries_t& w)
    {
        auto r = _r(t);
        auto dr = _dr(t);
                
        return 
        {  
            r + w*Vector3d<Timeseries_t>(-dr[1],dr[0],0.0)/norm(dr),
        };
    }

    const Circuit_preprocessor& get_preprocessor() const { return _preprocessor; }

 private:
    vPolynomial _r;     //! Position vector polynomial
    vPolynomial _dr;    //! Position vector derivative polynomial
    vPolynomial _d2r;   //! Position vector second derivative polynomial

    sPolynomial _wl;    //! Distance to the left track limit
    sPolynomial _wr;    //! Distance to the right track limit

    sPolynomial _wl_correction; //! Apply a correction to the left track limit
    sPolynomial _wr_correction; //! Apply a correction to the right track limit

    Circuit_preprocessor _preprocessor;     //! Save the preprocessor used to compute this track

    static std::tuple<vPolynomial,sPolynomial,sPolynomial> compute_track_polynomial(Xml_document& doc);
};

#include "track_by_polynomial.hpp"

#endif
