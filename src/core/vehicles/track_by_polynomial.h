#ifndef __TRACK_BY_POLYNOMIAL_H__
#define __TRACK_BY_POLYNOMIAL_H__

#include "lion/io/Xml_document.h"

class Track_by_polynomial
{
 public:

    Track_by_polynomial() = default;

    Track_by_polynomial(Xml_document& doc) : Track_by_polynomial(compute_track_polynomial(doc)) {}

    Track_by_polynomial(const vPolynomial& r) : _r(r), _dr(_r.derivative()), _d2r(_dr.derivative()) {}

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

 private:
    vPolynomial _r;     //! Position vector polynomial
    vPolynomial _dr;    //! Position vector derivative polynomial
    vPolynomial _d2r;   //! Position vector second derivative polynomial

    static vPolynomial compute_track_polynomial(Xml_document& doc);
};

#include "track_by_polynomial.hpp"

#endif
