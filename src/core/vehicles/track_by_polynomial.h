#ifndef __TRACK_BY_POLYNOMIAL_H__
#define __TRACK_BY_POLYNOMIAL_H__

class Track_by_polynomial
{
 public:
    Track_by_polynomial(const vPolynomial& r) : _r(r), _dr(_r.derivative()), _d2r(_dr.derivative()) {}

    constexpr const scalar& get_total_length() const { return _r.get_right_bound(); } 

    std::tuple<sVector3d,sVector3d,sVector3d> operator()(const scalar& t) { return std::make_tuple(_r(t),_dr(t),_d2r(t)); }

 private:
    vPolynomial _r;     //! Position vector polynomial
    vPolynomial _dr;    //! Position vector derivative polynomial
    vPolynomial _d2r;   //! Position vector second derivative polynomial
};

#endif
