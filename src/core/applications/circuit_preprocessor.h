#ifndef __CIRCUIT_PREPROCESSOR_H__
#define __CIRCUIT_PREPROCESSOR_H__

#include <vector>
#include <array>
#include "lion/math/vector3d.h"
#include "lion/io/Xml_document.h"
#include <memory>


class Circuit_preprocessor
{
 public:
    enum { LON, LAT };

    struct Options
    {
        scalar eps_d = 1.0e-1;
        scalar eps_k = 5.0e4;
        scalar eps_n = 1.0e-1;
        scalar eps_c = 1.0e-1;

        scalar maximum_kappa = 0.1;
        scalar maximum_dkappa = 2.0e-2;
    };

    Circuit_preprocessor(Xml_document& coord_left_kml, Xml_document& coord_right_kml, const size_t n, bool closed, const Options options);

    Circuit_preprocessor(const std::vector<std::array<scalar,2>>& coord_left, const std::vector<std::array<scalar,2>>& coord_right, const size_t n, bool closed, const Options options);

    // Inputs ------------------------------------:
    Options _options;
    bool is_closed;
    int direction;      // -1 clockwise/ +1 counter clockwise

    // Outputs -----------------------------------:
    scalar x0;      // done
    scalar y0;      // done
    scalar phi0;    // done
    scalar theta0;  // done
    scalar phi_ref; // done
    constexpr static const scalar R_earth = 6378388.0;

    std::vector<sVector3d> r_left;
    std::vector<sVector3d> r_left_measured;     
    std::vector<sVector3d> r_right;
    std::vector<sVector3d> r_right_measured;    
    std::vector<sVector3d> r_centerline;

    size_t n_points;
    scalar ds;
    std::vector<scalar> s;
    std::vector<scalar> theta;
    std::vector<scalar> kappa;
    std::vector<scalar> nl;
    std::vector<scalar> nr;
    std::vector<scalar> dkappa;
    std::vector<scalar> dnl;
    std::vector<scalar> dnr;

    scalar track_length;
    scalar left_boundary_max_error;
    scalar right_boundary_max_error;
    scalar left_boundary_L2_error;
    scalar right_boundary_L2_error;

    std::unique_ptr<Xml_document> xml() const;

 private:

    template<bool closed>
    void compute(const std::vector<std::array<scalar,2>>& coord_left, const std::vector<std::array<scalar,2>>& coord_right, const size_t n);

    template<bool closed>
    class FG
    {
     public:
        using ADvector = std::vector<CppAD::AD<scalar>>;

        enum State { IX, IY, ITHETA, IKAPPA, INL, INR, NSTATE };
        enum Controls { IDKAPPA, IDNL, IDNR, NCONTROLS };

        //!@param[in] n: number of elements, not points
        FG(const size_t n, const scalar track_length, const std::vector<sVector3d>& r_left, const std::vector<sVector3d>& r_right, const std::vector<sVector3d>& r_center, int direction, const Options options) 
            : _n(n), _n_points(closed ? n : n+1), _n_variables(1+(NSTATE+NCONTROLS)*_n_points), 
              _n_constraints(1+NSTATE*n + (closed ? 0 : 1)), _direction(direction), _options(options), _track_length(track_length), _ds(_track_length/n), _r_left(r_left), _r_right(r_right), _r_center(r_center), _q(_n_points), _u(_n_points), _dqds(_n_points),
              _dist2_left(n), _dist2_right(n), _dist2_center(n) {(void)_ds;}

        void operator()(ADvector& fg, const ADvector& x);

        std::array<CppAD::AD<scalar>,NSTATE> equations(const std::array<CppAD::AD<scalar>,NSTATE>& q, const std::array<CppAD::AD<scalar>,NCONTROLS>& u) const
        {
            return { cos(q[ITHETA]), sin(q[ITHETA]), q[IKAPPA], u[IDKAPPA], u[IDNL], u[IDNR] };
        }

        constexpr const size_t& get_n_points() const { return _n_points; }
        constexpr const size_t& get_n_variables() const { return _n_variables; }
        constexpr const size_t& get_n_constraints() const { return _n_constraints; }

     private:
        size_t _n;
        size_t _n_points;
        size_t _n_variables;
        size_t _n_constraints;

        int _direction;

        Options _options;
        scalar _track_length;
        scalar _ds;

        std::vector<sVector3d> _r_left;
        std::vector<sVector3d> _r_right;
        std::vector<sVector3d> _r_center;

        std::vector<std::array<CppAD::AD<scalar>,NSTATE>> _q;
        std::vector<std::array<CppAD::AD<scalar>,NCONTROLS>> _u;
        std::vector<std::array<CppAD::AD<scalar>,NSTATE>> _dqds;

        std::vector<CppAD::AD<scalar>> _dist2_left;
        std::vector<CppAD::AD<scalar>> _dist2_right;
        std::vector<CppAD::AD<scalar>> _dist2_center;
    };


    template<bool closed>
    static std::pair<std::vector<scalar>, std::vector<sVector3d>> compute_averaged_centerline(std::vector<sVector3d> r_left, 
                                                                                              std::vector<sVector3d> r_right, 
                                                                                              const size_t n);
};

#include "circuit_preprocessor.hpp"

#endif
