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

    constexpr const static int CLOCKWISE = -1;
    constexpr const static int COUNTERCLOCKWISE = 1;

    struct Coordinates
    {
        scalar longitude;
        scalar latitude;
    };

    struct Options
    {
        // Fitness function cost parameters
        scalar eps_d = 1.0e-1;
        scalar eps_k = 5.0e4;
        scalar eps_n = 1.0e-1;
        scalar eps_c = 1.0e-1;

        scalar maximum_kappa = 0.1;
        scalar maximum_dkappa = 2.0e-2;
    };

    //! Constructor for closed circuits, from KML
    Circuit_preprocessor(Xml_document& coord_left_kml, Xml_document& coord_right_kml, const size_t n_el, const Options opts);

    //! Constructor for open circuits, from KML
    Circuit_preprocessor(Xml_document& coord_left_kml, Xml_document& coord_right_kml, Coordinates start, Coordinates finish, const size_t n_el, const Options opts);

    //! Constructor for closed circuits
    Circuit_preprocessor(const std::vector<Coordinates>& coord_left, 
                         const std::vector<Coordinates>& coord_right, 
                         const size_t n_el, 
                         const Options opts) 
    : options(opts), n_elements(n_el), n_points(n_el), is_closed(true), direction(0)
    {
        compute<true>(coord_left, coord_right);
    }

    //! Constructor for open circuits
    Circuit_preprocessor(const std::vector<Coordinates>& coord_left, 
                         const std::vector<Coordinates>& coord_right, 
                         Coordinates start, 
                         Coordinates finish, 
                         const size_t n_el, 
                         const Options opts) 
    : options(opts), n_elements(n_el), n_points(n_el+1), is_closed(false), direction(0)
    {
        // Trim the coordinates to the provided start/finish points
        auto [coord_left_trim, coord_right_trim] = trim_coordinates(coord_left, coord_right, start, finish);

        // Perform the computation
        compute<false>(coord_left_trim,coord_right_trim);
    }

    // Inputs ------------------------------------:
    Options     options;
    size_t      n_elements;
    size_t      n_points;
    bool        is_closed;
    int         direction;      

    // Outputs -----------------------------------:
    scalar x0;      
    scalar y0;      
    scalar phi0;    
    scalar theta0;  
    scalar phi_ref; 
    constexpr static const scalar R_earth = 6378388.0;

    std::vector<sVector3d> r_left;
    std::vector<sVector3d> r_left_measured;     
    std::vector<sVector3d> r_right;
    std::vector<sVector3d> r_right_measured;    
    std::vector<sVector3d> r_centerline;

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
    void compute(const std::vector<Coordinates>& coord_left, const std::vector<Coordinates>& coord_right);

    template<bool closed>
    class FG
    {
     public:
        using ADvector = std::vector<CppAD::AD<scalar>>;

        enum State { IX, IY, ITHETA, IKAPPA, INL, INR, NSTATE };
        enum Controls { IDKAPPA, IDNL, IDNR, NCONTROLS };

        FG(const size_t n_elements, 
           const size_t n_points,
           const scalar track_length, 
           const std::vector<sVector3d>& r_left, 
           const std::vector<sVector3d>& r_right, 
           const std::vector<sVector3d>& r_center, 
           int direction, 
           const Options opts) 
            : _n_elements(n_elements), _n_points(n_points), _n_variables(1+(NSTATE+NCONTROLS)*_n_points), 
              _n_constraints(1+NSTATE*n_elements + (closed ? 0 : 1)), _direction(direction), options(opts), _track_length(track_length), _ds(_track_length/n_elements), _r_left(r_left), _r_right(r_right), _r_center(r_center), _q(_n_points), _u(_n_points), _dqds(_n_points),
              _dist2_left(_n_points), _dist2_right(_n_points), _dist2_center(_n_points) {(void)_ds;}

        void operator()(ADvector& fg, const ADvector& x);

        std::array<CppAD::AD<scalar>,NSTATE> equations(const std::array<CppAD::AD<scalar>,NSTATE>& q, const std::array<CppAD::AD<scalar>,NCONTROLS>& u) const
        {
            return { cos(q[ITHETA]), sin(q[ITHETA]), q[IKAPPA], u[IDKAPPA], u[IDNL], u[IDNR] };
        }

        constexpr const size_t& get_n_points() const { return _n_points; }
        constexpr const size_t& get_n_variables() const { return _n_variables; }
        constexpr const size_t& get_n_constraints() const { return _n_constraints; }

     private:
        size_t _n_elements;
        size_t _n_points;
        size_t _n_variables;
        size_t _n_constraints;

        int _direction;

        Options options;
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
                                                                                              const size_t n_elements,
                                                                                              const size_t n_points);

    static std::pair<std::vector<Coordinates>, std::vector<Coordinates>> trim_coordinates(const std::vector<Coordinates>& coord_left, 
                                                                                          const std::vector<Coordinates>& coord_right,
                                                                                          Coordinates start, Coordinates finish);

};

#include "circuit_preprocessor.hpp"

#endif
