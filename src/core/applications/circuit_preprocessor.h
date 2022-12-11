#ifndef CIRCUIT_PREPROCESSOR_H
#define CIRCUIT_PREPROCESSOR_H

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
        scalar altitude;
    };

    struct Options
    {
        bool with_elevation = true;

        // Fitness function cost parameters
        scalar eps_d = 1.0e-1;
        scalar eps_k = 5.0e4;
        scalar eps_n = 1.0e-1;
        scalar eps_c = 1.0e-1;
        scalar eps_mu = 1.0e6;
        scalar eps_phi = 1.0e6;

        scalar maximum_kappa = 0.1;
        scalar maximum_dkappa = 2.0e-2;
        scalar maximum_dn     = 1.0;
        scalar maximum_distance_find = 50.0;

        scalar adaption_aspect_ratio_max = 1.2;

        int print_level = 0;
    };

    //! Default constructor
    Circuit_preprocessor() = default;

    //! Constructor from Xml file
    Circuit_preprocessor(Xml_document& doc);

    //! Any constructor from KML
    template<typename ... Args>
    Circuit_preprocessor(Xml_document& coord_left_kml, 
                         Xml_document& coord_right_kml,
                         Options opts,
                         Args&&... args)
    : options(opts)
    {
        // (1) Read the KML files and transform to vector of coordinates
        auto [coord_left, coord_right] = read_kml(coord_left_kml,coord_right_kml);

        // (2) Call the proper implementation from vector of coordinates 
        *this = Circuit_preprocessor(coord_left, coord_right, opts, std::forward<Args>(args)...);
    }

    //! Constructor for closed circuits, from number of elements
    Circuit_preprocessor(const std::vector<Coordinates>& coord_left, 
                         const std::vector<Coordinates>& coord_right, 
                         const Options opts, 
                         const size_t n_el) 
    : options(opts), n_elements(n_el), n_points(n_el), is_closed(true), direction(0)
    {
        // (1) Compute the centerline and preprocess inputs
        transform_coordinates<true>(coord_left, coord_right);

        // (3) Compute the centerline estimate
        const auto [s_center,r_center,r_center_to_right,track_length_estimate] = compute_averaged_centerline<true>(r_left_measured,r_right_measured,n_elements,n_points,options);

        // (3) Perform the optimization
        if (opts.with_elevation)
        {
            compute<true, elevation_computation_names>(s_center, r_center, r_center_to_right, track_length_estimate);
        }
        else
        {
            compute<true, flat_computation_names>(s_center, r_center, r_center_to_right, track_length_estimate);
        }
    }

    //! Constructor for closed circuits, from mesh size given as breakpoints along the circuit
    Circuit_preprocessor(const std::vector<Coordinates>& coord_left, 
                         const std::vector<Coordinates>& coord_right, 
                         const Options opts, 
                         const std::vector<std::pair<Coordinates,scalar>>& ds_breakpoints) 
    : options(opts), n_elements(0), n_points(0), is_closed(true), direction(0)
    {
        // (1) Compute the centerline and preprocess inputs
        transform_coordinates<true>(coord_left, coord_right);

        // transform the ds_breakpoints to sVector
        std::vector<std::pair<sVector3d,scalar>> ds_breakpoints_v3d(ds_breakpoints.size());

        for (size_t i = 0; i < ds_breakpoints.size(); ++i)
        {
            ds_breakpoints_v3d[i].first = sVector3d((ds_breakpoints[i].first.longitude*DEG-theta0)*R_earth*cos(phi_ref), 
                                                    -(ds_breakpoints[i].first.latitude*DEG-phi0)*R_earth, 
                                                    0.0);
            ds_breakpoints_v3d[i].second = ds_breakpoints[i].second;
        }

        // (3) Compute the centerline estimate
        const auto [s_center,r_center,r_center_to_right,track_length_estimate] = compute_averaged_centerline<true>(r_left_measured,r_right_measured,ds_breakpoints_v3d,options);

        n_points   = s_center.size();
        n_elements = n_points;

        // (3) Perform the optimization
        if (opts.with_elevation)
        {
            compute<true,elevation_computation_names>(s_center, r_center, r_center_to_right, track_length_estimate);
        }
        else
        {
            compute<true,flat_computation_names>(s_center, r_center, r_center_to_right, track_length_estimate);
        }

    }

    //! Constructor for closed circuits, from mesh size given as breakpoints along the circuit
    Circuit_preprocessor(const std::vector<Coordinates>& coord_left, 
                         const std::vector<Coordinates>& coord_right, 
                         const Options opts, 
                         const std::vector<scalar>& s_distribution,
                         const std::vector<scalar>& ds_distribution)
    : options(opts), n_elements(0), n_points(0), is_closed(true), direction(0)
    {
        // (1) Compute the centerline and preprocess inputs
        transform_coordinates<true>(coord_left, coord_right);

        // (2) Compute the centerline estimate
        const auto [s_center,r_center,r_center_to_right,track_length_estimate] = compute_averaged_centerline<true>(r_left_measured,r_right_measured,s_distribution,ds_distribution,options);

        n_points   = s_center.size();
        n_elements = n_points;

        // (3) Perform the optimization
        if (opts.with_elevation)
        {
            compute<true,elevation_computation_names>(s_center, r_center, r_center_to_right, track_length_estimate);
        }
        else
        {
            compute<true,flat_computation_names>(s_center, r_center, r_center_to_right, track_length_estimate);
        }
    }

    //! Constructor for open circuits
    Circuit_preprocessor(const std::vector<Coordinates>& coord_left, 
                         const std::vector<Coordinates>& coord_right, 
                         const Options opts, 
                         Coordinates start, 
                         Coordinates finish, 
                         const size_t n_el) 
    : options(opts), n_elements(n_el), n_points(n_el+1), is_closed(false), direction(0)
    {
        // (1) Trim the coordinates to the provided start/finish points
        auto [coord_left_trim, coord_right_trim] = trim_coordinates(coord_left, coord_right, start, finish);

        // (2) Compute the centerline and preprocess inputs
        transform_coordinates<false>(coord_left_trim, coord_right_trim);

        // (3) Compute the centerline estimate
        const auto [s_center,r_center,r_center_to_right,track_length_estimate] = compute_averaged_centerline<false>(r_left_measured,r_right_measured,n_elements,n_points,options);

        // (4) Perform the optimization
        if (opts.with_elevation)
        {
            compute<false, elevation_computation_names>(s_center, r_center, r_center_to_right, track_length_estimate);
        }
        else
        {
            compute<false, flat_computation_names>(s_center, r_center, r_center_to_right, track_length_estimate);
        }
    }

    // Inputs ------------------------------------:-
    Options     options;
    size_t      n_elements;
    size_t      n_points;
    bool        is_closed;
    int         direction;      

    // Outputs -----------------------------------:-
    scalar x0;      
    scalar y0;      
    scalar phi0;    
    scalar theta0;  
    scalar phi_ref; 
    scalar R_earth = 6378388.0;

    std::vector<sVector3d> r_left;
    std::vector<sVector3d> r_left_measured;     
    std::vector<sVector3d> r_right;
    std::vector<sVector3d> r_right_measured;    
    std::vector<sVector3d> r_centerline;

    std::vector<scalar> s;
    std::vector<scalar> theta;
    std::vector<scalar> mu;
    std::vector<scalar> phi;
    std::vector<scalar> kappa;
    std::vector<scalar> mu_dot;
    std::vector<scalar> phi_dot;
    std::vector<scalar> nl;
    std::vector<scalar> nr;
    std::vector<scalar> dkappa;
    std::vector<scalar> dmu_dot;
    std::vector<scalar> dphi_dot;
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
    void transform_coordinates(const std::vector<Coordinates>& coord_left, const std::vector<Coordinates>& coord_right);

    template<bool closed, typename computation_type>
    void compute(const std::vector<scalar>& s_center, const std::vector<sVector3d>& r_center, const std::vector<sVector3d>& r_center_to_right, const scalar track_length_estimate);

    struct flat_computation_names
    {
        struct state_names
        {
            enum { x, y, theta, kappa, nl, nr, end };
        };

        struct control_names
        {
            enum { dkappa, dnl, dnr, end };
        };
    };

    struct elevation_computation_names
    {
        struct state_names
        {
            enum { x, y, z, theta, mu, phi, kappa, mu_dot, phi_dot, nl, nr, end };
        };

        struct control_names
        {
            enum { dkappa, dmu_dot, dphi_dot, dnl, dnr, end };
        };
    };

    template<bool closed, typename computation_type>
    class FG : public computation_type
    {
     public:
         using ADvector      = std::vector<CppAD::AD<scalar>>;
         using state_names   = typename computation_type::state_names;
         using control_names = typename computation_type::control_names;

        FG(const size_t n_elements, 
           const size_t n_points,
           const std::vector<scalar>& element_ds,
           const std::vector<sVector3d>& r_left, 
           const std::vector<sVector3d>& r_right, 
           const std::vector<sVector3d>& r_center, 
           int direction, 
           const Options opts) 
            : _n_elements(n_elements), _n_points(n_points), 
              _n_variables(1+(state_names::end+control_names::end)*_n_points), 
              _n_constraints(1+state_names::end*n_elements + (closed ? 0 : 1)), 
              _direction(direction), options(opts), _ds(element_ds), 
              _r_left(r_left), _r_right(r_right), _r_center(r_center), 
              _q(_n_points), _u(_n_points), _dqds(_n_points),
              _dist2_left(_n_points), _dist2_right(_n_points), _dist2_center(_n_points) {}

        void operator()(ADvector& fg, const ADvector& x);

        std::array<CppAD::AD<scalar>,state_names::end> equations(const std::array<CppAD::AD<scalar>,state_names::end>& q, const std::array<CppAD::AD<scalar>,control_names::end>& u) const
        {
            if constexpr (std::is_same_v<computation_type, flat_computation_names>)
                return { cos(q[state_names::theta]), sin(q[state_names::theta]), q[state_names::kappa], u[control_names::dkappa], u[control_names::dnl], u[control_names::dnr] };

            else
            {
                static_assert(std::is_same_v<computation_type, elevation_computation_names>);

                return { cos(q[state_names::theta]) * cos(q[state_names::mu]),
                         sin(q[state_names::theta]) * cos(q[state_names::mu]),
                         -sin(q[state_names::mu]),
                         q[state_names::kappa],
                         q[state_names::mu_dot],
                         q[state_names::phi_dot],
                         u[control_names::dkappa],
                         u[control_names::dmu_dot],
                         u[control_names::dphi_dot],
                         u[control_names::dnl],
                         u[control_names::dnr]
                };
            }
        }

        template<typename U, typename V>
        static Vector3d<U> get_coordinates(const std::array<U, state_names::end>& q, const V& n)
        {
            if constexpr (std::is_same_v<computation_type, flat_computation_names>)
            {
                return { q[state_names::x] - sin(q[state_names::theta]) * n, q[state_names::y] + cos(q[state_names::theta]) * n, 0.0 };
            }
            else
            {
                static_assert(std::is_same_v<computation_type, elevation_computation_names>);

                const auto& theta = q[state_names::theta];
                const auto& mu = q[state_names::mu];
                const auto& phi = q[state_names::phi];
                Vector3d<U> normal_vector = { cos(theta) * sin(mu) * sin(phi) - sin(theta) * cos(phi),
                                                             sin(theta) * sin(mu) * sin(phi) + cos(theta) * cos(phi),
                                                             cos(mu) * sin(phi) };

                return { q[state_names::x] + normal_vector.x() * n, 
                         q[state_names::y] + normal_vector.y() * n, 
                         q[state_names::z] + normal_vector.z() * n };
            }
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
        std::vector<scalar> _ds;

        std::vector<sVector3d> _r_left;
        std::vector<sVector3d> _r_right;
        std::vector<sVector3d> _r_center;

        std::vector<std::array<CppAD::AD<scalar>,state_names::end>> _q;
        std::vector<std::array<CppAD::AD<scalar>,control_names::end>> _u;
        std::vector<std::array<CppAD::AD<scalar>,state_names::end>> _dqds;

        std::vector<CppAD::AD<scalar>> _dist2_left;
        std::vector<CppAD::AD<scalar>> _dist2_right;
        std::vector<CppAD::AD<scalar>> _dist2_center;
    };

    std::pair<std::vector<Coordinates>,std::vector<Coordinates>> read_kml(Xml_document& coord_left_kml, Xml_document& coord_right_kml);

    //! Compute the averaged centerline between r_left and r_right with given number of elements
    struct Centerline
    {
        std::vector<scalar> s;
        std::vector<sVector3d> r_center;
        std::vector<sVector3d> r_center_to_right;
        scalar track_length;
    };

    template<bool closed>
    static Centerline compute_averaged_centerline(std::vector<sVector3d> r_left, 
                                                  std::vector<sVector3d> r_right, 
                                                  const size_t n_elements,
                                                  const size_t n_points,
                                                  const Options& options);
    
    //! Compute the averaged centerline between r_left and r_right with given arclength size breakpoints along the circuit
    template<bool closed>
    static Centerline compute_averaged_centerline(std::vector<sVector3d> r_left,
                                                  std::vector<sVector3d> r_right,
                                                  const std::vector<std::pair<sVector3d,scalar>>& ds_breakpoints,
                                                  const Options& options);

    //! Compute the averaged centerline between r_left and r_right with given arclength given as ds = f(s)
    template<bool closed>
    static Centerline compute_averaged_centerline(std::vector<sVector3d> r_left,
                                                  std::vector<sVector3d> r_right,
                                                  const std::vector<scalar>& s_distribution,
                                                  const std::vector<scalar>& ds_distribution,
                                                  const Options& options);



    static std::pair<std::vector<Coordinates>, std::vector<Coordinates>> trim_coordinates(const std::vector<Coordinates>& coord_left, 
                                                                                          const std::vector<Coordinates>& coord_right,
                                                                                          Coordinates start, Coordinates finish);

    template<bool closed>
    static scalar compute_ds_for_coordinates(const sVector3d point, const std::vector<sVector3d>& r_curve, const std::vector<std::pair<sVector3d,scalar>>& ds_breakpoints);

    static size_t who_is_ahead(std::array<size_t,2>& i_p1, std::array<size_t,2>& i_p2, const sVector3d& p1, const sVector3d& p2, const sVector3d& p_ref);
};

#include "circuit_preprocessor.hpp"

#endif
