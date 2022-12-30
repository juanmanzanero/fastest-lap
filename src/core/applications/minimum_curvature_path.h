#ifndef MINIMUM_CURVATURE_PATH_H
#define MINIMUM_CURVATURE_PATH_H

#include "src/core/vehicles/track_by_arcs.h"
#include "src/core/vehicles/circuit_geometry.h"

#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "src/core/foundation/fastest_lap_exception.h"

class Minimum_curvature_path
{
 public:
    struct Options
    {
        size_t print_level                = 0;
        size_t maximum_iterations         = 3000;
        bool   throw_if_fail              = true;
        scalar nlp_tolerance              = 1.0e-8;
        scalar constraints_viol_tolerance = 1.0e-8;
        scalar acceptable_tolerance       = 1.0e-8;
    };

    Minimum_curvature_path(const Track_by_arcs& track, const size_t N) { compute_track_by_arcs(track,N); }

    Minimum_curvature_path(const Circuit_geometry& circuit_geometry,
                           const std::vector<scalar>& s_compute_, 
                           const bool is_closed, 
                           const Options& options_) : options(options_)
    { 
        if (is_closed)
            compute_track_by_polynomial<true>(circuit_geometry, s_compute_); 
        else
            compute_track_by_polynomial<false>(circuit_geometry, s_compute_); 
    }

    std::vector<double> compute_track_by_arcs(const Track_by_arcs& track, const size_t N);

    template<bool is_closed>
    void compute_track_by_polynomial(const Circuit_geometry& circuit_geometry, const std::vector<scalar>& s_compute_);

    //! Export to XML
    std::unique_ptr<Xml_document> xml() const;

    const std::vector<double>& get_x() const { return _x; }
 private:

    class Fitness_fcn
    {
     public:
        using ADvector = std::vector<CppAD::AD<scalar>>;

        Fitness_fcn(const size_t N, const Track_by_arcs& track) : _n(N), _track(track) {}

        void operator()(ADvector& fg, const ADvector& x) const;

     private:
        size_t _n;
        Track_by_arcs _track;
    };

    template<bool is_closed>
    class Fitness_fcn_by_polynomial
    {
    public:
        struct input_names
        {
            enum { n, yaw, pitch, roll, yaw_dot, pitch_dot, roll_dot, delta_path_arclength, end };
        };

        using ADvector = std::vector<CppAD::AD<scalar>>;

        Fitness_fcn_by_polynomial(const size_t num_points, const size_t num_elements, const std::vector<double>& s_compute, const std::vector<sVector3d>& centerline, const std::vector<sVector3d>& normal_vector, const int circuit_direction)
            : _num_points(num_points), _num_elements(num_elements), 
              _num_variables((input_names::end-1)*_num_points + _num_elements), _num_constraints(_num_elements*6), _circuit_direction(circuit_direction),
              _s(s_compute), _centerline(centerline), _normal_vector(normal_vector),
              _delta_path_arclength(_num_elements, 0.0), _n(_num_points, 0.0), 
              _yaw(_num_points, 0.0), _pitch(_num_points, 0.0), _roll(_num_points, 0.0),
              _yaw_dot(_num_points, 0.0), _pitch_dot(_num_points, 0.0), _roll_dot(_num_points, 0.0),
              _curvature_x(_num_points, 0.0), _curvature_y(_num_points, 0.0), _curvature_z(_num_points, 0.0),
              _x(_num_points, 0.0), _y(_num_points, 0.0), _z(_num_points, 0.0) {}

        void operator()(ADvector& fg, const ADvector& x);

        constexpr const size_t& get_n_variables() const { return _num_variables; }

        constexpr const size_t& get_n_constraints() const { return _num_constraints; }

    private:
        size_t _num_points;
        size_t _num_elements;
        size_t _num_variables;
        size_t _num_constraints;
        int _circuit_direction;

        std::vector<double> _s;
        std::vector<sVector3d> _centerline;
        std::vector<sVector3d> _normal_vector;


    public:
        std::vector<CppAD::AD<scalar>> _delta_path_arclength;
        std::vector<CppAD::AD<scalar>> _n;
        std::vector<CppAD::AD<scalar>> _yaw;
        std::vector<CppAD::AD<scalar>> _pitch;
        std::vector<CppAD::AD<scalar>> _roll;
        std::vector<CppAD::AD<scalar>> _yaw_dot;
        std::vector<CppAD::AD<scalar>> _pitch_dot;
        std::vector<CppAD::AD<scalar>> _roll_dot;
        std::vector<CppAD::AD<scalar>> _curvature_x;
        std::vector<CppAD::AD<scalar>> _curvature_y;
        std::vector<CppAD::AD<scalar>> _curvature_z;
        std::vector<CppAD::AD<scalar>> _x;
        std::vector<CppAD::AD<scalar>> _y;
        std::vector<CppAD::AD<scalar>> _z;
    };

public:

    Options options;

    size_t num_points;
    size_t num_elements;

    std::vector<scalar> s_compute;

    bool success;
    size_t iter_count;
    std::vector<scalar> delta_path_arclength;
    std::vector<scalar> n;
    std::vector<scalar> x;
    std::vector<scalar> y;
    std::vector<scalar> z;
    std::vector<scalar> yaw;
    std::vector<scalar> pitch;
    std::vector<scalar> roll;
    std::vector<scalar> yaw_dot;
    std::vector<scalar> pitch_dot;
    std::vector<scalar> roll_dot;
    std::vector<double> _x;
};

#include "minimum_curvature_path.hpp"

#endif
