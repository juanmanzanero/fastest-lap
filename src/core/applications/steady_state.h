#ifndef STEADY_STATE_H
#define STEADY_STATE_H

#include "lion/foundation/types.h"
#include "lion/foundation/utils.hpp"
#include "lion/thirdparty/include/cppad/cppad.hpp"

template<typename Dynamic_model_t>
class Steady_state
{
 public:
    using Timeseries_t = typename Dynamic_model_t::Timeseries_type;

    Steady_state(Dynamic_model_t& car) : _car(car) {};

    struct Solution
    {
        bool solved;
        scalar v;
        scalar ax;
        scalar ay;
        std::array<scalar,Dynamic_model_t::number_of_inputs> inputs;
        std::array<scalar,Dynamic_model_t::number_of_controls> controls;
        std::array<scalar,Dynamic_model_t::number_of_states> dstates_dt;
    };
    
    //! Solve with numerical Jacobian
    template<typename T = Timeseries_t>
    std::enable_if_t<std::is_same<T,scalar>::value,Solution> 
        solve(scalar v, scalar ax, scalar ay, const size_t n_steps = 1, const bool provide_x0 = false, const std::vector<scalar>& x0_provided = {}, bool throw_if_fail = true);

    //! Solve with automatic differentiation
    template<typename T = Timeseries_t>
    std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,Solution> 
        solve(scalar v, scalar ax, scalar ay, const size_t n_steps = 1, const bool provide_x0 = false, const std::vector<scalar>& x0_provided = {}, bool throw_if_fail = true);

    //! Solve max lateral acceleration with numerical Jacobian
    template<typename T = Timeseries_t>
    std::enable_if_t<std::is_same<T,scalar>::value,Solution> 
        solve_max_lat_acc(scalar v);

    //! Solve max lateral acceleration with automatic differentiation
    template<typename T = Timeseries_t>
    std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,Solution> 
        solve_max_lat_acc(scalar v);

    //! Solve max longitudinal acceleration with numerical Jacobian
    template<typename T = Timeseries_t>
    std::enable_if_t<std::is_same<T,scalar>::value,std::pair<Solution,Solution>>
        solve_max_lon_acc(scalar v, scalar ay);

    //! Solve max longitudinal acceleration with automatic differentiation
    template<typename T = Timeseries_t>
    std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,std::pair<Solution,Solution>>
        solve_max_lon_acc(scalar v, scalar ay);

    template<typename T = Timeseries_t>
    std::enable_if_t<std::is_same<T,scalar>::value,std::pair<std::vector<Solution>,std::vector<Solution>>>
        gg_diagram(scalar v, const size_t n_points);

    template<typename T = Timeseries_t>
    std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,std::pair<std::vector<Solution>,std::vector<Solution>>>
        gg_diagram(scalar v, const size_t n_points); 

 private:
    Dynamic_model_t _car;

    // Private auxiliary functors to call optimise
    class Solve_fitness
    {
     public:
        using argument_type = std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_variables>;
        Timeseries_t operator()(const argument_type& q) { return 0.0; };
    };

    class Solve_constraints
    {
     public:
        using argument_type = std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_variables>;
        using output_type   = std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_equations>;
        Solve_constraints(Dynamic_model_t& car, scalar v, scalar ax, scalar ay) : _car(&car), _v(v), _ax(ax), _ay(ay), _inputs(), 
                                                                                  _controls() {}

        output_type operator()(const argument_type& x);

        const std::array<Timeseries_t,Dynamic_model_t::number_of_inputs>& get_inputs() const { return _inputs; }
        const std::array<Timeseries_t,Dynamic_model_t::number_of_controls>& get_controls() const { return _controls; }

     private:
        Dynamic_model_t* _car;
        scalar _v;
        scalar _ax;
        scalar _ay;

        std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> _inputs;
        std::array<Timeseries_t,Dynamic_model_t::number_of_controls> _controls;
    };

    class Solve
    {
     public:
        using ADvector = std::vector<Timeseries_t>;

        Solve(Dynamic_model_t& car, scalar v, scalar ax, scalar ay): _f(), _g(car,v,ax,ay) {}

        void operator()(ADvector& fg, const ADvector& x) 
        {
            assert(x.size() == Dynamic_model_t::number_of_steady_state_variables);
            assert(fg.size() == (1+Dynamic_model_t::number_of_steady_state_equations));

            // Put x into a std::array
            std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_variables> x_array;
            std::copy_n(x.begin(), x.size(), x_array.begin());

            // Compute f and g
            Timeseries_t f = _f(x_array);
            std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_equations> g = _g(x_array);

            fg[0] = f;

            for (size_t i = 0; i < (Dynamic_model_t::number_of_steady_state_equations); ++i)
                fg[i+1] = g[i];
        }
    
     private:
        Solve_fitness _f;
        Solve_constraints _g;
    };

    // Private auxiliary functors to call optimise
    class Max_lat_acc_fitness
    {
     public:
        using argument_type = std::array<Timeseries_t,2+Dynamic_model_t::number_of_steady_state_variables>;
        Timeseries_t operator()(const argument_type& x) { return -x.back(); };
    };

    class Max_lat_acc_constraints
    {
     public:
        using argument_type = std::array<Timeseries_t,2+Dynamic_model_t::number_of_steady_state_variables>;
        using output_type   = std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_equations>;
        Max_lat_acc_constraints(Dynamic_model_t& car, scalar v) : _car(&car), _v(v), _inputs(), _controls() {}

        output_type operator()(const argument_type& x);

        const std::array<Timeseries_t,Dynamic_model_t::number_of_inputs>& get_inputs() const { return _inputs; }
        const std::array<Timeseries_t,Dynamic_model_t::number_of_controls>& get_controls() const { return _controls; }

     private:
        Dynamic_model_t* _car;
        scalar _v;

        std::array<Timeseries_t,Dynamic_model_t::number_of_inputs>     _inputs;
        std::array<Timeseries_t,Dynamic_model_t::number_of_controls>   _controls;
    };

    class Max_lat_acc
    {
     public:
        using ADvector = std::vector<Timeseries_t>;

        Max_lat_acc(Dynamic_model_t& car, scalar v): _f(), _g(car,v) {}

        void operator()(ADvector& fg, const ADvector& x) 
        {
            assert(x.size() == 2+Dynamic_model_t::number_of_steady_state_variables);
            assert(fg.size() == (1+Dynamic_model_t::number_of_steady_state_equations));

            // Put x into a std::array
            std::array<Timeseries_t,2+Dynamic_model_t::number_of_steady_state_variables> x_array;
            std::copy_n(x.begin(), x.size(), x_array.begin());

            // Compute f and g
            Timeseries_t f = _f(x_array);
            std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_equations> g = _g(x_array);

            fg[0] = f;

            for (size_t i = 0; i < (Dynamic_model_t::number_of_steady_state_equations); ++i)
                fg[i+1] = g[i];
        }
    
     private:
        Max_lat_acc_fitness _f;
        Max_lat_acc_constraints _g;
    };

    class Max_lon_acc_fitness
    {
     public:
        using argument_type = std::array<Timeseries_t,1+Dynamic_model_t::number_of_steady_state_variables>;
        Timeseries_t operator()(const argument_type& x) { return -x.back(); };
    };

    class Min_lon_acc_fitness
    {
     public:
        using argument_type = std::array<Timeseries_t,1+Dynamic_model_t::number_of_steady_state_variables>;
        Timeseries_t operator()(const argument_type& x) { return x.back(); };
    };

    class Max_lon_acc_constraints
    {
     public:
        using argument_type = std::array<Timeseries_t,1+Dynamic_model_t::number_of_steady_state_variables>;
        using output_type   = std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_equations>;
        Max_lon_acc_constraints(Dynamic_model_t& car, scalar v, scalar ay) : _car(&car), _v(v), _ay(ay), _inputs(), _controls() {}

        output_type operator()(const argument_type& x);

        const std::array<Timeseries_t,Dynamic_model_t::number_of_inputs>& get_inputs() const { return _inputs; }
        const std::array<Timeseries_t,Dynamic_model_t::number_of_controls>& get_controls() const { return _controls; }

     private:
        Dynamic_model_t* _car;
        scalar _v;
        scalar _ay;

        std::array<Timeseries_t,Dynamic_model_t::number_of_inputs>     _inputs;
        std::array<Timeseries_t,Dynamic_model_t::number_of_controls>   _controls;
    };

    class Max_lon_acc
    {
     public:
        using ADvector = std::vector<Timeseries_t>;

        Max_lon_acc(Dynamic_model_t& car, scalar v, scalar ay): _f(), _g(car,v,ay) {}

        void operator()(ADvector& fg, const ADvector& x)
        {
            assert(x.size() == 1+Dynamic_model_t::number_of_steady_state_variables);
            assert(fg.size() == (1+Dynamic_model_t::number_of_steady_state_equations));

            // Put x into a std::array
            std::array<Timeseries_t,1+Dynamic_model_t::number_of_steady_state_variables> x_array;
            std::copy_n(x.begin(), x.size(), x_array.begin());

            // Compute f and g
            Timeseries_t f = _f(x_array);
            std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_equations> g = _g(x_array);

            fg[0] = f;

            for (size_t i = 0; i < (Dynamic_model_t::number_of_steady_state_equations); ++i)
                fg[i+1] = g[i];
        }
    
     private:
        Max_lon_acc_fitness _f;
        Max_lon_acc_constraints _g;
    };

    class Min_lon_acc
    {
     public:
        using ADvector = std::vector<Timeseries_t>;

        Min_lon_acc(Dynamic_model_t& car, scalar v, scalar ay): _f(), _g(car,v,ay) {}

        void operator()(ADvector& fg, const ADvector& x) 
        {
            assert(x.size() == 1+Dynamic_model_t::number_of_steady_state_variables);
            assert(fg.size() == (1+Dynamic_model_t::number_of_steady_state_equations));

            // Put x into a std::array
            std::array<Timeseries_t,1+Dynamic_model_t::number_of_steady_state_variables> x_array;
            std::copy_n(x.begin(), x.size(), x_array.begin());

            // Compute f and g
            Timeseries_t f = _f(x_array);
            std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_equations> g = _g(x_array);

            fg[0] = f;

            for (size_t i = 0; i < (Dynamic_model_t::number_of_steady_state_equations); ++i)
                fg[i+1] = g[i];
        }
    
     private:
        Min_lon_acc_fitness _f;
        Max_lon_acc_constraints _g;
    };

};

#include "steady_state.hpp"

#endif
