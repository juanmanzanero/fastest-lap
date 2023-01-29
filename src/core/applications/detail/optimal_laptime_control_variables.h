#ifndef OPTIMAL_LAPTIME_CONTROL_VARIABLES_H
#define OPTIMAL_LAPTIME_CONTROL_VARIABLES_H

#include "src/core/applications/detail/optimal_laptime_control_variable.h"

namespace fastest_lap::optimal_laptime::detail
{
    //! Control variables class: provide a container for the control variables + its statistics and 
    template<size_t number_of_controls, typename T = scalar>
    struct Control_variables : public std::array<Control_variable<T>, number_of_controls>
    {
        //! Store the base_type
        using base_type = std::array<Control_variable<T>,number_of_controls>;
        using Control_variable_type = Control_variable<T>;

        //! Transform the set of control variables to cppad
        template<typename U = T>
        std::enable_if_t<std::is_same_v<U,scalar>,Control_variables<number_of_controls, CppAD::AD<scalar>>> to_CppAD() const;

        //! Clear
        Control_variables& clear();

        template<typename Dynamic_model_t>
        std::array<T,number_of_controls> control_array_at_s(const Dynamic_model_t& car, const size_t i_fullmesh, const scalar s) const;

        template<typename Dynamic_model_t>
        std::pair<std::array<T,number_of_controls>,std::array<T,number_of_controls>>
            control_array_and_derivative_at_s(const Dynamic_model_t& car, const size_t i_fullmesh, const scalar s) const;

        Control_variables& check();

        //! Data members -------------------------------------:-
        size_t number_of_constant_optimizations;            //! Number of control variables with constant optimization
        size_t number_of_hypermesh_optimization_points;     //! Total number of points used in hypermesh optimizations
        size_t number_of_full_optimizations;                //! Total number of full optimizations

     private:
    
        //! Checks the consistency of the sizes of the input vectors depending on the optimization case
        void check_inputs();
    
        //! Computes the integer data members
        void compute_statistics();
    };

    //
    // Implementation of the helper class Control_variables ----------------------------------------------------:-
    //
    
    template<size_t number_of_controls, typename T>
    inline void Control_variables<number_of_controls, T>::check_inputs()
    {
        for (auto& control_variable : *this)
        {
            switch(control_variable.optimal_control_type)
            {
             case(Optimal_control_type::DONT_OPTIMIZE):
                // Check that s_hypermesh is empty, and that controls only contains one value 
                if ( control_variable.s_hypermesh.size() > 0 ) 
                    throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                        "In \"don't optimize\" mode, s_hypermesh should be empty");
    
                if ( control_variable.controls.size() > 0 )
                    throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                        "In \"don't optimize\" mode, controls should be empty. Its size is " + std::to_string(control_variable.controls.size()) );
    
                if ( control_variable.dcontrols_dt.size() != 0 )
                    throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                        "In \"don't optimize\" mode, dcontrols_dt should be empty");
    
                control_variable.dissipation = 0.0;
    
                break;
    
             case(Optimal_control_type::CONSTANT):
                // Check that s_hypermesh is empty, and that controls only contains one value 
                if ( control_variable.s_hypermesh.size() > 0 ) 
                    throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                        "In \"constant optimization\" mode, s_hypermesh should be empty");
    
                if ( control_variable.controls.size() != 1 )
                    throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                        "In \"constant optimization\" mode, controls should contain only one value");
    
                if ( control_variable.dcontrols_dt.size() != 0 )
                    throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                        "In \"constant optimization\" mode, dcontrols_dt should be empty");
    
                control_variable.dissipation = 0.0;
    
                break;
             case(Optimal_control_type::HYPERMESH):
                // Check that the size of s_hypermesh equals the size of controls
                if ( control_variable.s_hypermesh.size() != control_variable.controls.size() )
                    throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                        "In \"hypermesh optimization\" mode, size(s_hypermesh) should be equal to size(controls)");
    
                if ( control_variable.dcontrols_dt.size() != 0 )
                    throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                        "In \"hypermesh optimization\" mode, dcontrols_dt should be empty");
    
                control_variable.dissipation = 0.0;
    
                break;
             case(Optimal_control_type::FULL_MESH): 
                // Check that s_hypermesh is empty, the size of controls will be checked later 
                if ( control_variable.s_hypermesh.size() > 0 ) 
                    throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                        "In \"full-mesh optimize\" mode, s_hypermesh should be empty");
    
                if ( (control_variable.controls.size() != control_variable.dcontrols_dt.size()) && (control_variable.dcontrols_dt.size() != 0) )
                    throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                        "In \"full-mesh optimize\" mode, dcontrols_dt should either have the same size as controls, or be empty");
    
                if ( control_variable.dissipation < 0.0 )
                    throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> dissipation must be non-negative");
    
                break;
             default:
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> optimization mode not recognized");
        
                break;
            }
        }
    }
    
    
    template<size_t number_of_controls, typename T>
    inline void Control_variables<number_of_controls, T>::compute_statistics()
    {
        // (1) Reset values
        number_of_constant_optimizations        = 0;
        number_of_hypermesh_optimization_points = 0;
        number_of_full_optimizations            = 0;
    
        for (const auto& control_variable : *this)
        {
            switch(control_variable.optimal_control_type)
            {
             case(Optimal_control_type::DONT_OPTIMIZE):
                // Do nothing
                break;
             case(Optimal_control_type::CONSTANT):
                ++number_of_constant_optimizations;
                break;
             case(Optimal_control_type::HYPERMESH):
                number_of_hypermesh_optimization_points += control_variable.s_hypermesh.size();
                break;
             case(Optimal_control_type::FULL_MESH): 
                ++number_of_full_optimizations;
                break;
             default:
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> optimization mode not recognized");
        
                break;
            }
        }
    }
    
    
    template<size_t number_of_controls, typename T>
    template<typename U>
    inline auto Control_variables<number_of_controls, T>::to_CppAD() const -> std::enable_if_t<std::is_same_v<U,scalar>, Control_variables<number_of_controls, CppAD::AD<scalar>>> 
    {
        // (1) Create return value
        Control_variables<number_of_controls, CppAD::AD<scalar>> output;
    
        // (2) Copy the std::array storage
        std::transform(base_type::cbegin(), base_type::cend(), output.begin(), [](const auto& scalar_control) -> auto { return scalar_control.to_CppAD(); });
    
        // (3) Copy the statistics
        output.number_of_constant_optimizations        = number_of_constant_optimizations;
        output.number_of_hypermesh_optimization_points = number_of_hypermesh_optimization_points;
        output.number_of_full_optimizations            = number_of_full_optimizations;
    
        // (4) Return
        return output;
    }
    
    template<size_t number_of_controls, typename T>
    inline Control_variables<number_of_controls, T>& Control_variables<number_of_controls, T>::clear()
    {
        std::transform(base_type::begin(), base_type::end(), base_type::begin(), [](auto& input) -> auto { return input.clear(); });
        return *this;
    }
    
    
    template<size_t number_of_controls, typename T>
    template<typename Dynamic_model_t>
    inline std::array<T,number_of_controls> Control_variables<number_of_controls, T>::control_array_at_s(const Dynamic_model_t& car, const size_t i_fullmesh, const scalar s) const
    {
        static_assert(number_of_controls == Dynamic_model_t::number_of_controls);

        std::array<scalar,number_of_controls> controls_scalar = car.get_state_and_control_upper_lower_and_default_values().controls_def;
        std::array<T,number_of_controls> controls;
    
        std::copy(controls_scalar.cbegin(), controls_scalar.cend(), controls.begin());
    
        for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
        {
            switch((*this)[j].optimal_control_type)
            {
             case (Optimal_control_type::DONT_OPTIMIZE):
                // Keep the default value
                break;
    
             case (Optimal_control_type::CONSTANT):
                controls[j] = (*this)[j].controls.front();
                break;
    
             case (Optimal_control_type::HYPERMESH):
                controls[j] = (*this)[j].get_hypermesh_control_value_for_s(s);
                break;
    
             case (Optimal_control_type::FULL_MESH):
                controls[j] = (*this)[j].controls[i_fullmesh];
                break;
            }
        }
        
        return controls;
    }
    
    
    template<size_t number_of_controls, typename T>
    template<typename Dynamic_model_t>
    inline std::pair<std::array<T,number_of_controls>,std::array<T,number_of_controls>>
        Control_variables<number_of_controls, T>::control_array_and_derivative_at_s
            (const Dynamic_model_t& car, const size_t i_fullmesh, const scalar s) const
    {
        static_assert(number_of_controls == Dynamic_model_t::number_of_controls);

        std::array<T,number_of_controls> controls = car.get_state_and_control_upper_lower_and_default_values().controls_def;
        std::array<T,number_of_controls> dcontrols_dt{0.0};
    
        for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
        {
            switch((*this)[j].optimal_control_type)
            {
             case (Optimal_control_type::DONT_OPTIMIZE):
                dcontrols_dt[j] = 0.0;
                break;
    
             case (Optimal_control_type::CONSTANT):
                controls[j] = (*this)[j].controls.front();
                dcontrols_dt[j] = 0.0;
                break;
    
             case (Optimal_control_type::HYPERMESH):
                controls[j] = (*this)[j].get_hypermesh_control_value_for_s(s);
                dcontrols_dt[j] = 0.0;
                break;
    
             case (Optimal_control_type::FULL_MESH):
                controls[j] = (*this)[j].controls[i_fullmesh];
                dcontrols_dt[j] = (*this)[j].dcontrols_dt[i_fullmesh];
                break;
            }
        }
    
        return {controls,dcontrols_dt};
    }
    
    
    template<size_t number_of_controls, typename T>
    inline Control_variables<number_of_controls, T>& Control_variables<number_of_controls, T>::check()
    {
        check_inputs();
        compute_statistics();
        return *this;
    }
}
#endif
