#ifndef OPTIMAL_LAPTIME_CONTROL_VARIABLE_H
#define OPTIMAL_LAPTIME_CONTROL_VARIABLE_H

#include "src/core/applications/optimal_laptime_constants.h"

namespace fastest_lap::optimal_laptime::detail
{
    //! Helper classes to encapsulate control variables ---------------------------------------------:-

    
    //! Control variable class: contains the definition of the control variables and its type of optimization
    //! [param] T: scalar for ctor and return values (well, for any interface), CppAD::AD<scalar> for internal values
    template<typename T = scalar>
    struct Control_variable
    {
        Optimal_control_type::Type optimal_control_type; //! Type of the optimization
        std::vector<scalar> s_hypermesh;           //! Provide here the hypermesh. Only non-empty for hypermesh optimization
        std::vector<T>  controls;                  //! Control value(s)
        std::vector<T>  dcontrols_dt;              //! Control values time derivative: only non-empty for full_mesh optimization
        scalar          dissipation;               //! The dissipation associated to this control variable
    
        //! Transform the set of control variables to cppad
        template<typename U = T>
        inline auto to_CppAD() const -> std::enable_if_t<std::is_same_v<U,scalar>, Control_variable<CppAD::AD<scalar>>>;
    
        static size_t get_hypermesh_position_for_s(const std::vector<scalar>& s_hypermesh, scalar s);
    
        const T& get_hypermesh_control_value_for_s(const scalar s) const;
    
        std::pair<const T&,const T&> get_hypermesh_control_value_and_derivative_for_s(const scalar s) const;
    
        //! Set to zero the values for optimizable control variables
        Control_variable& clear();
    };

    //
    // Implementation of the helper class Control_variable -----------------------------------------------------:-
    //
    
    template<typename T>
    template<typename U>
    inline auto Control_variable<T>::to_CppAD() const -> std::enable_if_t<std::is_same_v<U,scalar>, Control_variable<CppAD::AD<scalar>>>
    {
        // (1) Define the new variable of type CppAD::AD
        Control_variable<CppAD::AD<scalar>> output;
    
        // (2) Copy optimal control type
        output.optimal_control_type = optimal_control_type; 
    
        // (3) Copy hypermesh
        output.s_hypermesh = s_hypermesh; 
    
        // (4) Copy control variable values
        output.controls = std::vector<CppAD::AD<scalar>>(controls.size());
        std::copy(controls.cbegin(), controls.cend(), output.controls.begin());
    
        output.dcontrols_dt = std::vector<CppAD::AD<scalar>>(dcontrols_dt.size());
        std::copy(dcontrols_dt.cbegin(), dcontrols_dt.cend(), output.dcontrols_dt.begin());
    
        // (5) Copy dissipation
        output.dissipation = dissipation;
    
        // (6) Return
        return output;
    }
    
    
    template<typename T>
    inline size_t Control_variable<T>::get_hypermesh_position_for_s(const std::vector<scalar>& s_hypermesh, scalar s)
    {
        // (1) Get an iterator to the first point in the hypermesh grid that *it > s
        auto it = std::upper_bound(s_hypermesh.begin(), s_hypermesh.end(), s);
    
        // (2) Error if the point found is the first node of the hypermesh
        if ( it == s_hypermesh.cbegin() )
            throw fastest_lap_exception("[ERROR] Optimal_laptime::Control_variable::get_hypermesh_value_for_s -> input s(" 
                + std::to_string(s) + ") is smaller than the first hypermesh checkpoint (" + std::to_string(s_hypermesh.front()) + ")");
    
        // (3) Compute position
        const size_t i_position = std::distance(s_hypermesh.begin(), it) - 1;
    
        return i_position;
    }
    
    
    template<typename T>
    inline const T& Control_variable<T>::get_hypermesh_control_value_for_s(const scalar s) const
    {
        // (1) Error if control variable is not of type hypermesh
        if (optimal_control_type != Optimal_control_type::HYPERMESH )
            throw fastest_lap_exception("[ERROR] Optimal_laptime::Control_variable::get_hypermesh_value_for_s can only be called for HYPERMESH parameters");
    
        // (2) Return
        return controls[get_hypermesh_position_for_s(s_hypermesh,s)]; 
    }
    
    
    template<typename T>
    inline std::pair<const T&,const T&> Control_variable<T>::get_hypermesh_control_value_and_derivative_for_s(const scalar s) const
    {
        // (1) Error if control variable is not of type hypermesh
        if (optimal_control_type != Optimal_control_type::HYPERMESH )
            throw fastest_lap_exception("[ERROR] Optimal_laptime::Control_variable::get_hypermesh_value_for_s can only be called for HYPERMESH parameters");
    
        // (2) Return
        const size_t i_position = get_hypermesh_position_for_s(s_hypermesh,s);
        return {controls[i_position], dcontrols_dt[i_position]}; 
    }
    
    
    template<typename T>
    inline Control_variable<T>& Control_variable<T>::clear()
    {
        std::fill(controls.begin(), controls.end(), 0.0);
        std::fill(dcontrols_dt.begin(), dcontrols_dt.end(), 0.0);
    
        return *this;
    }
}

#endif
