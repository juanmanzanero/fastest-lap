#ifndef DYNAMIC_MODEL_CAR_HPP
#define DYNAMIC_MODEL_CAR_HPP

#include "lion/math/matrix_extensions.h"


template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
std::array<Timeseries_t, _NSTATE> Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::transform_states_to_input_states
    (const std::array<Timeseries_t, _NSTATE>& states,const std::array<Timeseries_t, _NCONTROL>& controls) const
{
    // (1) Define input_states
    std::array<Timeseries_t, _NSTATE> input_states = states;

    // (2) Duplicate the chassis: we do not want to modify its internal state
    auto chassis = get_chassis();
    chassis.transform_states_to_input_states(states, controls, input_states);

    return input_states;
}

template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
template<size_t NALG>
std::enable_if_t<NALG==0, std::array<Timeseries_t,_NSTATE>> 
    Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::operator()
        (const std::array<Timeseries_t,_NSTATE>& states, const std::array<Timeseries_t,_NCONTROL>& controls, scalar time)
{
    const auto input_states = transform_states_to_input_states(states, controls);
    return (*this)(input_states,{},controls,time).dstates_dt;
}


template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
template<typename T>
void Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::set_parameter(const std::string& parameter, const T value) 
{ 
    // (1) Set the value of the parameter
    get_chassis().set_parameter(parameter,value); 

    // (2) Check if the parameter is an optimization parameter, if so, change its value also there.
    if constexpr ( std::is_same_v<T,scalar> || std::is_same_v<T,CppAD::AD<scalar>> )
    {
        const auto it_p = std::find_if(base_type::get_parameters().begin(), base_type::get_parameters().end(), [&](const auto& p) -> auto { return p.get_path() == parameter; });

        if ( it_p != base_type::get_parameters().cend() )
            std::fill(it_p->get_values().begin(), it_p->get_values().end(), value);
    }
}


template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
typename Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::Dynamics_equations
    Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::operator()
        (const std::array<Timeseries_t,_NSTATE>& input_states, const std::array<Timeseries_t,NALGEBRAIC>& algebraic_states, 
         const std::array<Timeseries_t,_NCONTROL>& controls, scalar time)
{
    // (1) Initialize outputs
    Dynamics_equations dynamics_equations;

    auto& states              = dynamics_equations.states;
    auto& dstates_dt          = dynamics_equations.dstates_dt;
    auto& algebraic_equations = dynamics_equations.algebraic_equations;

    // (1.1) Default to state vector being the input state vector
    std::copy(input_states.cbegin(), input_states.cend(), states.begin());

    // (2) Set the variable parameters
    for (auto const& parameter : base_type::get_parameters() )
        get_chassis().set_parameter(parameter.get_path(), parameter(time));

    // (3) Set state and controls
    _chassis.set_state_and_controls(input_states,algebraic_states,controls);
    _road.set_state_and_controls(time,input_states,controls);

    // (4) Update
    _road.update(_chassis.get_u(), _chassis.get_v(), _chassis.get_omega());
    _chassis.update(_road.get_x(), _road.get_y(), _road.get_psi());

    // (5) Get state and state time derivative
    _chassis.get_state_and_state_derivative(states,dstates_dt);
    _road.get_state_and_state_derivative(states,dstates_dt);

    // (6) Get algebraic constraints from the chassis
    if constexpr (NALGEBRAIC > 0)
        _chassis.get_algebraic_constraints(algebraic_equations);

    // (7) Scale the temporal parameter to curvilinear if needed
    for (auto it = dstates_dt.begin(); it != dstates_dt.end(); ++it)
        (*it) *= _road.get_dtimedt();

    return dynamics_equations;
}


template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
typename Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::Equations 
Dynamic_model_car<Timeseries_t, Chassis_t, RoadModel_t, _NSTATE, _NCONTROL>::equations
(const std::array<scalar, _NSTATE>& input_states, const std::array<scalar, NALGEBRAIC>& algebraic_states,
    const std::array<scalar, _NCONTROL>& controls, scalar time)
{
    // (1) Put the states into a single vector, which will be declared as independent variables
    constexpr const size_t n_total = _NSTATE + NALGEBRAIC + _NCONTROL;
    std::vector<CppAD::AD<double>> inputs_all_ad(n_total);

    std::copy(input_states.cbegin(), input_states.cend(), inputs_all_ad.begin());
    std::copy(algebraic_states.cbegin(), algebraic_states.cend(), inputs_all_ad.begin() + _NSTATE);
    std::copy(controls.cbegin(), controls.cend(), inputs_all_ad.begin() + _NSTATE + NALGEBRAIC);

    // (2) Declare the contents of x0 as the independent variables
    CppAD::Independent(inputs_all_ad);

    // (3) Create new inputs to the operator() of the vehicle 
    std::array<CppAD::AD<double>, _NSTATE>     input_states_ad;
    std::array<CppAD::AD<double>, NALGEBRAIC>  algebraic_states_ad;
    std::array<CppAD::AD<double>, _NCONTROL>   controls_ad;

    std::copy_n(inputs_all_ad.cbegin(), _NSTATE, input_states_ad.begin());
    std::copy_n(inputs_all_ad.cbegin() + _NSTATE, NALGEBRAIC, algebraic_states_ad.begin());
    std::copy(inputs_all_ad.cbegin() + _NSTATE + NALGEBRAIC, inputs_all_ad.cend(), controls_ad.begin());

    // (4) Call operator(), transform arrays to vectors
    auto dynamic_equations = (*this)(input_states_ad, algebraic_states_ad, controls_ad, 0.0);
    const auto& state_ad = dynamic_equations.states;
    const auto& dstates_dt_ad = dynamic_equations.dstates_dt;
    const auto& algebraic_equations_ad = dynamic_equations.algebraic_equations;

    // (5) Concatenate [states, dstates_dt, algebraic_equations]
    std::vector<CppAD::AD<double>> outputs_all_ad(state_ad.cbegin(), state_ad.cend());
    outputs_all_ad.insert(outputs_all_ad.end(), dstates_dt_ad.cbegin(), dstates_dt_ad.cend());
    outputs_all_ad.insert(outputs_all_ad.end(), algebraic_equations_ad.cbegin(), algebraic_equations_ad.cend());

    // (6) Create the AD functions and stop the recording
    CppAD::ADFun<double> f;
    f.Dependent(inputs_all_ad, outputs_all_ad);

    // (7) Transform inputs to double, to evaluate the functions
    std::vector<scalar> inputs_all(inputs_all_ad.size());
    std::transform(inputs_all_ad.cbegin(), inputs_all_ad.cend(), inputs_all.begin(),
        [](const auto& ad) -> auto { return Value(ad); });
    
    // (8) Evaluate y = f(q0,u0,0)
    auto outputs_all = f.Forward(0, inputs_all);
    auto jacobian_outputs_all = f.Jacobian(inputs_all);
    
    std::vector<std::vector<double>> hessian_outputs_all(_NSTATE + _NSTATE + NALGEBRAIC);

    for (size_t i = 0; i < _NSTATE + _NSTATE + NALGEBRAIC; ++i)
        hessian_outputs_all[i] = f.Hessian(inputs_all,i);

    // (9) Fill the solution struct
    Equations solution;

    // (9.1) Solution
    std::copy_n(outputs_all.cbegin()              , _NSTATE           , solution.states.begin());
    std::copy_n(outputs_all.cbegin() + _NSTATE    , _NSTATE           , solution.dstates_dt.begin());
    std::copy  (outputs_all.cbegin() + 2 * _NSTATE, outputs_all.cend(), solution.algebraic_equations.begin());

    // (9.2) Jacobians
    for (size_t i = 0; i < _NSTATE; ++i)
        for (size_t j = 0; j < n_total; ++j)
            // The CppAD jacobian is sorted row major, [dy1/dx1, ..., dy1/dxn, dy2/dx1, ..., dy2/dxn, ...]
            solution.jacobian_states[i][j] = jacobian_outputs_all[j + n_total*i];

    for (size_t i = 0; i < _NSTATE; ++i)
        for (size_t j = 0; j < n_total; ++j)
            // The CppAD jacobian is sorted row major, [dy1/dx1, ..., dy1/dxn, dy2/dx1, ..., dy2/dxn, ...]
            solution.jacobian_dstates_dt[i][j] = jacobian_outputs_all[j + n_total*(i+_NSTATE)];

    for (size_t i = 0; i < NALGEBRAIC; ++i)
        for (size_t j = 0; j < n_total; ++j)
            solution.jacobian_algebraic_equations[i][j] = jacobian_outputs_all[j + n_total*(i+2*_NSTATE)];

    // (9.3) Hessians
    for (size_t var = 0; var < _NSTATE; ++var)
        for (size_t i = 0; i < n_total; ++i)
            for (size_t j = 0; j < n_total; ++j)
            {
                solution.hessian_states[var][i][j] = hessian_outputs_all[var][j + n_total*i];
    
                // Check its symmetry
                assert(std::abs(hessian_outputs_all[var][j + n_total*i]- hessian_outputs_all[var][i + n_total*j])
                         < 1.0e-10*std::max(1.0,std::abs(hessian_outputs_all[var][j+n_total*i])));
            }

    for (size_t var = 0; var < _NSTATE; ++var)
        for (size_t i = 0; i < n_total; ++i)
            for (size_t j = 0; j < n_total; ++j)
            {
                solution.hessian_dstates_dt[var][i][j] = hessian_outputs_all[var+_NSTATE][j + n_total*i];
    
                // Check its symmetry
                assert(std::abs(hessian_outputs_all[var+_NSTATE][j + n_total*i]- hessian_outputs_all[var+_NSTATE][i + n_total*j])
                         < 1.0e-10*std::max(1.0,std::abs(hessian_outputs_all[var+_NSTATE][j+n_total*i])));
            }

    for (size_t var = 0; var < NALGEBRAIC; ++var)
        for (size_t i = 0; i < n_total; ++i)
            for (size_t j = 0; j < n_total; ++j)
            {
                solution.hessian_algebraic_equations[var][i][j] = hessian_outputs_all[var+2*_NSTATE][j + n_total*i];

                // Check its symmetry
                assert(std::abs(hessian_outputs_all[var+2*_NSTATE][j + n_total*i]- hessian_outputs_all[var+2*_NSTATE][i + n_total*j])
                         < 1.0e-10*std::max(1.0,std::abs(hessian_outputs_all[var+2*_NSTATE][j+n_total*i])));
            }

    return solution;
}


template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
std::tuple<std::string,std::array<std::string,_NSTATE>,std::array<std::string,Chassis_t::NALGEBRAIC>,std::array<std::string,_NCONTROL>> Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::get_state_and_control_names() const 
{
    std::string key_name;
    std::array<std::string,NSTATE> input_states_names;
    std::array<std::string,NALGEBRAIC> algebraic_states_names;
    std::array<std::string,NCONTROL> controls_names;

    RoadModel_t::set_state_and_control_names(key_name,input_states_names,controls_names);
    _chassis.set_state_and_control_names(input_states_names,algebraic_states_names,controls_names);

    return {key_name,input_states_names,algebraic_states_names,controls_names};
}


template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
typename Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::State_and_control_upper_lower_and_default_values 
    Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::get_state_and_control_upper_lower_and_default_values() const
{
    // (1) Define outputs
    State_and_control_upper_lower_and_default_values values_all;

    auto& input_states_def = values_all.input_states_def;
    auto& input_states_lb  = values_all.input_states_lb;
    auto& input_states_ub  = values_all.input_states_ub;

    auto& algebraic_states_def = values_all.algebraic_states_def;
    auto& algebraic_states_lb = values_all.algebraic_states_lb;
    auto& algebraic_states_ub = values_all.algebraic_states_ub;

    auto& controls_def = values_all.controls_def;
    auto& controls_lb = values_all.controls_lb;
    auto& controls_ub = values_all.controls_ub;

    // (2) Outputs are filled by chassis
    _chassis.set_state_and_control_upper_lower_and_default_values(input_states_def, input_states_lb, input_states_ub, 
        algebraic_states_def, algebraic_states_lb, algebraic_states_ub, 
        controls_def, controls_lb, controls_ub);

    // (3) Outputs are filled by road
    _road.set_state_and_control_upper_lower_and_default_values(input_states_def, input_states_lb, input_states_ub,
            controls_def, controls_lb, controls_ub);

    // (4) Return
    return values_all;
}


template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
bool Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::is_ready() const
{
    return _chassis.is_ready();
}

#endif
