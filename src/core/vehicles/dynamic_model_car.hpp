#ifndef __CAR_HPP__
#define __CAR_HPP__

#include "lion/math/matrix_extensions.h"

template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
template<size_t NALG>
std::enable_if_t<NALG==0,std::array<Timeseries_t,_NSTATE>> Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::operator()
    (const std::array<Timeseries_t,_NSTATE>& q, const std::array<Timeseries_t,_NCONTROL>& u, scalar t)
{
    return std::get<0>((*this)(q,{},u,t));
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
        const auto it_p = std::find_if(_parameters.begin(), _parameters.end(), [&](const auto& p) -> auto { return p.get_path() == parameter; });

        if ( it_p != _parameters.cend() )
            std::fill(it_p->get_values().begin(), it_p->get_values().end(), value);
    }
}


template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
std::pair<std::array<Timeseries_t,_NSTATE>,std::array<Timeseries_t,Chassis_t::NALGEBRAIC>> Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::operator()
    (const std::array<Timeseries_t,_NSTATE>& q, const std::array<Timeseries_t,NALGEBRAIC>& qa, const std::array<Timeseries_t,_NCONTROL>& u, scalar t)
{
    std::array<Timeseries_t,NSTATE> dqdt;
    std::array<Timeseries_t,NALGEBRAIC> dqa;

    // (1) Set the variable parameters
    for (auto const& parameter : _parameters )
        get_chassis().set_parameter(parameter.get_path(), parameter(t));

    // (2) Set state and controls
    _chassis.set_state_and_controls(q,qa,u);
    _road.set_state_and_controls(t,q,u);

    // (3) Update
    _road.update(_chassis.get_u(), _chassis.get_v(), _chassis.get_omega());
    _chassis.update(_road.get_x(), _road.get_y(), _road.get_psi());

    // (4) Get time derivative
    _chassis.get_state_derivative(dqdt);
    _road.get_state_derivative(dqdt);

    // (5) Get algebraic constraints from the chassis
    if constexpr (NALGEBRAIC != 0)
        _chassis.get_algebraic_constraints(dqa);

    // (6) Scale the temporal parameter to curvilinear if needed
    for (auto it = dqdt.begin(); it != dqdt.end(); ++it)
        (*it) *= _road.get_dtimedt();

    return {dqdt,dqa};
}


template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
typename Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::Equations 
    Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::equations
        (const std::array<scalar,_NSTATE>& q, const std::array<scalar,NALGEBRAIC>& qa,
         const std::array<scalar,_NCONTROL>& u, scalar t)
{
    // Put the states into a single vector, which will be declared as independent variables
    constexpr const size_t n_total = _NSTATE + NALGEBRAIC + _NCONTROL;
    std::vector<CppAD::AD<double>> x0(n_total);

    for (size_t i = 0; i < _NSTATE; ++i)
        x0[i] = q[i];

    for (size_t i = 0; i < NALGEBRAIC; ++i)
        x0[i+_NSTATE] = qa[i];

    for (size_t i = 0; i < _NCONTROL; ++i)
        x0[i+_NSTATE+NALGEBRAIC] = u[i];

    // Declare the contents of x0 as the independent variables
    CppAD::Independent(x0);

    // Create new inputs to the operator() of the vehicle 
    std::array<CppAD::AD<double>,_NSTATE>     q0;
    std::array<CppAD::AD<double>,NALGEBRAIC> qa0;
    std::array<CppAD::AD<double>,_NCONTROL>   u0;

    for (size_t i = 0; i < _NSTATE; ++i)
        q0[i] = x0[i];

    for (size_t i = 0; i < NALGEBRAIC; ++i)
        qa0[i] = x0[i+_NSTATE];

    for (size_t i = 0; i < _NCONTROL; ++i)
        u0[i] = x0[i+_NSTATE+NALGEBRAIC];

    // Call operator(), transform arrays to vectors
    auto [dqdt_out,dqa_out] = (*this)(q0,qa0,u0,0.0);
    std::vector<CppAD::AD<double>> out_vector(dqdt_out.cbegin(), dqdt_out.cend());
    out_vector.insert(out_vector.end(), dqa_out.cbegin(), dqa_out.cend());

    // Create the AD functions and stop the recording
    CppAD::ADFun<double> f;
    f.Dependent(x0,out_vector);

    // Transform x0 to double, to evaluate the functions
    std::vector<scalar> x0_double(x0.size());

    for (size_t i = 0; i < x0.size(); ++i)
        x0_double[i] = Value(x0[i]);

    // Evaluate y = f(q0,u0,0)
    auto out0 = f.Forward(0, x0_double);
    auto out0_jacobian = f.Jacobian(x0_double);
    
    std::vector<std::vector<double>> out0_hessian(_NSTATE+NALGEBRAIC);

    for (size_t i = 0; i < _NSTATE + NALGEBRAIC; ++i)
        out0_hessian[i] = f.Hessian(x0_double,i);

    // Fill the solution struct
    Equations solution;

    for (size_t i = 0; i < _NSTATE; ++i)
        solution.dqdt[i] = out0[i];

    for (size_t i = 0; i < NALGEBRAIC; ++i)
        solution.dqa[i] = out0[i+_NSTATE];

    for (size_t i = 0; i < _NSTATE; ++i)
        for (size_t j = 0; j < n_total; ++j)
            // The CppAD jacobian is sorted as [dy1/dx1, ..., dy1/dxn, dy2/dx1, ..., dy2/dxn, ...]
            solution.jac_dqdt[i][j] = out0_jacobian[j + n_total*i];

    for (size_t i = 0; i < NALGEBRAIC; ++i)
        for (size_t j = 0; j < n_total; ++j)
            solution.jac_dqa[i][j] = out0_jacobian[j + n_total*(i+_NSTATE)];

    for (size_t var = 0; var < _NSTATE; ++var)
        for (size_t i = 0; i < n_total; ++i)
            for (size_t j = 0; j < n_total; ++j)
            {
                solution.hess_dqdt[var][i][j] = out0_hessian[var][j + n_total*i];
    
                // Check its symmetry
                assert(std::abs(out0_hessian[var][j + n_total*i]-out0_hessian[var][i + n_total*j]) 
                         < 1.0e-10*std::max(1.0,std::abs(out0_hessian[var][j+n_total*i])));

            }

    for (size_t var = 0; var < NALGEBRAIC; ++var)
        for (size_t i = 0; i < n_total; ++i)
            for (size_t j = 0; j < n_total; ++j)
            {
                solution.hess_dqa[var][i][j] = out0_hessian[var+_NSTATE][j + n_total*i];

                // Check its symmetry
                assert(std::abs(out0_hessian[var+_NSTATE][j + n_total*i]-out0_hessian[var+_NSTATE][i + n_total*j]) 
                         < 1.0e-10*std::max(1.0,std::abs(out0_hessian[var+_NSTATE][j+n_total*i])));
            }

    return solution;
}


template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
std::tuple<std::string,std::array<std::string,_NSTATE>,std::array<std::string,Chassis_t::NALGEBRAIC>,std::array<std::string,_NCONTROL>> Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::get_state_and_control_names() 
{
    std::string key_name;
    std::array<std::string,NSTATE> q_names;
    std::array<std::string,NALGEBRAIC> qa_names;
    std::array<std::string,NCONTROL> u_names;
    RoadModel_t::set_state_and_control_names(key_name,q_names,u_names);
    Chassis_t::set_state_and_control_names(q_names,qa_names,u_names);

    return {key_name,q_names,qa_names,u_names};
}



template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
typename Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::State_and_control_upper_lower_and_default_values 
    Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::get_state_and_control_upper_lower_and_default_values() const
{
    // (1) Define outputs
    std::array<scalar,_NSTATE> q_def;
    std::array<scalar,_NSTATE> q_lb;
    std::array<scalar,_NSTATE> q_ub;
    std::array<scalar,NALGEBRAIC> qa_def;
    std::array<scalar,NALGEBRAIC> qa_lb;
    std::array<scalar,NALGEBRAIC> qa_ub;
    std::array<scalar,_NCONTROL> u_def;
    std::array<scalar,_NCONTROL> u_lb;
    std::array<scalar,_NCONTROL> u_ub;

    // (2) Outputs are filled by chassis
    _chassis.set_state_and_control_upper_lower_and_default_values(q_def, q_lb, q_ub, qa_def, qa_lb, qa_ub, u_def, u_lb, u_ub);

    // (3) Outputs are filled by road
    _road.set_state_and_control_upper_lower_and_default_values(q_def, q_lb, q_ub, u_def, u_lb, u_ub);

    // (4) Return
    return (State_and_control_upper_lower_and_default_values)
    {
        .q_def  = q_def , .q_lb  = q_lb , .q_ub  = q_ub , 
        .qa_def = qa_def, .qa_lb = qa_lb, .qa_ub = qa_ub, 
        .u_def  = u_def , .u_lb  = u_lb , .u_ub  = u_ub   
    };
}

#endif
