#ifndef __CAR_HPP__
#define __CAR_HPP__

#include "lion/math/matrix_extensions.h"

template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
std::array<Timeseries_t,_NSTATE> Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::operator()(const std::array<Timeseries_t,_NSTATE>& q, const std::array<Timeseries_t,_NCONTROL>& u, scalar t)
{
    std::array<Timeseries_t,NSTATE> dqdt;

    // (1) Set state and controls
    _chassis.set_state_and_controls(q,u);
    _road.set_state_and_controls(t,q,u);

    // (2) Update
    _road.update(_chassis.get_u(), _chassis.get_v(), _chassis.get_omega());
    _chassis.update(_road.get_x(), _road.get_y(), _road.get_psi());

    // (3) Get time derivative
    _chassis.get_state_derivative(dqdt);
    _road.get_state_derivative(dqdt);

    // (4) Scale the temporal parameter to curvilinear if needed
    for (auto it = dqdt.begin(); it != dqdt.end(); ++it)
        (*it) *= _road.get_dtimedt();

    return dqdt;
}


template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
std::tuple<std::string,std::array<std::string,_NSTATE>,std::array<std::string,_NCONTROL>> Dynamic_model_car<Timeseries_t,Chassis_t,RoadModel_t,_NSTATE,_NCONTROL>::get_state_and_control_names() const
{
    std::string key_name;
    std::array<std::string,NSTATE> q_names;
    std::array<std::string,NCONTROL> u_names;
    _road.set_state_and_control_names(key_name,q_names,u_names);
    _chassis.set_state_and_control_names(q_names,u_names);

    return {key_name,q_names,u_names};
}

#endif
