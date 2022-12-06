#ifndef __DYNAMIC_MODEL_POWERED_AXLE_HPP__
#define __DYNAMIC_MODEL_POWERED_AXLE_HPP__

template<typename Timeseries_t, typename Axle_t, size_t STATE0, size_t CONTROL0>
Dynamic_model_powered_axle<Timeseries_t,Axle_t,STATE0,CONTROL0>::Dynamic_model_powered_axle(const scalar Fz, Xml_document& database)
: _axle("axle", 
        typename Axle_t::Tire_left_type("rear left tire", database, "vehicle/rear-tire/"),
        typename Axle_t::Tire_right_type("rear right tire", database, "vehicle/rear-tire/"),
        database, "vehicle/rear-axle/"),
  _road(),
  _inertial_frame(),
  _Fz(Fz)
{ 
    _axle.get_frame().set_parent(_inertial_frame); 

    const scalar k_chassis = database.get_element("vehicle/rear-axle/stiffness/chassis").get_value(scalar());
    const scalar k_tire    = database.get_element("vehicle/rear-tire/radial-stiffness").get_value(scalar());
    const scalar R0        = database.get_element("vehicle/rear-tire/radius").get_value(scalar());

    const scalar sym_stiffness = k_chassis*k_tire/(k_chassis + k_tire);
    const scalar dz_hub = Fz/sym_stiffness;

    _z = dz_hub - R0;
    _m = database.get_element("vehicle/chassis/mass").get_value(double());
} 


template<typename Timeseries_t, typename Axle_t, size_t STATE0, size_t CONTROL0>
std::array<Timeseries_t,Dynamic_model_powered_axle<Timeseries_t,Axle_t,STATE0,CONTROL0>::NSTATE>
    Dynamic_model_powered_axle<Timeseries_t,Axle_t,STATE0,CONTROL0>::operator()(
        const std::array<Timeseries_t,Dynamic_model_powered_axle<Timeseries_t,Axle_t,STATE0,CONTROL0>::NSTATE>& states, 
        const std::array<Timeseries_t,Dynamic_model_powered_axle<Timeseries_t,Axle_t,STATE0,CONTROL0>::NCONTROL>& controls, Timeseries_t t)
{
    const auto input_states = transform_states_to_inputs(states, controls);

    // (1) Set state and controls
    _axle.set_state_and_controls(input_states, controls);
    _road.set_state_and_controls(t,input_states,controls);
    _u = input_states[input_state_names::U];

    // (2) Update
    _road.update(_u,0.0,0.0);
    _axle.update({input_states[Road_type::input_state_names::X],0.0,_z}, {_u,0.0,0.0}, 0.0, 0.0);

    // (3) Set state derivative
    std::array<Timeseries_t,Dynamic_model_powered_axle<Timeseries_t,Axle_t,STATE0,CONTROL0>::NSTATE> dstates_dt;
    auto states_recompute = states;

    _axle.get_state_and_state_derivative(states_recompute, dstates_dt);
    _road.get_state_and_state_derivative(states_recompute, dstates_dt);

    const Vector3d<Timeseries_t> F_rear  = _axle.get_force();
    const Timeseries_t F_aero = 0.5*1.2*_u*_u*0.7; 
    dstates_dt[state_names::U] = (F_rear[0]-F_aero)/_m;

    return dstates_dt;
}

#endif
