#ifndef DYNAMIC_MODEL_CAR_H
#define DYNAMIC_MODEL_CAR_H

#include <map>
#include <vector>
#include "lion/foundation/types.h"
#include "dynamic_model.h"

//!      The dynamic model of a Car
//!      --------------------------
//!
//!  This model takes a car chassis and a road, and provides an ODE functor to be used
//! by a propagator of the form dqdt = operator()(q,u,t)
//! @param Chassis_t: type of the chassis
//! @param RoadModel_t: type of the road model
//! @param _NSTATE: number of state variables
//! @param number_of_controls: number of control variables
template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t>
class Dynamic_model_car : public Dynamic_model<Timeseries_t>
{
 public: 
    using base_type = Dynamic_model<Timeseries_t>;

    using Timeseries_type = Timeseries_t;

    //! The chassis type
    using Chassis_type = Chassis_t;
   
    //! The road model type
    using Road_type = RoadModel_t;

    //! The number of state variables
    constexpr static const size_t number_of_inputs           = Road_type::input_names::end;
    constexpr static const size_t number_of_states           = Road_type::state_names::end;
    constexpr static const size_t number_of_controls         = Road_type::control_names::end;

    static_assert(number_of_inputs == number_of_states);

    //! Default constructor
    Dynamic_model_car(const RoadModel_t& road = RoadModel_t() ) : _chassis(), _road(road) {}

    //! Constructor from parameter map and tire types
    //! @param[in] parameters: parameters map
    //! @param[in] front_left_tire_type: type of the front left tire 
    //! @param[in] front_right_tire_type: type of the front right tire 
    //! @param[in] rear_left_tire_type: type of the rear left tire 
    //! @param[in] rear_right_tire_type: type of the rear right tire 
    //! @param[in] road: the road
    Dynamic_model_car(Xml_document& database, const RoadModel_t& road = RoadModel_t()) 
        : _chassis(database), _road(road)
    {
        if( !database_parameters_all_used(database.get_root_element()) )
        {
            std::ostringstream s_out;
            s_out << "[ERROR] Dynamic_model_car -> not all parameters were used" << std::endl;
            database.print(s_out);
            throw fastest_lap_exception(s_out.str());
        }
    }

    //! Modifyer to set a parameter
    template<typename T>
    void set_parameter(const std::string& parameter, const T value);

    //! Get the input states for given states
    std::array<Timeseries_t,number_of_inputs>
        transform_states_to_inputs(const std::array<Timeseries_t, number_of_states>& states, 
                                   const std::array<Timeseries_t, number_of_controls>& controls 
                                  ) const;


    //! The time derivative functor, dqdt = operator()(q,u,t)
    //! Only enabled if the dynamic model has no algebraic equations
    //! @param[in] x: input state vector
    //! @param[in] u: controls vector
    //! @param[in] t: time/arclength
    std::array<Timeseries_t,number_of_states> ode(const std::array<Timeseries_t,number_of_states>& states, 
                                                  const std::array<Timeseries_t,number_of_controls>& controls,
                                                  scalar time);

    //! Get states and their time derivative: (q,dqdt) = operator()(x,u,t)
    //! @param[in] inputs: input state vector, states = f(inputs)
    //! @param[in] algebraic_states: states which do not have a time derivative associated
    //! @param[in] u: controls vector
    //! @param[in] t: time/arclength
    struct Dynamics_equations
    {
        std::array<Timeseries_t, number_of_states> states;
        std::array<Timeseries_t, number_of_states> dstates_dt;
    };

    Dynamics_equations operator()(const std::array<Timeseries_t,number_of_inputs>& inputs,
                                  const std::array<Timeseries_t,number_of_controls>& controls,
                                  scalar time);

    //! The time derivative functor + algebraic equations, their Jacobians, and Hessians
    //! @param[in] inputs: input state vector
    //! @param[in] controls: controls vector
    //! @param[in] t: time/arclength
    struct Equations
    {
        // Values
        std::array<scalar,number_of_states> states;
        std::array<scalar,number_of_states> dstates_dt;

        // Jacobians: jac[i] represents the Jacobian of the i-th variable w.r.t. the pack (inputs,controls)
        std::array<std::array<scalar,number_of_inputs+number_of_controls>,number_of_states> jacobian_states; 
        std::array<std::array<scalar,number_of_inputs+number_of_controls>,number_of_states> jacobian_dstates_dt; 

        // Hessians: hess[i] represents the Hessian of the i-th variable
        std::array<std::array<std::array<scalar,number_of_inputs+number_of_controls>,number_of_inputs+number_of_controls>,number_of_states> hessian_states;     
        std::array<std::array<std::array<scalar,number_of_inputs+number_of_controls>,number_of_inputs+number_of_controls>,number_of_states> hessian_dstates_dt;     
    };

    auto equations(const std::array<scalar,number_of_inputs>& inputs,
                   const std::array<scalar,number_of_controls>& controls,
                   scalar time) -> Equations;

    std::tuple<std::string,std::array<std::string,number_of_inputs>,std::array<std::string,number_of_controls>> get_state_and_control_names() const;

    //! Get state and control upper and lower values
    struct State_and_control_upper_lower_and_default_values
    {
        std::array<scalar,number_of_inputs> inputs_def;
        std::array<scalar,number_of_inputs> inputs_lb;
        std::array<scalar,number_of_inputs> inputs_ub;
        std::array<scalar,number_of_controls> controls_def;
        std::array<scalar,number_of_controls> controls_lb;
        std::array<scalar,number_of_controls> controls_ub;
    };

    State_and_control_upper_lower_and_default_values get_state_and_control_upper_lower_and_default_values() const;

    //! Return a bool that states that the model is "ready"
    bool is_ready() const;

    //! Return the chassis
    constexpr const Chassis_t& get_chassis() const { return _chassis; }
    constexpr       Chassis_t& get_chassis()       { return _chassis; }

    //! Return the road
    constexpr const RoadModel_t& get_road() const { return _road; }

    //! Update the track at a given point
    constexpr void update_track_at_arclength(const scalar s) { _road.update_track(s); }

    //! Change the track
    template<typename U = RoadModel_t>
    std::enable_if_t<road_is_curvilinear<U>::value, void> change_track(const typename U::Track_type& new_track) { _road.change_track(new_track); }

    //! Write as xml
    std::unique_ptr<Xml_document> xml() const
    {
        std::unique_ptr<Xml_document> doc_ptr(std::make_unique<Xml_document>());
        doc_ptr->create_root_element("vehicle");

        _chassis.fill_xml(*doc_ptr);

        return doc_ptr;
    }

    std::unordered_map<std::string,Timeseries_t> get_outputs_map() const { return _chassis.get_outputs_map(); }

    struct Equations_classification
    {
        std::vector<size_t> time_derivative;
        std::vector<size_t> algebraic;
    };

    Equations_classification classify_equations() const
    {
        // By default, set all equations as time derivative, but remove the time
        std::vector<size_t> time_derivative(number_of_states - 1);
        std::vector<size_t> algebraic{};

        size_t equation_id = 0;
        std::for_each(time_derivative.begin(), time_derivative.begin() + Road_type::state_names::time, [&](auto& eq_id) { eq_id = equation_id++; });
        std::for_each(time_derivative.begin() + Road_type::state_names::time, time_derivative.end(), [&](auto& eq_id) { eq_id = ++equation_id; });

        if (equation_id != number_of_states-1)
            throw fastest_lap_exception("[ERROR] classify_equations() -> Not all equations (except time) were assigned");

        return Equations_classification
        {
            .time_derivative = time_derivative,
            .algebraic = algebraic
        };
    }

 private:
    Frame<Timeseries_t> _inertial_frame;    //! Inertial frame that serves as an absolute coordinate system
    Chassis_t _chassis;                     //! The chassis
    RoadModel_t _road;                      //! The road
};

#include "dynamic_model_car.hpp"

#endif
