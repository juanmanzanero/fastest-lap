#ifndef __DYNAMIC_MODEL_CAR_H__
#define __DYNAMIC_MODEL_CAR_H__

#include <map>
#include <vector>
#include "lion/foundation/types.h"
#include "src/core/chassis/chassis_car_6dof.h"
#include "src/core/chassis/axle_car_6dof.h"
#include "src/core/tire/tire_pacejka.h"
#include "dynamic_model.h"

//!      The dynamic model of a Car
//!      --------------------------
//!
//!  This model takes a car chassis and a road, and provides an ODE functor to be used
//! by a propagator of the form dqdt = operator()(q,u,t)
//! @param Chassis_t: type of the chassis
//! @param RoadModel_t: type of the road model
//! @param _NSTATE: number of state variables
//! @param _NCONTROL: number of control variables
template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
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
    constexpr static size_t NSTATE    = _NSTATE;

    //! The number of algebraic variables
    constexpr static size_t NALGEBRAIC = Chassis_t::NALGEBRAIC;

    //! The number of control variables
    constexpr static size_t NCONTROL  = _NCONTROL; 

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
        : _chassis(database), _road(road) {};

    //! Modifyer to set a parameter
    template<typename T>
    void set_parameter(const std::string& parameter, const T value);

    //! The time derivative functor, dqdt = operator()(q,u,t)
    //! Only enabled if the dynamic model has no algebraic equations
    //! @param[in] q: state vector
    //! @param[in] u: controls vector
    //! @param[in] t: time/arclength
    template<size_t NALG = NALGEBRAIC>
    std::enable_if_t<NALG==0,std::array<Timeseries_t,_NSTATE>> operator()(const std::array<Timeseries_t,_NSTATE>& q, 
                                                                          const std::array<Timeseries_t,_NCONTROL>& u,
                                                                          scalar t);

    //! The time derivative functor + algebraic equations: dqdt,dqa = operator()(q,qa,u,t)
    //! @param[in] q: state vector
    //! @param[in] qa: constraint variables vector
    //! @param[in] u: controls vector
    //! @param[in] t: time/arclength
    std::pair<std::array<Timeseries_t,_NSTATE>,std::array<Timeseries_t,Chassis_t::NALGEBRAIC>> operator()(const std::array<Timeseries_t,_NSTATE>& q,
                                                                                                          const std::array<Timeseries_t,NALGEBRAIC>& qa,
                                                                                                          const std::array<Timeseries_t,_NCONTROL>& u,
                                                                                                          scalar t);

    //! The time derivative functor + algebraic equations, their Jacobians, and Hessians
    //! @param[in] q: state vector
    //! @param[in] qa: constraint variables vector
    //! @param[in] u: controls vector
    //! @param[in] t: time/arclength
    struct Equations
    {
        // Values
        std::array<scalar,_NSTATE> dqdt;
        std::array<scalar,NALGEBRAIC> dqa;

        // Jacobians: jac[i] represents the Jacobian of the i-th variable
        std::array<std::array<scalar,_NSTATE+NALGEBRAIC+_NCONTROL>,_NSTATE> jac_dqdt; 
        std::array<std::array<scalar,_NSTATE+NALGEBRAIC+_NCONTROL>, NALGEBRAIC> jac_dqa;

        // Hessians: hess[i] represents the Hessian of the i-th variable
        std::array<std::array<std::array<scalar,_NSTATE+NALGEBRAIC+_NCONTROL>,_NSTATE+NALGEBRAIC+_NCONTROL>,_NSTATE> hess_dqdt;     
        std::array<std::array<std::array<scalar,_NSTATE+NALGEBRAIC+_NCONTROL>,_NSTATE+NALGEBRAIC+_NCONTROL>,NALGEBRAIC> hess_dqa;
    };

    Equations equations(const std::array<scalar,_NSTATE>& q,
                        const std::array<scalar,NALGEBRAIC>& qa,
                        const std::array<scalar,_NCONTROL>& u,
                        scalar t);

    static std::tuple<std::string,std::array<std::string,_NSTATE>,std::array<std::string,Chassis_t::NALGEBRAIC>,std::array<std::string,_NCONTROL>> 
        get_state_and_control_names();

    //! Get state and control upper and lower values
    struct State_and_control_upper_lower_and_default_values
    {
        std::array<scalar,_NSTATE> q_def;
        std::array<scalar,_NSTATE> q_lb;
        std::array<scalar,_NSTATE> q_ub;
        std::array<scalar,NALGEBRAIC> qa_def;
        std::array<scalar,NALGEBRAIC> qa_lb;
        std::array<scalar,NALGEBRAIC> qa_ub;
        std::array<scalar,_NCONTROL> u_def;
        std::array<scalar,_NCONTROL> u_lb;
        std::array<scalar,_NCONTROL> u_ub;
    };

    State_and_control_upper_lower_and_default_values get_state_and_control_upper_lower_and_default_values() const;

    //! Return a bool that states that the model is "ready"
    bool is_ready() const;

    //! Return the chassis
    constexpr const Chassis_t& get_chassis() const { return _chassis; }
    constexpr       Chassis_t& get_chassis()       { return _chassis; }

    //! Return the road
    constexpr const RoadModel_t& get_road() const { return _road; }
    constexpr       RoadModel_t& get_road()       { return _road; }

    //! Write as xml
    std::unique_ptr<Xml_document> xml() const
    {
        std::unique_ptr<Xml_document> doc_ptr(std::make_unique<Xml_document>());
        doc_ptr->create_root_element("vehicle");

        _chassis.fill_xml(*doc_ptr);

        return doc_ptr;
    }

 private:
    Chassis_t _chassis;    //! The chassis
    RoadModel_t _road;     //! The road

};

#include "dynamic_model_car.hpp"

#endif
