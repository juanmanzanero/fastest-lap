#ifndef __DYNAMIC_MODEL_CAR_H__
#define __DYNAMIC_MODEL_CAR_H__

#include <map>
#include <vector>
#include "lion/foundation/types.h"
#include "src/core/chassis/chassis_car_6dof.h"
#include "src/core/chassis/axle_car_6dof.h"
#include "src/core/tire/tire_pacejka.h"

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
class Dynamic_model_car
{
 public: 

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
        : _chassis(database), _road(road), _parameters() {};

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

    //! Handling of parameters ------------------------------------------:-
    template<typename ... Args> 
    void add_parameter(const std::string& parameter_path, Args&& ... args)
    {
        _parameters.emplace_back(parameter_path, std::forward<Args>(args)...);
    }

    // Set all the parameters from a hyper-vector
    void set_all_parameters(const std::vector<Timeseries_t>& inputs) { _parameters.set_all_parameters(inputs); }

 private:
    Chassis_t _chassis;    //! The chassis
    RoadModel_t _road;     //! The road

    class Parameter
    {
     public:
        //! Construct a constant parameter
        Parameter(const std::string& path, const scalar& value) 
        : _path(path), _values{value}, _mesh{} 
        {}

        //! Construct a varying parameter
        Parameter(const std::string& path, const std::vector<scalar>& values, const std::vector<std::pair<scalar,size_t>>& mesh) 
        : _path(path), _values(values.size()), _mesh(mesh)
        {
            // (1) Copy the parameters, which will transform their values from scalar to Timeseries_t
            std::copy(values.cbegin(), values.cend(), _values.begin());

            // (2) Check that there are at least two parameters
            if ( _values.size() < 2 )
                throw std::runtime_error("[ERROR] A spatially-varying parameter should be provided with at least two parameters");

            // (3) Check that the mesh is sorted
            if ( !std::is_sorted(_mesh.cbegin(), _mesh.cend(), [](const auto& lhs, const auto& rhs) -> auto { return lhs.first <= rhs.first; }) )
                throw std::runtime_error("[ERROR] Mesh checkpoints should be sorted in ascending order");
    
            // (4) Check that all parameters are used
            std::vector<bool> used_parameters(_values.size(), false);

            for (const auto& [s_i, i_par] : _mesh)
            {
                if ( i_par >= _values.size() )
                    throw std::runtime_error("[ERROR] Error creating varying parameter, i_par is out of bounds");

                used_parameters[i_par] = true;
            }

            if ( std::count(used_parameters.cbegin(), used_parameters.cend(), false) > 0 )
                throw std::runtime_error("[ERROR] There are unused parameters");
        }

        //! Get the total number of parameters
        size_t n_parameters() const { return _values.size(); }

        //! Evaluate the parameter at a given arclenght
        Timeseries_t operator()(const scalar& s) const
        {
            if ( _values.size() == 1 )            
            {
                return _values.front();
            }
            else
            {
                // (1) Get an iterator to the first point in the hypermesh grid that *it > s
                auto it = std::upper_bound(_mesh.cbegin(), _mesh.cend(), s, [](const auto& lhs, const auto& rhs) -> auto { return lhs < rhs.first; });

                // (2) Compute the value
                if ( it == _mesh.cbegin() )   
                    // (2.1) If s < mesh[0], return the first element
                    return _values.front();
                else if ( it == _mesh.cend() )
                    // (2.2) If s > mesh[end], return the last element 
                    return _values.back();
                else
                {
                    // (2.3) Blend using a linear piecewise interpolation: 
                    //       xi=0 for left checkpoint, xi=1 for right checkpoint
                    const scalar xi      = (s - (it-1)->first)/(it->first - (it-1)->first);
                    return _values[(it-1)->second] + xi*(_values[it->second] - _values[(it-1)->second]);
                }
            }
        }
        
        //! Get the values
        const std::vector<Timeseries_t>& get_values() const { return _values; }
              std::vector<Timeseries_t>& get_values()       { return _values; }

        //! Get the parameter path
        const std::string& get_path() const { return _path; }

     private:
        std::string _path;                           // Path to the parameter. To compute gradients w.r.t. it, make sure it points to a CppAD variable
        std::vector<Timeseries_t> _values;           // Values of the parameter
        std::vector<std::pair<scalar,size_t>> _mesh; // (Optional) mesh for varying parameters: (arclength, parameter_index)
    };

    struct Parameters : public std::vector<Parameter>
    {
        using base_type =  std::vector<Parameter>;
        size_t get_number_of_parameters() const { return std::accumulate(base_type::cbegin(), base_type::cend(), 0, [](const auto& lhs, const auto& rhs) -> auto { return lhs + rhs.get_values().size(); }); }

        std::vector<scalar> get_all_parameters_as_scalar() const
        {
            std::vector<scalar> p(get_number_of_parameters());
    
            auto it_p = p.begin();
            for (const auto& parameter : *this)
            {
                std::transform(parameter.get_values().cbegin(), parameter.get_values().cend(), it_p, [](const auto& p) -> auto { return Value(p); } );
                it_p += parameter.get_values().size();
            }

            assert(it_p == p.cend());

            return p;
        }

        void set_all_parameters(const std::vector<Timeseries_t>& inputs)
        {
            assert(inputs.size() == get_number_of_parameters());

            auto it_p = inputs.cbegin();
            for (auto& parameter : *this)
            {
                std::copy(it_p, it_p+parameter.get_values().size(), parameter.get_values().begin());
                it_p += parameter.get_values().size();
            }

            assert(it_p == inputs.cend());
        }
    };

    Parameters _parameters; //! Parameters that can be used to have spatially varying parameters, and/or to compute sensitivities

 public:

    // Get a constant reference to the parameters
    const Parameters& get_parameters() const { return _parameters; }
};

#include "dynamic_model_car.hpp"

#endif
