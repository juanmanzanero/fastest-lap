#ifndef DYNAMIC_MODEL_H
#define DYNAMIC_MODEL_H

#include "src/core/foundation/fastest_lap_exception.h"
#include "src/core/foundation/model_parameter.h"

template<typename Timeseries_t>
class Dynamic_model
{
 public:
    using T = Timeseries_t;

    //! Handling of parameters ------------------------------------------:-
    template<typename ... Args> 
    void add_parameter(const std::string& parameter_path, Args&& ... args)
    {
        // Check that the parameter was not already loaded
        for (const auto& p : _parameters)
        {
            if ( p.get_path() == parameter_path )
                throw fastest_lap_exception("[ERROR] Dynamic_model::add_parameter -> parameter already exists");
        }

        _parameters.emplace_back(parameter_path, std::forward<Args>(args)...);
    }

    // Set all the parameters from a hyper-vector
    void set_all_parameters(const std::vector<Timeseries_t>& inputs) { _parameters.set_all_parameters(inputs); }

    // Get a constant reference to the parameters
    const Model_parameters<T>& get_parameters() const { return _parameters; }

 private:

    Model_parameters<T> _parameters; //! Model_parameters<T> that can be used to have spatially varying parameters, and/or to compute sensitivities

 protected:
    Model_parameters<T>& get_parameters() { return _parameters; }

    
};

#endif
