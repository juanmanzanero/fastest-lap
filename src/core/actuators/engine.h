#ifndef __ENGINE_H__
#define __ENGINE_H__

#include<map>
#include "lion/math/polynomial.h"
#include "lion/io/Xml_document.h"
#include "lion/io/database_parameters.h"
#include "src/core/foundation/fastest_lap_exception.h"

template<typename Timeseries_t>
class Engine
{
 public:
    using Timeseries_type = Timeseries_t;
    //! A default constructor if no engine is needed
    Engine() = default;

    //! Proper constructor from parameters + engine path
    Engine(Xml_document& database, const std::string& path, const bool only_max_power);

    //! Default constructor from engine path
    Engine(const std::string& path, const bool only_max_power) 
        : _path(path), _gear_ratio(1.0), _direct_torque(false), _only_max_power(only_max_power), _maximum_power(0.0) {}

    // Set parameter    
    template<typename T>
    void set_parameter(const std::string& parameter, const T value)
    {
        // Find the parameter in the database
        const auto found = ::set_parameter(get_parameters(), parameter, _path, value); 

        // If not found, throw an exception
        if ( !found )
            throw fastest_lap_exception("Parameter \"" + parameter + "\" was not found");
    }

    void fill_xml(Xml_document& doc) const 
    {
        ::write_parameters(doc, _path, get_parameters());
    }

    constexpr const sPolynomial& get_polynomial() const { return _p; }

    Timeseries_t operator()(const Timeseries_t throttle_percentage, const Timeseries_t rpm);

    constexpr const bool& direct_torque() const { return _direct_torque; }
    constexpr       bool& direct_torque()       { return _direct_torque; }

    constexpr const scalar& gear_ratio() const { return _gear_ratio; }

    const Timeseries_t& get_power() const { return _power; }

 private:
    std::string _path;

    sPolynomial _p;

    scalar _gear_ratio;
    bool _direct_torque = true;
    bool _only_max_power;

    Timeseries_t _maximum_power;      

    // Variables
    Timeseries_t _power;

    DECLARE_PARAMS({ "maximum-power", _maximum_power }); 
};

#include "engine.hpp"

#endif
