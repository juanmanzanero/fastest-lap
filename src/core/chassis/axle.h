#ifndef AXLE_H
#define AXLE_H

#include <unordered_map>
#include "lion/frame/frame.h"
#include "lion/io/Xml_document.h"
#include "lion/io/database_parameters.h"

//!         A generic axle class
//!         --------------------
//! The generic axle class is responsible of the storage and interface with the tire objects
//!  @param Tires_tuple: must be a tuple with the types of the tires 
//!                      (1 for motorcycles/2 for cars)
//!  @param state_start: index of the first state variable defined here
//!  @param control_start: index of the first control variable defined here
template <typename Timeseries_t,typename Tires_tuple, size_t state_start, size_t control_start>
class Axle
{
 public:
    using Timeseries_type = Timeseries_t;

    //! Number of tires
    constexpr static size_t NTIRES = std::tuple_size<Tires_tuple>::value;

    //! Indices of the state variables of this class: none
    struct input_names
    {
        enum { end = state_start };
    };

    //! Indices of the control variables of this class: none
    struct control_names
    {
        enum { end = control_start };
    };

    struct state_names
    {
        enum { end = state_start };
    };

    static_assert(static_cast<size_t>(input_names::end) == static_cast<size_t>(state_names::end));

    //! Default constructor
    Axle() = default;

    //! Constructor
    //! 
    //! @param[in] name: name of the axle
    //! @param[in] tires: tuple containing the tires
    Axle(const std::string& name, const Tires_tuple& tires);

    //! Copy assignment operator
    //! @param[in] other: axle to copy
    Axle& operator=(const Axle& other);

    //! Copy constructor operator

    //! @param[in] other: axle to copy
    Axle(const Axle& other);

    template<typename T>
    bool set_parameter(const std::string& parameter, const T value);

    //! Get a reference to the axle frame
    Frame<Timeseries_t>& get_frame() { return _frame; }

    //! Get a const reference to the frame
    const Frame<Timeseries_t>& get_frame() const { return _frame; }

    //! Get a const reference to one of the tires
    //! @param I: position of the tire in the tuple
    template<size_t I>
    const typename std::tuple_element<I,Tires_tuple>::type& get_tire() const 
        { return std::get<I>(_tires); }

    //! Get the force acting on the axle center [N]
    const Vector3d<Timeseries_t>& get_force() const { return _F; }

    //! Get the torque acting on the axle center [Nm]
    const Vector3d<Timeseries_t>& get_torque() const { return _T; } 

    //! Load the time derivative of the state variables computed herein to the dqdt
    //! @param[out] dqdt: the vehicle state vector time derivative
    template<size_t number_of_states>
    void get_state_and_state_derivative(std::array<Timeseries_t,number_of_states>& state, 
                                        std::array<Timeseries_t,number_of_states>& dstate_dt
                                        ) const;

    //! Set the state variables of this class
    //! @param[in] q: the vehicle state vector 
    //! @param[in] u: the vehicle control vector
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_controls(const std::array<Timeseries_t,number_of_inputs>& inputs, 
                                const std::array<Timeseries_t,number_of_controls>& controls);

    //! Set the state and controls upper, lower, and default values
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_upper_lower_and_default_values(std::array<scalar,number_of_inputs>& inputs_def,
                                                               std::array<scalar,number_of_inputs>& inputs_lb,
                                                               std::array<scalar,number_of_inputs>& inputs_ub,
                                                               std::array<scalar,number_of_controls>& controls_def,
                                                               std::array<scalar,number_of_controls>& controls_lb,
                                                               std::array<scalar,number_of_controls>& controls_ub 
                                                              ) const;

    //! Get the names of the state and control varaibles of this class
    //! @param[out] q: the vehicle state names
    //! @param[out] u: the vehicle control names
    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_names(std::array<std::string,number_of_inputs>& inputs, 
                                     std::array<std::string,number_of_controls>& controls) const;

    bool is_ready() const;

    static std::string type() { return "axle"; }
    
    void fill_xml(Xml_document& doc) const;

    std::unordered_map<std::string,Timeseries_t> get_outputs_map() const
    {
        auto map = get_outputs_map_self();

        auto tire_0_map = std::get<0>(_tires).get_outputs_map();
        map.insert(tire_0_map.cbegin(), tire_0_map.cend());

        if constexpr (NTIRES == 2)
        {
            auto tire_1_map = std::get<1>(_tires).get_outputs_map();
            map.insert(tire_1_map.cbegin(), tire_1_map.cend());
        }

        return map;
    }

 private:

    Frame<Timeseries_t> _frame; //! Frame<Timeseries_t> on the axle center

    DECLARE_PARAMS();

    std::unordered_map<std::string,Timeseries_t> get_outputs_map_self() const
    {
        return
        {
        };
    }

 protected:
    std::string _name;    //! Axle name (e.g. front, rear)
    std::string _path;    //! Axle path in the database file
    Tires_tuple _tires;   //! Tuple containing the tires

    Vector3d<Timeseries_t> _F;   //! [out] Forces generated by the tires
    Vector3d<Timeseries_t> _T;   //! [out] Torque generated by the tires
};

#include "axle.hpp"

#endif
