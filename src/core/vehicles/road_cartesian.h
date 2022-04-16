#ifndef __ROAD_CARTESIAN_H__
#define __ROAD_CARTESIAN_H__

#include "road.h"

template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
class Road_cartesian : public Road<Timeseries_t,STATE0,CONTROL0>
{
 public:
    using base_type = Road<Timeseries_t,STATE0,CONTROL0>;

    enum State { IX = base_type::STATE_END, IY, IPSI, STATE_END };
    enum Controls { CONTROL_END = base_type::CONTROL_END };

    constexpr static size_t IIDX   = IX;
    constexpr static size_t IIDY   = IY;
    constexpr static size_t IIDPSI = IPSI;

    void update(const Timeseries_t u, const Timeseries_t v, const Timeseries_t omega);

    template<size_t N>
    void get_state_derivative(std::array<Timeseries_t,N>& dqdt) const;

    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_controls(const Timeseries_t t, const std::array<Timeseries_t,NSTATE>& q, const std::array<Timeseries_t,NCONTROL>& u);


    //! Set the state and controls upper, lower, and default values
    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_control_upper_lower_and_default_values(std::array<scalar,NSTATE>& q_def,
                                                               std::array<scalar,NSTATE>& q_lb,
                                                               std::array<scalar,NSTATE>& q_ub,
                                                               std::array<scalar,NCONTROL>& u_def,
                                                               std::array<scalar,NCONTROL>& u_lb,
                                                               std::array<scalar,NCONTROL>& u_ub 
                                                              ) const;


    template<size_t NSTATE, size_t NCONTROL>
    static void set_state_and_control_names(std::string& key_name, std::array<std::string,NSTATE>& q, std::array<std::string,NCONTROL>& u);

 private:
    Timeseries_t _dx;
    Timeseries_t _dy;
    Timeseries_t _dpsi;
};

#include "road_cartesian.hpp"

#endif
