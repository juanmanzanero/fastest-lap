#ifndef __ROAD_H__
#define __ROAD_H__

#include "lion/foundation/types.h"

template<typename Timeseries_t, size_t STATE0, size_t CONTROL0>
class Road
{
 public:
    using Timeseries_type = Timeseries_t; 

    struct input_state_names
    {
        enum
        {
            end = STATE0
        };
    };

    struct control_names
    {
        enum
        {
            end = CONTROL0
        };
    };

    constexpr const Timeseries_t& get_x() const { return _x; }

    constexpr const Timeseries_t& get_y() const { return _y; }

    constexpr const Timeseries_t& get_psi() const { return _psi; }

    constexpr const Timeseries_t& get_dtimedt() const { return _dtimedt; } 

 protected:

    Timeseries_t _dtimedt = 1.0;

    // The road contains the 3 dof of the movement
    Timeseries_t _x;
    Timeseries_t _y;
    Timeseries_t _psi;
};

#endif
