#ifndef OPTIMAL_LAPTIME_CONSTANTS_H
#define OPTIMAL_LAPTIME_CONSTANTS_H

//! Types of optimization for the control variables:
//!     (1) Do not optimize: keep constant from its value given initially
//!     (2) Constant: optimize using a fixed value along the mesh
//!     (3) Hypermesh: optimize the control variable using an alternative mesh
//!     (4) Full mesh: optimize the control variable along the full mesh
struct Optimal_control_type
{
    enum Type { DONT_OPTIMIZE, CONSTANT, HYPERMESH, FULL_MESH };
};

#endif
