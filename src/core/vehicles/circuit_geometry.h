#ifndef CIRCUIT_GEOMETRY_H
#define CIRCUIT_GEOMETRY_H

struct Circuit_geometry
{
    size_t      n_elements;
    size_t      n_points;
    bool        is_closed;
    int         direction;      

    std::vector<sVector3d> r_left;
    std::vector<sVector3d> r_left_measured;     
    std::vector<sVector3d> r_right;
    std::vector<sVector3d> r_right_measured;    
    std::vector<sVector3d> r_centerline;

    std::vector<scalar> s;
    std::vector<scalar> yaw;
    std::vector<scalar> pitch;
    std::vector<scalar> roll;
    std::vector<scalar> yaw_dot;
    std::vector<scalar> pitch_dot;
    std::vector<scalar> roll_dot;
    std::vector<scalar> nl;
    std::vector<scalar> nr;
    std::vector<scalar> dyaw_dot;
    std::vector<scalar> dpitch_dot;
    std::vector<scalar> droll_dot;
    std::vector<scalar> dnl;
    std::vector<scalar> dnr;

    scalar track_length;
    scalar left_boundary_max_error;
    scalar right_boundary_max_error;
    scalar left_boundary_L2_error;
    scalar right_boundary_L2_error;
};

#endif
