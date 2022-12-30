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
    std::vector<scalar> theta;
    std::vector<scalar> mu;
    std::vector<scalar> phi;
    std::vector<scalar> kappa;
    std::vector<scalar> mu_dot;
    std::vector<scalar> phi_dot;
    std::vector<scalar> nl;
    std::vector<scalar> nr;
    std::vector<scalar> dkappa;
    std::vector<scalar> dmu_dot;
    std::vector<scalar> dphi_dot;
    std::vector<scalar> dnl;
    std::vector<scalar> dnr;

    scalar track_length;
    scalar left_boundary_max_error;
    scalar right_boundary_max_error;
    scalar left_boundary_L2_error;
    scalar right_boundary_L2_error;
};

#endif
