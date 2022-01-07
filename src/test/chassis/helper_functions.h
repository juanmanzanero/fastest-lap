// Implement the tire model
inline sVector3d get_tire_forces(const scalar kappa, const scalar lambda, const scalar Fz)
{
    const scalar kappa_max = (Fz - 2000.0)*(0.10-0.11)/(6000.0-2000.0) + 0.11;
    const scalar lambda_max = (Fz - 2000.0)*(8.0*DEG - 9.0*DEG)/(6000.0-2000.0) + 9.0*DEG;
    const scalar mu_x_max  = (Fz - 2000.0)*(1.40-1.75)/(6000.0-2000.0) + 1.75;
    const scalar mu_y_max  = (Fz - 2000.0)*(1.45-1.80)/(6000.0-2000.0) + 1.80;

    const scalar kappa_n = kappa/kappa_max;
    const scalar lambda_n = lambda/lambda_max;

    const scalar rho = sqrt(kappa_n*kappa_n + lambda_n*lambda_n + 1.0e-12);

    const scalar mu_x = mu_x_max*sin(1.9*atan(0.5*pi*rho/atan(1.9)));
    const scalar mu_y = mu_y_max*sin(1.9*atan(0.5*pi*rho/atan(1.9)));

    const scalar Fx_computed = mu_x*Fz*kappa_n/rho;
    const scalar Fy_computed = mu_y*Fz*lambda_n/rho;

    return {Fx_computed, Fy_computed, -Fz};
}
