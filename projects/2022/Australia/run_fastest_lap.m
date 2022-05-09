function [error] = run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,aero_eff,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, warm_start)

differential_stiffness = 10^(differential_stiffness);
mu_x_front_1 = mu_y_front_1;
mu_x_front_2 = mu_y_front_2;
mu_x_rear_1 = mu_y_rear_1;
mu_x_rear_2 = mu_y_rear_2;
cl = aero_eff*cd;

% (1) Construct car
wheelbase = 3.600; % [m]
track = 1.52; % [m]
mass = 795.0;

power = power*mass;
max_torque = max_torque*mass*9.81;

vehicle = 'vehicle';
calllib("libfastestlapc","create_vehicle",vehicle,'limebeer-2014-f1','');

calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-axle/track', track);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-axle/inertia', 1.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-axle/smooth_throttle_coeff', 1.0e-5);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-axle/brakes/max_torque', max_torque);

calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-axle/track', track);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-axle/inertia', 1.55);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-axle/smooth_throttle_coeff', 1.0e-5);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-axle/differential_stiffness', differential_stiffness);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-axle/brakes/max_torque', max_torque);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-axle/engine/maximum-power', power);

calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/mass', mass);
calllib("libfastestlapc","set_matrix_parameter",vehicle,'vehicle/chassis/inertia', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 500.0]);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/aerodynamics/rho', 1.2);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/aerodynamics/area', 1.5);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/aerodynamics/cd', cd);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/aerodynamics/cl', cl);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/com/x', 0.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/com/y', 0.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/com/z', -h_cog);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/front_axle/x', wheelbase*(1.0-x_cog));
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/front_axle/y', 0.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/front_axle/z', -0.36);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/rear_axle/x', -wheelbase*x_cog);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/rear_axle/y', 0.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/rear_axle/z', -0.36);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/pressure_center/x', x_press*wheelbase);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/pressure_center/y', 0.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/pressure_center/z', -z_press);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/brake_bias', brake_bias);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/roll_balance_coefficient', roll_balance);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/Fz_max_ref2', 1.0);

calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/radius',0.360);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/radial-stiffness',0.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/radial-damping',0.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/Fz-max-ref2', 1.0 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/reference-load-1', 2000.0 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/reference-load-2', 8000.0 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/mu-x-max-1', mu_x_front_1 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/mu-x-max-2', mu_x_front_2 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/kappa-max-1', 0.11 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/kappa-max-2', 0.11 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/mu-y-max-1', mu_y_front_1 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/mu-y-max-2', mu_y_front_2 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/lambda-max-1', 9.0 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/lambda-max-2', 9.0 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/Qx', 1.9 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/front-tire/Qy', 1.9 );

calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/radius',0.360);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/radial-stiffness',0.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/radial-damping',0.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/Fz-max-ref2', 1.0 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/reference-load-1', 2000.0 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/reference-load-2', 8000.0 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/mu-x-max-1', mu_x_rear_1 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/mu-x-max-2', mu_x_rear_2 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/kappa-max-1', 0.11 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/kappa-max-2', 0.11 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/mu-y-max-1', mu_y_rear_1 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/mu-y-max-2', mu_y_rear_2 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/lambda-max-1', 9.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/lambda-max-2', 9.0);
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/Qx', 1.9 );
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-tire/Qy', 1.9 );

% (2) Set options
options = '<options>';
if ( warm_start )
    options = [options,'<warm_start> true </warm_start>'];
else
    options = [options,'<save_warm_start> true </save_warm_start>'];
end
options = [options,    '<save_variables>'];
options = [options,    '    <prefix>run/</prefix>'];
options = [options,    '    <variables>'];
options = [options,    '        <u/>'];
options = [options,    '    </variables>'];
options = [options,    '</save_variables>'];
options = [options,    '<sigma> 0.5 </sigma>'];
options = [options,'</options>'];


% (3) Run optimal laptime
calllib("libfastestlapc","optimal_laptime",vehicle,circuit,length(s),s,options);

% (4) Get the velocity
u = zeros(1,length(s));
u = calllib("libfastestlapc","download_vector_table_variable",u,length(s), 'run/u');

% (5) Compute the error w.r.t. Leclerc's lap:
u = 3.6*u;
error = norm(u(1:397) - u_reference(1:397));

% (6) Clean up
calllib("libfastestlapc","delete_vehicle", vehicle);
calllib("libfastestlapc","clear_tables_by_prefix", 'run/');

end
