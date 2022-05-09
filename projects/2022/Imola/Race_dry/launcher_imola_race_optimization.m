addpath('../../../../src/main/matlab');

global output_folder;

if ~exist('output_folder','var')
    error('Set output_folder');
end

% (1) Load library
if libisloaded('libfastestlapc')
    unloadlibrary libfastestlapc
end

loadlibrary('../../../../build/lib/libfastestlapc.so','../../../../src/main/c/fastestlapc.h');

fprintf('fastestlap loaded\n');

% (2) Construct track
circuit = calllib('libfastestlapc','create_track',[],'imola','../../../../database/tracks/imola/imola_adapted.xml',fastestlap_default_options('create_track'));

% (2.1) Get the arclength
len_s = calllib("libfastestlapc","download_vector_table_variable_size",'track/s');
s = zeros(1,len_s);
s = calllib("libfastestlapc","download_vector_table_variable",s,length(s), 'track/s');

% (3) Preprocess the reference lap

% (3.1) Load
u_telemetry_fcn = @(s)(interp1([0,s_ref],[speed_ref(end),speed_ref],s));

% (3.2) Get telemetry data
telemetry_data = preprocess_telemetry(s_ref,time_ref,speed_ref,250.0,10.0);
telemetry_data = [telemetry_data(1),telemetry_data(3:end)];

% (4) Compute a sample lap with fastestlap
[time_fl,u_fl] = run_fastest_lap(circuit,s,get_x_from_xml('../../../../database/vehicles/f1/ferrari-2022-australia.xml'), false);
fl_data = preprocess_telemetry(s,time_fl,u_fl,250.0,10.0);

% (5) Map the telemetry into the fastest lap arclength
s_telemetry = zeros(1,len_s);
for i = 1 : len_s
   s_telemetry(i) = map_telemetry_to_fastest_lap(s(i), telemetry_data, fl_data, s(end), s_ref);
end

% (6) Prepare run_fastest_lap for the optimization.
run_fastest_lap(u_telemetry_fcn(s_telemetry));


% (6.1) Upper and lower bounds
lb = [1.0  ,0.7, 0.995,2.5,0.40,0.30,-0.2,0.20,0.20 ,1.6,1.6,1.6,1.6,4000.0/(795.0*9.81)];
ub = [3.0  ,0.85,1.3,  3.0,0.50,0.40, 0.2,0.35,0.80,2.0,2.0,2.0,2.0,6000.0/(795.0*9.81)];

% (6.2) Linear constraints: mu2 < mu1, mu_front < mu_rear
A = zeros(4,numel(lb));
b = zeros(4,1);

A(1,10) = -1.0;
A(1,11) = 1.0;
A(2,12) = -1.0;
A(2,13) = 1.0;

A(3,10) = 1.0;
A(3,12) = -1.0;

A(4,11) = 1.0;
A(4,13) = -1.0;

% (6.3) Options
options = optimoptions('fmincon','TolX',1.0e-4,'TolFun',1.0e-4,'Display','iter','SpecifyObjectiveGradient',true);

% (6.4) Initial point
x0 = [2.71, 0.72, 1.01, 2.94, 0.55, 0.30, -0.07, 0.30, 0.38, 1.90, 1.74, 1.81, 1.66, 0.54];

% (6.5) Save workspace prior to the start of the optimization
fprintf('Saving initialization data: %s\n',[output_folder,'/initialization'])
save([output_folder,'/initialization'])

% (7) Optimize
x=fmincon(@(x)(fitness_function(circuit,s,x,laptime_ref)),x0,A,b,[],[],lb,ub,[],options);

%--------------------------------------------------------------------------:-
%
% Helper functions --------------------------------------------------------:-
%
%--------------------------------------------------------------------------:-
function [f,df] = fitness_function(circuit,s,x,laptime)
 persistent f_count;
 global output_folder;
 
 if ( isempty(f_count) )
     f_count = 0;
 end
 
 f_count = f_count + 1;
 
 fprintf('Fitness function evaluation %d: ',f_count);
 fprintf('%.2f, ',x(1:numel(x)-1));
 fprintf('%.2f -> ',x(end));
 
 [~,~,f] = run_fastest_lap(circuit,s,x,false,laptime,[output_folder,'/data/autosave_',num2str(f_count)]);
 fprintf('%.2f\n',f);
 
 fprintf('    Gradient evaluation: ');
 
 dx = 1.0e-4*ones(1,numel(x));
 
 df = zeros(1,numel(x));
 for i = 1 : length(x)
     e_i = zeros(1,numel(x));
     e_i(i) = 1;
     [~,~,f_prime] = run_fastest_lap(circuit,s,x+dx(i)*e_i,true,laptime);
     df(i) = (f_prime-f)/dx(i);
     
     fprintf('%d ',i);
 end
 fprintf('\n');
 
end

function x = get_x_from_xml(filename)
    
    database = readstruct(filename);

    differential_stiffness = log10(database.rear_axle.differential_stiffness);
    power = database.rear_axle.engine.maximum_power/(database.chassis.mass);
    cd    = database.chassis.aerodynamics.cd;
    aero_eff = database.chassis.aerodynamics.cl/cd;
    
    wheelbase = database.chassis.front_axle.x - database.chassis.rear_axle.x;
    x_cog = -database.chassis.rear_axle.x/wheelbase;
    h_cog = -database.chassis.com.z;
    
    x_press = database.chassis.pressure_center.x/wheelbase;
    z_press = -database.chassis.pressure_center.z;
    
    roll_balance = database.chassis.roll_balance_coefficient;
    
    mu_y_front_1 = database.front_tire.mu_y_max_1;
    mu_y_front_2 = database.front_tire.mu_y_max_2;
    mu_y_rear_1 = database.rear_tire.mu_y_max_1;
    mu_y_rear_2 = database.rear_tire.mu_y_max_2;
    
    max_torque = database.rear_axle.brakes.max_torque/(database.chassis.mass*9.81);
    
    x = fold_x(differential_stiffness,power,cd,aero_eff,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque);
end

function [time,u,error] = run_fastest_lap(circuit,s,x, warm_start, laptime, filename)

persistent u_reference;

% A call just to update the reference speed. 
% Quite a dirty trick but I like it
if ( nargin == 1 )
    u_reference = circuit;
    return
end

[differential_stiffness,power,cd,aero_eff,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque] = unfold_x(x);

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
calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/brake_bias', 0.6);
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
options = [options,    '        <time/>'];
options = [options,    '        <laptime/>'];
options = [options,    '    </variables>'];
options = [options,    '</save_variables>'];
options = [options,    '<control_variables>'];
options = [options,    '    <brake-bias type="hypermesh">'];
options = [options,    '        <hypermesh> 0.0, 990.0, 1526.0, 1925.0, 2589.0, 3024.0, 3554.0 </hypermesh>'];
options = [options,    '    </brake-bias>'];
options = [options,    '</control_variables>'];
options = [options,'</options>'];

if ( nargin >= 6)
    calllib("libfastestlapc","save_vehicle_as_xml",vehicle,[filename,'.xml']);
end

% (3) Run optimal laptime
calllib("libfastestlapc","optimal_laptime",vehicle,circuit,length(s),s,options);

% (4) Get the velocity and time
u = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)),length(s), 'run/u')*3.6;
time = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)),length(s), 'run/time');

if ( nargout >= 3 )
    if (isempty(u_reference))
        error('Error cannot be computed because u_reference is empty');
    end
    error = norm(u-u_reference);
    
    % Add the laptime target
    error = error + 500.0*(calllib("libfastestlapc","download_scalar_table_variable",'run/laptime') - laptime)^2;
end


if ( nargin >= 6)
    save(filename);
end


% (7) Clean up
calllib("libfastestlapc","delete_vehicle", vehicle);
calllib("libfastestlapc","clear_tables_by_prefix", 'run/');

end

function [differential_stiffness,power,cd,aero_eff,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque] = unfold_x(x)
    differential_stiffness = 10^(x(1));
    power = x(2);
    cd = x(3);
    aero_eff = x(4); 
    x_cog = x(5); 
    h_cog = x(6);
    x_press = x(7); 
    z_press = x(8); 
    roll_balance = x(9);
    mu_y_front_1 = x(10); 
    mu_y_front_2 = x(11);
    mu_y_rear_1 = x(12);
    mu_y_rear_2 = x(13); 
    max_torque = x(14);
end

function x = fold_x(differential_stiffness,power,cd,aero_eff,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque)
 x = [differential_stiffness,power,cd,aero_eff,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque];
end

function s_telemetry = map_telemetry_to_fastest_lap(s_fl, telemetry_data, fl_data, s_max_fl, s_telemetry_full)

    if ( (s_fl < 0.0) || (s_fl > s_max_fl) )
        error('map_telemetry_to_fastest_lap -> s_fl is out of bounds');
    end
    
    % (1) Find the interval where the point is found in fastest_lap
    s_corner_fl = [fl_data(end).s_corner-s_max_fl, [fl_data.s_corner], fl_data(1).s_corner + s_max_fl];
    
    i_after_s_fl = find(s_corner_fl >= s_fl);
    
    i_start = i_after_s_fl(1)-1;
    i_end   = i_after_s_fl(1);
    
    % (2) Get xi 
    xi = (s_fl - s_corner_fl(i_start))/(s_corner_fl(i_end) - s_corner_fl(i_start));
    
    % (3) Get the arclength in the telemetry
    s_corner_telemetry = [telemetry_data(end).s_corner-s_telemetry_full(end), [telemetry_data.s_corner], telemetry_data(1).s_corner + s_telemetry_full(end)];

    s_telemetry = s_corner_telemetry(i_start) + xi*(s_corner_telemetry(i_end) - s_corner_telemetry(i_start));
    
    % (4) Put the output back to [0,s_max_telemetry]
    if ( s_telemetry < s_telemetry_full(1) )
        s_telemetry = s_telemetry + s_telemetry_full(end);
    elseif ( s_telemetry > s_telemetry_full(end) )
        s_telemetry = s_telemetry - s_telemetry_full(end);
    end
   
end
