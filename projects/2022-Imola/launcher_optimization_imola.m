clear all
%clc
%close all
format long

% (1) Load library
if libisloaded('libfastestlapc')
    unloadlibrary libfastestlapc
end

loadlibrary('../../build/lib/libfastestlapc.dylib','../../src/main/c/fastestlapc.h');

% (2) Construct track
options = '<options> <save_variables> <prefix>track/</prefix> <variables> <s/> </variables> </save_variables> </options>';
circuit = calllib('libfastestlapc','create_track',[],'imola','../../database/tracks/imola/imola_adapted.xml',options);


len_s = calllib("libfastestlapc","download_vector_table_variable_size",'track/s');
s = zeros(1,len_s);
s = calllib("libfastestlapc","download_vector_table_variable",s,length(s), 'track/s');

imola_reference_lap;

reference_lap_stats = preprocess_telemetry(s_ref, time_ref, speed_ref, 250.0,10.0);

f = @(x)(fitness_function_imola(circuit,reference_lap_stats,s,x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),x(9),x(10),x(11),x(12),x(13),x(14)));

lb = [1.0  ,0.7,0.995,2.5,0.45,0.30,-0.2,0.20,0.20 ,1.0,1.0,1.0,1.0,4000.0/(795.0*9.81)];
ub = [3.0  ,0.85,1.3,3.0,0.55,0.40, 0.2,0.35,0.80,1.8,1.8,1.8,1.8,6000.0/(795.0*9.81)];

x0 = 0.5*(lb+ub);

options = optimoptions('fmincon','TolX',1.0e-4,'TolFun',1.0e-4,'Display','iter','SpecifyObjectiveGradient',true);

x=fmincon(f,x0,[],[],[],[],lb,ub,[],options);


function [f,df] = fitness_function_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2,max_torque)

print_variables(differential_stiffness,power*795.0,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2,max_torque*795.0*9.81);

N = 14;

% Compute the fitness function
f = run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque, false);

fprintf("Fitness function computed: ");
if nargout > 1
    % Compute the gradient
    df = zeros(1,N);
    
    dx = 1.0e-5*10^(differential_stiffness);
    df(1) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness+dx,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("1 ");
    
    dx = 1.0e-3;
    df(2) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power+dx,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("2 ");
    dx = 1.0e-3;
    df(3) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd+dx,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("3 ");
    dx = 1.0e-3;
    df(4) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl+dx,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("4 ");
    dx = 1.0e-4;
    df(5) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog+dx,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("5 ");
    dx = 1.0e-4;
    df(6) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog,h_cog+dx,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("6 ");
    dx = 1.0e-3;
    df(7) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press+dx,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("7 ");
    dx = 1.0e-3;
    df(8) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press+dx, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("8 ");
    dx = 1.0e-3;
    df(9) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance+dx, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("9 ");
    
    dx = 1.0e-3;
    df(10) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1+dx, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("10 ");
    dx = 1.0e-3;
    df(11) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2+dx, mu_y_rear_1, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("11 ");
    dx = 1.0e-3;
    df(12) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1+dx, mu_y_rear_2, max_torque, true)-f)/dx;
    fprintf("12 ");
    dx = 1.0e-3;
    df(13) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2+dx, max_torque, true)-f)/dx;
    fprintf("13 ");
    dx = 1.0e-4;
    df(14) = (run_fastest_lap_imola(circuit,reference_lap_stats,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque+dx, true)-f)/dx;
    fprintf("14");
end
fprintf("\n");
end

function print_variables(varargin)
for i = 1:nargin
    fprintf("    scalar %s = %24.16f;\n",inputname(i),varargin{i})
end
end
