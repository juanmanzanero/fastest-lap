steps_1_to_3 = false;

if ( steps_1_to_3 )
    
    clear all
    clc
    close all
    format long
    
    % (1) Load library
    if libisloaded('libfastestlapc')
        unloadlibrary libfastestlapc
    end
    
    loadlibrary('/Users/juanmanzanero/Documents/software/fastest-lap/build/lib/libfastestlapc.dylib','/Users/juanmanzanero/Documents/software/fastest-lap/src/main/c/fastestlapc.h');
    
    % (2) Construct track
    options = '<options> <save_variables> <prefix>track/</prefix> <variables> <s/> </variables> </save_variables> </options>';
    circuit = calllib('libfastestlapc','create_track',[],'catalunya','../../database/tracks/melbourne/melbourne_700.xml',options);
    
    N = calllib("libfastestlapc","download_vector_table_variable_size",'track/s');
    s = zeros(1,N);
    s = calllib("libfastestlapc","download_vector_table_variable",s,length(s), 'track/s');
    track_length = 5264.36268833544;
    
    % (2.1) Get track coordinates
    [x_center,y_center,x_left,y_left,x_right,y_right,theta] = calllib('libfastestlapc','track_coordinates',zeros(1,N+1),zeros(1,N+1),zeros(1,N+1),zeros(1,N+1),zeros(1,N+1),zeros(1,N+1),zeros(1,N+1),circuit,N+1);
    x_center = x_center(1:N);
    y_center = y_center(1:N);
    x_left = x_left(1:N);
    y_left = y_left(1:N);
    x_right = x_right(1:N);
    y_right = y_right(1:N);
    
    % (3) Construct car
    vehicle = calllib("libfastestlapc","create_vehicle",[],'vehicle','limebeer-2014-f1','/Users/juanmanzanero/Documents/software/fastest-lap/database/vehicles/f1/ferrari-2022-australia.xml');
    vehicle_data = readstruct('/Users/juanmanzanero/Documents/software/fastest-lap/database/vehicles/f1/ferrari-2022-australia.xml');
    % (3) Simulation
    options = '<options>';
    options = [options,    '<save_variables>'];
    options = [options,    '    <prefix>run/</prefix>'];
    options = [options,    '    <variables>'];
    options = [options,    '        <laptime/>'];
    options = [options,    '        <x/>'];
    options = [options,    '        <y/>'];
    options = [options,    '        <front_axle.left_tire.kappa/>'];
    options = [options,    '        <front_axle.right_tire.kappa/>'];
    options = [options,    '        <rear_axle.left_tire.kappa/>'];
    options = [options,    '        <rear_axle.right_tire.kappa/>'];
    options = [options,    '        <front_axle.left_tire.dissipation/>'];
    options = [options,    '        <front_axle.right_tire.dissipation/>'];
    options = [options,    '        <rear_axle.left_tire.dissipation/>'];
    options = [options,    '        <rear_axle.right_tire.dissipation/>'];
    options = [options,    '        <u/>'];
    options = [options,    '        <v/>'];
    options = [options,    '        <omega/>'];
    options = [options,    '        <time/>'];
    options = [options,    '        <n/>'];
    options = [options,    '        <alpha/>'];
    options = [options,    '        <delta/>'];
    options = [options,    '        <throttle/>'];
    options = [options,    '        <brake-bias/>'];
    options = [options,    '        <Fz_fl/>'];
    options = [options,    '        <Fz_fr/>'];
    options = [options,    '        <Fz_rl/>'];
    options = [options,    '        <Fz_rr/>'];
    options = [options,    '    </variables>'];
    options = [options,    '</save_variables>'];
    options = [options,'</options>'];
    calllib("libfastestlapc","optimal_laptime",vehicle,circuit,length(s),s,options);
    
    % (3.1) Download the variables
    run.laptime = calllib("libfastestlapc","download_scalar_table_variable", 'run/laptime');
    run.x=zeros(1,length(s));        run.x        = calllib("libfastestlapc","download_vector_table_variable",run.x, length(s), 'run/x');
    run.y=zeros(1,length(s));        run.y        = calllib("libfastestlapc","download_vector_table_variable",run.y, length(s), 'run/y');
    run.kappa_fl=zeros(1,length(s)); run.kappa_fl = calllib("libfastestlapc","download_vector_table_variable",run.kappa_fl, length(s), 'run/front_axle.left_tire.kappa');
    run.kappa_fr=zeros(1,length(s)); run.kappa_fr = calllib("libfastestlapc","download_vector_table_variable",run.kappa_fr, length(s), 'run/front_axle.right_tire.kappa');
    run.kappa_rl=zeros(1,length(s)); run.kappa_rl = calllib("libfastestlapc","download_vector_table_variable",run.kappa_rl, length(s), 'run/rear_axle.left_tire.kappa');
    run.kappa_rr=zeros(1,length(s)); run.kappa_rr = calllib("libfastestlapc","download_vector_table_variable",run.kappa_rr, length(s), 'run/rear_axle.right_tire.kappa');
    run.u=zeros(1,length(s));        run.u        = calllib("libfastestlapc","download_vector_table_variable",run.u, length(s), 'run/u');
    run.v=zeros(1,length(s));        run.v        = calllib("libfastestlapc","download_vector_table_variable",run.v, length(s), 'run/v');
    run.omega=zeros(1,length(s));    run.omega    = calllib("libfastestlapc","download_vector_table_variable",run.omega, length(s), 'run/omega');
    run.time=zeros(1,length(s));     run.time     = calllib("libfastestlapc","download_vector_table_variable",run.time, length(s), 'run/time');
    run.n=zeros(1,length(s));        run.n        = calllib("libfastestlapc","download_vector_table_variable",run.n, length(s), 'run/n');
    run.alpha=zeros(1,length(s));    run.alpha    = calllib("libfastestlapc","download_vector_table_variable",run.alpha, length(s), 'run/alpha');
    run.delta=zeros(1,length(s));    run.delta    = calllib("libfastestlapc","download_vector_table_variable",run.delta, length(s), 'run/delta');
    run.throttle=zeros(1,length(s)); run.throttle = calllib("libfastestlapc","download_vector_table_variable",run.throttle, length(s), 'run/throttle');
    run.brake_bias=zeros(1,length(s)); run.brake_bias = calllib("libfastestlapc","download_vector_table_variable",run.brake_bias, length(s), 'run/brake-bias');
    run.Fz_fl=zeros(1,length(s));    run.Fz_fl    = calllib("libfastestlapc","download_vector_table_variable",run.Fz_fl, length(s), 'run/Fz_fl');
    run.Fz_fr=zeros(1,length(s));    run.Fz_fr    = calllib("libfastestlapc","download_vector_table_variable",run.Fz_fr, length(s), 'run/Fz_fr');
    run.Fz_rl=zeros(1,length(s));    run.Fz_rl    = calllib("libfastestlapc","download_vector_table_variable",run.Fz_rl, length(s), 'run/Fz_rl');
    run.Fz_rr=zeros(1,length(s));    run.Fz_rr    = calllib("libfastestlapc","download_vector_table_variable",run.Fz_rr, length(s), 'run/Fz_rr');
    
    dissipation_fl = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/front_axle.left_tire.dissipation');
    dissipation_fr = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/front_axle.right_tire.dissipation');
    dissipation_rl = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/rear_axle.left_tire.dissipation');
    dissipation_rr = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/rear_axle.right_tire.dissipation');
    
    energy_fl = zeros(1,length(s));
    energy_fr = zeros(1,length(s));
    energy_rl = zeros(1,length(s));
    energy_rr = zeros(1,length(s));
    for i = 2 : length(s)
        energy_fl(i) = energy_fl(i-1) + 0.5*(run.time(i)-run.time(i-1))*(dissipation_fl(i-1) + dissipation_fl(i));
        energy_fr(i) = energy_fr(i-1) + 0.5*(run.time(i)-run.time(i-1))*(dissipation_fr(i-1) + dissipation_fr(i));
        energy_rl(i) = energy_rl(i-1) + 0.5*(run.time(i)-run.time(i-1))*(dissipation_rl(i-1) + dissipation_rl(i));
        energy_rr(i) = energy_rr(i-1) + 0.5*(run.time(i)-run.time(i-1))*(dissipation_rr(i-1) + dissipation_rr(i));
    end
    
    % (3.2) Put the variables into the states and controls vectors
    q = [run.kappa_fl; run.kappa_fr;  run.kappa_rl; run.kappa_rr;...
        run.u; run.v; run.omega; run.time; run.n; run.alpha];
    qa = [run.Fz_fl; run.Fz_fr; run.Fz_rl; run.Fz_rr];
    u  = [run.delta; run.throttle; run.brake_bias];
    energy = [energy_fl; energy_fr; energy_rl; energy_rr];
    
    r_center = [x_center ; y_center];
    
    s_track = [s,track_length];
    x_center = [x_center, x_center(1)];
    y_center = [y_center, y_center(1)];
    x_left = [x_left, x_left(1)];
    y_left = [y_left, y_left(1)];
    x_right = [x_right, x_right(1)];
    y_right = [y_right, y_right(1)];
    
    
    % (3.2.1) Make periodic: add part of another lap
    n_extra = 100;
    n_extra_prev = 100;
    q = [q(:,N-n_extra_prev+1:end), q, q(:,1:n_extra)];
    x = [run.x(:,N-n_extra_prev+1:end), run.x, run.x(:,1:n_extra)];
    y = [run.y(:,N-n_extra_prev+1:end), run.y, run.y(:,1:n_extra)];
    qa = [qa(:,N-n_extra_prev+1:end), qa, qa(:,1:n_extra)];
    u = [u(:,N-n_extra_prev+1:end), u, u(:,1:n_extra)];
    energy = [energy(:,N-n_extra_prev+1:end), energy, energy(:,1:n_extra)];
    r_center = [r_center(:,N-n_extra_prev+1:end), r_center, r_center(:,1:n_extra)];
    s = [s(N-n_extra_prev+1:end)-track_length, s, s(1:n_extra)+track_length];
    q(8,n_extra_prev+1+N:end) = q(8,n_extra_prev+1+N:end)+run.laptime;
    q(8,1:n_extra_prev) = q(8,1:n_extra_prev)-run.laptime;
    
    
    
    
    % (4) Interpolation: we want 30FPS, so the time grid will be with dt = 1/30
    
    % (4.1) Interpolate the track to 4000 points
    n_track = 4000;
    x_center_plot = interp1(s_track,x_center,linspace(s_track(1),s_track(end),n_track),'spline');
    y_center_plot = interp1(s_track,y_center,linspace(s_track(1),s_track(end),n_track),'spline');
    x_left_plot = interp1(s_track,x_left,linspace(s_track(1),s_track(end),n_track),'spline');
    y_left_plot = interp1(s_track,y_left,linspace(s_track(1),s_track(end),n_track),'spline');
    x_right_plot = interp1(s_track,x_right,linspace(s_track(1),s_track(end),n_track),'spline');
    y_right_plot = interp1(s_track,y_right,linspace(s_track(1),s_track(end),n_track),'spline');
    
    % (4.2) Interpolate the solution to the points given by the FPS
    dt = 1/30;
    t_plot = -8:dt:80.0;
    
    s_plot = interp1(q(8,:), s, t_plot,'spline')';
    q_plot = interp1(q(8,:), q', t_plot,'spline')';
    qa_plot = interp1(q(8,:), qa', t_plot,'spline')';
    u_plot = interp1(q(8,:), u', t_plot,'spline')';
    energy_plot = interp1(q(8,:), energy', t_plot,'spline')';
    x_plot = interp1(q(8,:), x, t_plot,'spline')';
    y_plot = interp1(q(8,:), y, t_plot,'spline')';
    r_center_plot = interp1(q(8,:), r_center', t_plot,'spline')';
    
    % (4.2.1) Correct s so that is inside [0,track_length]
    s_plot(s_plot < 0) = s_plot(s_plot < 0) + track_length;
    s_plot(s_plot > track_length) = s_plot(s_plot > track_length) - track_length;
    
end

close all
for i = 420
    h=plot_run_dashboard(i,t_plot,s_plot,x_plot,y_plot,r_center_plot,q_plot,qa_plot,u_plot, x_center_plot, y_center_plot, x_left_plot, y_left_plot,...
        x_right_plot, y_right_plot, run.laptime, vehicle, vehicle_data,-min(min(qa)),energy_plot);
    print(['fig_',num2str(i)],'-dpng');
    close(h);
end
