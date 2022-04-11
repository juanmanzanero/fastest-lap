function h=plot_run_dashboard(i,t,s,x,y,r_center,q,qa,u, x_center, y_center, x_left, y_left,...
    x_right, y_right, laptime, vehicle, vehicle_data, Fz_max)

h = figure('Units','normalized','Position',[0 0 1 1]);
hold on;
% (1) Plot track
patch([x_left, x_right], [-y_left, -y_right], [13/255, 17/255, 23/255]);
plot(x_center,-y_center,'--','LineWidth',2,'Color',[1,1,1])
axis equal
h.CurrentAxes.Visible = 'off';
ax = gca;
ax.Position = [0,0,1,1];
% (2) Plot throttle up to i
col = -u(2,1:i);  % This is the color, vary with x in this case.
col = min(max(-1,col),1);
max_col = max(col);
col(col>0) = col(col>0)/max_col;
cMap = interp1([0;1],[0 1 0; 1 0 0],linspace(0,1,256));
z = zeros(size(x));
surface([x(max(1,i-1400):i),x(max(1,i-1400):i)],[-y(max(1,i-1400):i),-y(max(1,i-1400):i)],[z(max(1,i-1400):i),z(max(1,i-1400):i)],[col(max(1,i-1400):i)',col(max(1,i-1400):i)'],...s =
    'facecol','no',...
    'edgecol','interp',...
    'linew',4);
colormap(cMap)

% (n) Draw car
x_rr = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'rear_axle.right_tire.x');
y_rr = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'rear_axle.right_tire.y');
psi = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'psi');

plot_f1(x_rr,-y_rr,rad2deg(psi)+180,1.6+1.8, 0.73*2);

% (3) Set camera
camera_width = 40.0;
xlim([r_center(1,i)-1.777*camera_width*0.5,r_center(1,i)+1.777*camera_width*0.5])
ylim([-r_center(2,i)-camera_width*0.5,-r_center(2,i)+camera_width*0.5])

% (n) Dashboard
ax_dashboard = axes('Position',[0.0 0.4 1.0 0.59]);

hold on
%ax_throttle_brake.Visible='off';
% (n.1) Throttle and brake
plot_throttle_brake(u(2,i),u(1,i)*20);

% (n.2) Acceleration
plot_acceleration(q,qa,u,s,i,vehicle);

% (n.3) Tires
plot_tires(q,qa,u,s,i,vehicle,vehicle_data,Fz_max);

% (n.4) Track map
plot_track_map(x_center,-y_center,x(i),-y(i));

% (n.5) Telemetry
plot_telemetry(i,t,q,qa,u)

ax_dashboard.XLim = [-3.2,  8];
ax_dashboard.YLim = [ax_dashboard.YLim(1)-ax_dashboard.YLim(2)+1.2,  1.2];
ax_dashboard.Visible = 'off';
end

function plot_throttle_brake(throttle_raw,delta_raw)

% Trim throttle
throttle_raw = min(max(throttle_raw,-1),1);
patch([-0.2,-0.2,1.2,1.2,-0.2],[-0.1,1.1,1.1,-0.1,-0.1],[1,1,1])
plot([-0.2,-0.2,1.2,1.2,-0.2],[-0.1,1.1,1.1,-0.1,-0.1],'-k','LineWidth',3)

% Plot throttle and brake
throttle = throttle_raw; throttle(throttle < 0) = 0.0;
brake = -throttle_raw; brake(brake < 0) = 0.0;
plot([0,0,0.1,0.1,0],[0,1,1,0,0],'-k');

plot([0.15,0.15,0.25,0.25,0.15],[0,1,1,0,0],'-k');
patch([0,0,0.1,0.1,0],[0,brake,brake,0,0],[1,0,0]);
patch([0.15,0.15,0.25,0.25,0.15],[0,throttle,throttle,0,0],[0,1,0]);
plot([0,0.02],[0.25,0.25],'-k')
plot([0,0.02],[0.5,0.5],'-k')
plot([0,0.02],[0.75,0.75],'-k')
plot([0.08,0.1],[0.25,0.25],'-k')
plot([0.08,0.1],[0.5,0.5],'-k')
plot([0.08,0.1],[0.75,0.75],'-k')
plot(0.15+[0,0.02],[0.25,0.25],'-k')
plot(0.15+[0,0.02],[0.5,0.5],'-k')
plot(0.15+[0,0.02],[0.75,0.75],'-k')
plot(0.15+[0.08,0.1],[0.25,0.25],'-k')
plot(0.15+[0.08,0.1],[0.5,0.5],'-k')
plot(0.15+[0.08,0.1],[0.75,0.75],'-k')

% Plot steering wheel
theta = linspace(0,2*pi,100);
plot(0.75+0.25*cos(theta),0.5+0.25*sin(theta),'-k','LineWidth',2,'MarkerSize',20);


plot(0.75,0.5,'ok','MarkerFaceColor',[0,0,0]);
plot([0.75,0.75],[0.5+0.25,0.5+0.35],'-k','LineWidth',1);
plot([0.75+0.25*sin(delta_raw),0.75+0.35*sin(delta_raw)],[0.5+0.25*cos(delta_raw),0.5+0.35*cos(delta_raw)],'-k','LineWidth',1);
plot([0.75+0.3*sin(linspace(0,delta_raw,100))],[0.5+0.3*cos(linspace(0,delta_raw,100))],'Color','#D95319','LineWidth',2);

plot([0.75,0.75+0.25*sin(delta_raw+deg2rad(90))],[0.5,0.5+0.25*cos(delta_raw+deg2rad(90))],'-k','LineWidth',2);
plot([0.75,0.75+0.25*sin(delta_raw-deg2rad(180))],[0.5,0.5+0.25*cos(delta_raw-deg2rad(180))],'-k','LineWidth',2);
plot([0.75,0.75+0.25*sin(delta_raw+deg2rad(270))],[0.5,0.5+0.25*cos(delta_raw+deg2rad(270))],'-k','LineWidth',2);

axis equal


end

function plot_acceleration(q,qa,u,s,i,vehicle)
patch([1.2,1.2,3.2,3.2,1.2],[-0.1,1.1,1.1,-0.1,-0.1],[1,1,1])
plot([1.2,1.2,3.2,3.2,1.2],[-0.1,1.1,1.1,-0.1,-0.1],'-k','LineWidth',3)
plot([1.3,3.1],[0.6,0.6],'-k')
plot([2.2,2.2],[0.0,1],'-k')
% Choose how many values to retain
n_previous = 50;
i_start = max(1,i-n_previous+1);

i_values = i_start:i;
a_x = zeros(1,length(i_values));
a_y = zeros(1,length(i_values));

for i_point = i_start:i
    a_x(i_point-i_start+1) = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i_point),qa(:,i_point),u(:,i_point),s(i_point),'ax')/9.81;
    a_y(i_point-i_start+1) = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i_point),qa(:,i_point),u(:,i_point),s(i_point),'ay')/9.81;
end

col = sqrt(a_x.^2 + a_y.^2);  % This is the color, vary with x in this case.
cMap = interp1([0;0.5;1],[0 1 0; 1 1 0; 1 0 0],linspace(0,1,256));
z = zeros(size(a_x));
for gs = 1:5
    gs_x = 2.2+gs/8*cos(linspace(0,2*pi,100));
    gs_y = 0.6+gs/8*sin(linspace(0,2*pi,100));
    
    gs_x(gs_y > 1) = NaN;
    gs_y(gs_y > 1) = NaN;
    
    gs_x(gs_y < 0) = NaN;
    gs_y(gs_y < 0) = NaN;
    plot(gs_x,gs_y,'-k','LineWidth',0.5);
end
col_surf = col;
col_surf(end) = 0;
col_surf(length(col)-1) = 6;
surface([2.2+a_y/8;2.2+a_y/8],[0.6+a_x/8;0.6+a_x/8],[z;z],[col_surf;col_surf],...s =
    'facecol','no',...
    'edgecol','interp',...
    'linew',4);
plot(2.2+a_y(end)/8,0.6+a_x(end)/8,'o','MarkerEdgeColor',[0,0,0],'MarkerSize',10,'MarkerFaceColor',interp1([0;3;6],[0 1 0; 1 1 0; 1 0 0],col(end)));
colormap(cMap)



end

function plot_tires(q,qa,u,s,i,vehicle,vehicle_data,Fz_max)
xlb = -3.2;
xub = -0.2;
xspan = xub-xlb;

wb = 2;
track = 0.4*2;

r_rr = [xlb+0.5*xspan+0.5*track,-1.8];
r_rl = r_rr - [track,0.0];
r_fr = r_rr + [0,wb];
r_fl = r_fr - [track,0.0];

patch([xlb,xlb,xub,xub,xlb],[-2.5,1.1,1.1,-2.5,-2.5],[1,1,1])
plot([xlb,xlb,xub,xub,xlb],[-2.5,1.1,1.1,-2.5,-2.5],'-k','LineWidth',3)


% Plot f1
plot_f1(r_rr(1),r_rr(2),90,wb,track)

% (1) Get normal forces
Fz_fl = -qa(1,i)*vehicle_data.chassis.mass*9.81;
Fz_fr = -qa(2,i)*vehicle_data.chassis.mass*9.81;
Fz_rl = -qa(3,i)*vehicle_data.chassis.mass*9.81;
Fz_rr = -qa(4,i)*vehicle_data.chassis.mass*9.81;

% (2) Get maximum grip
mu_x_max_fl = (Fz_fl - 2000)*(vehicle_data.front_tire.mu_x_max_2  - vehicle_data.front_tire.mu_x_max_1 )/(6000 - 2000) + vehicle_data.front_tire.mu_x_max_1;
mu_x_max_fr = (Fz_fr - 2000)*(vehicle_data.front_tire.mu_x_max_2  - vehicle_data.front_tire.mu_x_max_1 )/(6000 - 2000) + vehicle_data.front_tire.mu_x_max_1;
mu_x_max_rl = (Fz_rl - 2000)*(vehicle_data.rear_tire.mu_x_max_2  - vehicle_data.rear_tire.mu_x_max_1 )/(6000 - 2000) + vehicle_data.rear_tire.mu_x_max_1;
mu_x_max_rr = (Fz_rr - 2000)*(vehicle_data.rear_tire.mu_x_max_2  - vehicle_data.rear_tire.mu_x_max_1 )/(6000 - 2000) + vehicle_data.rear_tire.mu_x_max_1;

mu_y_max_fl = (Fz_fl - 2000)*(vehicle_data.front_tire.mu_y_max_2  - vehicle_data.front_tire.mu_y_max_1 )/(6000 - 2000) + vehicle_data.front_tire.mu_y_max_1;
mu_y_max_fr = (Fz_fr - 2000)*(vehicle_data.front_tire.mu_y_max_2  - vehicle_data.front_tire.mu_y_max_1 )/(6000 - 2000) + vehicle_data.front_tire.mu_y_max_1;
mu_y_max_rl = (Fz_rl - 2000)*(vehicle_data.rear_tire.mu_y_max_2  - vehicle_data.rear_tire.mu_y_max_1 )/(6000 - 2000) + vehicle_data.rear_tire.mu_y_max_1;
mu_y_max_rr = (Fz_rr - 2000)*(vehicle_data.rear_tire.mu_y_max_2  - vehicle_data.rear_tire.mu_y_max_1 )/(6000 - 2000) + vehicle_data.rear_tire.mu_y_max_1;




% (4) Get maximum kappa and lambda
mu_x_max_fl = (Fz_fl - 2000)*(vehicle_data.front_tire.mu_x_max_2  - vehicle_data.front_tire.mu_x_max_1 )/(6000 - 2000) + vehicle_data.front_tire.mu_x_max_1;
mu_x_max_fr = (Fz_fr - 2000)*(vehicle_data.front_tire.mu_x_max_2  - vehicle_data.front_tire.mu_x_max_1 )/(6000 - 2000) + vehicle_data.front_tire.mu_x_max_1;
mu_x_max_rl = (Fz_rl - 2000)*(vehicle_data.rear_tire.mu_x_max_2  - vehicle_data.rear_tire.mu_x_max_1 )/(6000 - 2000) + vehicle_data.rear_tire.mu_x_max_1;
mu_x_max_rr = (Fz_rr - 2000)*(vehicle_data.rear_tire.mu_x_max_2  - vehicle_data.rear_tire.mu_x_max_1 )/(6000 - 2000) + vehicle_data.rear_tire.mu_x_max_1;

kappa_max_fl = (Fz_fl - 2000)*(vehicle_data.front_tire.kappa_max_2  - vehicle_data.front_tire.kappa_max_1 )/(6000 - 2000) + vehicle_data.front_tire.kappa_max_1;
kappa_max_fr = (Fz_fr - 2000)*(vehicle_data.front_tire.kappa_max_2  - vehicle_data.front_tire.kappa_max_1 )/(6000 - 2000) + vehicle_data.front_tire.kappa_max_1;
kappa_max_rl = (Fz_rl - 2000)*(vehicle_data.rear_tire.kappa_max_2  - vehicle_data.rear_tire.kappa_max_1 )/(6000 - 2000) + vehicle_data.rear_tire.kappa_max_1;
kappa_max_rr = (Fz_rr - 2000)*(vehicle_data.rear_tire.kappa_max_2  - vehicle_data.rear_tire.kappa_max_1 )/(6000 - 2000) + vehicle_data.rear_tire.kappa_max_1;

lambda_max_fl = deg2rad((Fz_fl - 2000)*(vehicle_data.front_tire.lambda_max_2  - vehicle_data.front_tire.lambda_max_1 )/(6000 - 2000) + vehicle_data.front_tire.lambda_max_1);
lambda_max_fr = deg2rad((Fz_fr - 2000)*(vehicle_data.front_tire.lambda_max_2  - vehicle_data.front_tire.lambda_max_1 )/(6000 - 2000) + vehicle_data.front_tire.lambda_max_1);
lambda_max_rl = deg2rad((Fz_rl - 2000)*(vehicle_data.rear_tire.lambda_max_2  - vehicle_data.rear_tire.lambda_max_1 )/(6000 - 2000) + vehicle_data.rear_tire.lambda_max_1);
lambda_max_rr = deg2rad((Fz_rr - 2000)*(vehicle_data.rear_tire.lambda_max_2  - vehicle_data.rear_tire.lambda_max_1 )/(6000 - 2000) + vehicle_data.rear_tire.lambda_max_1);

% (5) Get current kappa and lambda
kappa_fl = q(1,i);
kappa_fr = q(2,i);
kappa_rl = q(3,i);
kappa_rr = q(4,i);

lambda_fl = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'front_axle.left_tire.lambda');
lambda_fr = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'front_axle.right_tire.lambda');
lambda_rl = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'rear_axle.left_tire.lambda');
lambda_rr = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'rear_axle.right_tire.lambda');

% (6) Compute rho
rho_fl = sqrt((kappa_fl/kappa_max_fl)^2 + (lambda_fl/lambda_max_fl)^2);
rho_fr = sqrt((kappa_fr/kappa_max_fr)^2 + (lambda_fr/lambda_max_fr)^2);
rho_rl = sqrt((kappa_rl/kappa_max_rl)^2 + (lambda_rl/lambda_max_rl)^2);
rho_rr = sqrt((kappa_rr/kappa_max_rr)^2 + (lambda_rr/lambda_max_rr)^2);

% (7) Compute mu
mu_x_fl = mu_x_max_fl*(kappa_fl/kappa_max_fl)/rho_fl*sin(vehicle_data.front_tire.Qy*atan(rho_fl*pi/(2.0*atan(vehicle_data.front_tire.Qy))));
mu_y_fl = mu_y_max_fl*(lambda_fl/lambda_max_fl)/rho_fl*sin(vehicle_data.front_tire.Qy*atan(rho_fl*pi/(2.0*atan(vehicle_data.front_tire.Qy))));

mu_x_fr = mu_x_max_fr*(kappa_fr/kappa_max_fr)/rho_fr*sin(vehicle_data.front_tire.Qy*atan(rho_fr*pi/(2.0*atan(vehicle_data.front_tire.Qy))));
mu_y_fr = mu_y_max_fr*(lambda_fr/lambda_max_fr)/rho_fr*sin(vehicle_data.front_tire.Qy*atan(rho_fr*pi/(2.0*atan(vehicle_data.front_tire.Qy))));

mu_x_rl = mu_x_max_rl*(kappa_rl/kappa_max_rl)/rho_rl*sin(vehicle_data.front_tire.Qy*atan(rho_rl*pi/(2.0*atan(vehicle_data.front_tire.Qy))));
mu_y_rl = mu_y_max_rl*(lambda_rl/lambda_max_rl)/rho_rl*sin(vehicle_data.front_tire.Qy*atan(rho_rl*pi/(2.0*atan(vehicle_data.front_tire.Qy))));

mu_x_rr = mu_x_max_rr*(kappa_rr/kappa_max_rr)/rho_rr*sin(vehicle_data.front_tire.Qy*atan(rho_rr*pi/(2.0*atan(vehicle_data.front_tire.Qy))));
mu_y_rr = mu_y_max_rr*(lambda_rr/lambda_max_rr)/rho_rr*sin(vehicle_data.front_tire.Qy*atan(rho_rr*pi/(2.0*atan(vehicle_data.front_tire.Qy))));


scale = 1/8;
Fref = vehicle_data.chassis.mass*9.81;


% (8) Get tire forces
Fx_fl = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'front_axle.left_tire.Fx');
Fx_fr = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'front_axle.right_tire.Fx');
Fx_rl = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'rear_axle.left_tire.Fx');
Fx_rr = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'rear_axle.right_tire.Fx');

Fy_fl = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'front_axle.left_tire.Fy');
Fy_fr = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'front_axle.right_tire.Fy');
Fy_rl = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'rear_axle.left_tire.Fy');
Fy_rr = calllib('libfastestlapc','get_vehicle_property',vehicle,q(:,i),qa(:,i),u(:,i),s(i),'rear_axle.right_tire.Fy');

assert( abs(-qa(1,i)*mu_x_fl*vehicle_data.chassis.mass*9.81 - Fx_fl) < 1.0e-4*abs(Fx_fl), simscape.enum.assert.action.warn);
assert( abs(-qa(1,i)*mu_y_fl*vehicle_data.chassis.mass*9.81 - Fy_fl) < 1.0e-4*abs(Fy_fl), simscape.enum.assert.action.warn );

assert( abs(-qa(2,i)*mu_x_fr*vehicle_data.chassis.mass*9.81 - Fx_fr) < 1.0e-4*abs(Fx_fr), simscape.enum.assert.action.warn );
assert( abs(-qa(2,i)*mu_y_fr*vehicle_data.chassis.mass*9.81 - Fy_fr) < 1.0e-4*abs(Fy_fr), simscape.enum.assert.action.warn );

assert( abs(-qa(3,i)*mu_x_rl*vehicle_data.chassis.mass*9.81 - Fx_rl) < 1.0e-4*abs(Fx_rl), simscape.enum.assert.action.warn );
assert( abs(-qa(3,i)*mu_y_rl*vehicle_data.chassis.mass*9.81 - Fy_rl) < 1.0e-4*abs(Fy_rl), simscape.enum.assert.action.warn );

assert( abs(-qa(4,i)*mu_x_rr*vehicle_data.chassis.mass*9.81 - Fx_rr) < 1.0e-4*abs(Fx_rr), simscape.enum.assert.action.warn );
assert( abs(-qa(4,i)*mu_y_rr*vehicle_data.chassis.mass*9.81 - Fy_rr) < 1.0e-4*abs(Fy_rr), simscape.enum.assert.action.warn );

% (3) Draw maximum grip ellipses
delta = u(1,i);
patch(r_fl(1) + scale*(-qa(1,i))*mu_y_max_fl*cos(linspace(0,2*pi,100)), r_fl(2)+scale*(-qa(1,i))*mu_x_max_fl*sin(linspace(0,2*pi,100)),[1,1,1]);
patch(r_fr(1) + scale*(-qa(2,i))*mu_y_max_fr*cos(linspace(0,2*pi,100)), r_fr(2)+scale*(-qa(2,i))*mu_x_max_fr*sin(linspace(0,2*pi,100)),[1,1,1]);
patch(r_rl(1) + scale*(-qa(3,i))*mu_y_max_rl*cos(linspace(0,2*pi,100)), r_rl(2)+scale*(-qa(3,i))*mu_x_max_rl*sin(linspace(0,2*pi,100)),[1,1,1]);
patch(r_rr(1) + scale*(-qa(4,i))*mu_y_max_rr*cos(linspace(0,2*pi,100)), r_rr(2)+scale*(-qa(4,i))*mu_x_max_rr*sin(linspace(0,2*pi,100)),[1,1,1]);

F_fl_rotated = Fx_fl*[cos(delta),sin(delta)] + Fy_fl*[-sin(delta),cos(delta)];
F_fr_rotated = Fx_fr*[cos(delta),sin(delta)] + Fy_fr*[-sin(delta),cos(delta)];

draw_arrow([r_fl(1),r_fl(1)+scale*F_fl_rotated(2)/Fref], [r_fl(2), r_fl(2) + scale*F_fl_rotated(1)/Fref]);
draw_arrow([r_fr(1),r_fr(1)+scale*F_fr_rotated(2)/Fref], [r_fr(2), r_fr(2) + scale*F_fr_rotated(1)/Fref]);
draw_arrow([r_rl(1),r_rl(1)+scale*Fy_rl/Fref], [r_rl(2), r_rl(2) + scale*Fx_rl/Fref]);
draw_arrow([r_rr(1),r_rr(1)+scale*Fy_rr/Fref], [r_rr(2), r_rr(2) + scale*Fx_rr/Fref]);

plot([r_fl(1) -  1.5*scale*(-qa(1,i))*mu_y_max_fl*sin(delta), r_fl(1) +  1.5*scale*(-qa(1,i))*mu_y_max_fl*sin(delta)],...
    [r_fl(2) -  1.5*scale*(-qa(1,i))*mu_y_max_fl*cos(delta), r_fl(2) +  1.5*scale*(-qa(1,i))*mu_y_max_fl*cos(delta)],'-.k')

plot([r_fr(1) -  1.5*scale*(-qa(2,i))*mu_y_max_fr*sin(delta), r_fr(1) +  1.5*scale*(-qa(2,i))*mu_y_max_fr*sin(delta)],...
    [r_fr(2) -  1.5*scale*(-qa(2,i))*mu_y_max_fr*cos(delta), r_fr(2) +  1.5*scale*(-qa(2,i))*mu_y_max_fr*cos(delta)],'-.k')

% Draw slip measurement
max_rho = fminunc(@(x)(-sin(vehicle_data.front_tire.Qy*atan(x*pi/(2.0*atan(vehicle_data.front_tire.Qy))))),0);

plot_slip_bar(max_rho,r_fl, qa(1,i), mu_y_max_fl, rho_fl, scale);
plot_slip_bar(max_rho,r_fr, qa(2,i), mu_y_max_fr, rho_fr, scale);
plot_slip_bar(max_rho,r_rl, qa(3,i), mu_y_max_rl, rho_rl, scale);
plot_slip_bar(max_rho,r_rr, qa(4,i), mu_y_max_rr, rho_rr, scale);

% Draw normal load bar
plot_normal_load_bar(r_fl+[scale*(-qa(1,i))*mu_y_max_fl,0], -qa(1,i), Fz_max)
plot_normal_load_bar(r_fr+[scale*(-qa(2,i))*mu_y_max_fr,0], -qa(2,i), Fz_max)
plot_normal_load_bar(r_rl+[scale*(-qa(3,i))*mu_y_max_rl,0], -qa(3,i), Fz_max)
plot_normal_load_bar(r_rr+[scale*(-qa(4,i))*mu_y_max_rr,0], -qa(4,i), Fz_max)
end

function draw_arrow(x,y)
head_length = 0.4;
plot(x,y,'-k','LineWidth',2);


triangle_base = [x(1),y(1)] + (1-head_length)*[x(2)-x(1),y(2)-y(1)];
dr = [x(2)-x(1),y(2)-y(1)];

triangle_P0 = triangle_base + [dr(2),-dr(1)]*head_length*0.4;
triangle_P1 = triangle_base - [dr(2),-dr(1)]*head_length*0.4;

patch([x(2),triangle_P0(1),triangle_P1(1)], [y(2), triangle_P0(2), triangle_P1(2)],[0.01,0.01,0.01]);
end

function plot_slip_bar(max_rho,r_c, qa_i, mu_max, rho, scale)
patch([r_c(1) - 1.5*scale*(-qa_i)*mu_max*sind(linspace(30,180,50)), r_c(1) - 1.1*scale*(-qa_i)*mu_max*sind(linspace(180,30,50))],...
    [r_c(2) - 1.5*scale*(-qa_i)*mu_max*cosd(linspace(30,180,50)), r_c(2) - 1.1*scale*(-qa_i)*mu_max*cosd(linspace(180,30,50))],[1,1,1]);

patch([r_c(1) - 1.5*scale*(-qa_i)*mu_max*sind(linspace(180,215,50)), r_c(1) - 1.1*scale*(-qa_i)*mu_max*sind(linspace(215,180,50))],...
    [r_c(2) - 1.5*scale*(-qa_i)*mu_max*cosd(linspace(180,215,50)), r_c(2) - 1.1*scale*(-qa_i)*mu_max*cosd(linspace(215,180,50))],[1,1,1]);

theta_max = interp1([0,max_rho],[30,180],min(rho,max_rho));
x_surf = [r_c(1) - 1.5*scale*(-qa_i)*mu_max*sind(linspace(30,theta_max,50)); r_c(1) - 1.1*scale*(-qa_i)*mu_max*sind(linspace(30,theta_max,50))];
y_surf = [r_c(2) - 1.5*scale*(-qa_i)*mu_max*cosd(linspace(30,theta_max,50)); r_c(2) - 1.1*scale*(-qa_i)*mu_max*cosd(linspace(30,theta_max,50))];


col = (linspace(30,theta_max,50)-30)/(180-30);
surface(x_surf,y_surf,zeros(2,50),[6*col;6*col], 'facecol','interp',...
    'edgecol','no',...
    'linew',4);
% Plot the remaining if exceeds in #80461B
if ( rho > max_rho )
    theta_max = interp1([0,max_rho,210/180*max_rho],[0,180,210],min(rho,210/180*max_rho));
    x_surf = [r_c(1) - 1.5*scale*(-qa_i)*mu_max*sind(linspace(180,theta_max,50)), r_c(1) - 1.1*scale*(-qa_i)*mu_max*sind(linspace(theta_max,180,50))];
    y_surf = [r_c(2) - 1.5*scale*(-qa_i)*mu_max*cosd(linspace(180,theta_max,50)), r_c(2) - 1.1*scale*(-qa_i)*mu_max*cosd(linspace(theta_max,180,50))];
    patch(x_surf,y_surf,[139/255, 0, 0]);
end
end

function plot_normal_load_bar(r_c, Fz, Fz_max)
bar_len = 0.3;
patch([r_c(1)+0.01,r_c(1)+0.01,r_c(1)+0.05,r_c(1)+0.05],[r_c(2)-bar_len,r_c(2),r_c(2),r_c(2)-bar_len],[1,1,1])
plot([r_c(1)+0.01,r_c(1)+0.01,r_c(1)+0.05,r_c(1)+0.05,r_c(1)+0.01],[r_c(2)-bar_len,r_c(2),r_c(2),r_c(2),r_c(2)],'-k')

x_surf = [(r_c(1)+0.01)*ones(1,100);(r_c(1)+0.05)*ones(1,100)];
y_surf = [r_c(2)-bar_len + bar_len*linspace(0,Fz/Fz_max,100);r_c(2)-bar_len + bar_len*linspace(0,Fz/Fz_max,100)];
col = 6*[linspace(0,Fz/Fz_max,100);linspace(0,Fz/Fz_max,100)];

surface(x_surf,y_surf,zeros(2,100),col, 'facecol','interp',...
    'edgecol','no',...
    'linew',4);
end

function plot_track_map(x_center, y_center, x, y)
new_width = 1.5;
x_max = max(x_center);
x_min = min(x_center);

y_max = max(y_center);
y_min = min(y_center);

% Goes from [6,8]x[-1,0]
x_max_new = 8;
x_min_new = 8 - new_width*(x_max-x_min)/(y_max-y_min);
plot(x_min_new + (x_center-x_min)*(x_max_new-x_min_new)/(x_max-x_min),-0.4 -new_width + new_width*(y_center-y_min)/(y_max-y_min),'Color',[0.95,0.95,0.95],'LineWidth',5);
plot(x_min_new + (x_center-x_min)*(x_max_new-x_min_new)/(x_max-x_min),-0.4 -new_width + new_width*(y_center-y_min)/(y_max-y_min),'Color','#0072BD','LineWidth',2);
plot(x_min_new + (x-x_min)*(x_max_new-x_min_new)/(x_max-x_min),-0.4 -new_width + new_width*(y-y_min)/(y_max-y_min),'o','Color','#D95319','MarkerFaceColor','#D95319','MarkerSize',10);
end

function plot_telemetry(i,t,q,qa,u)
patch([3.2,3.2,8,8,3.2],[-0.1,1.1,1.1,-0.1,-0.1],[13/255, 17/255, 23/255])
plot([3.2,3.2,8,8,3.2],[-0.1,1.1,1.1,-0.1,-0.1],'-k','LineWidth',3)
frame_start = 3.3;
frame_end = 7.9;
t_span = 30;

t_end = t(i) + 0.25*t_span;
t_start = t(i) - 0.75*t_span;

[~,i_start] = min(abs(t-t_start));
[~,i_end] = min(abs(t-t_end));

% Plot some lines
for dt = linspace(0,1,8+1)*t_span
plot(frame_start + (frame_start-frame_end)*([t_end,t_end]-dt-t_start)/(t_start-t_end),[-0.05,1.05],'LineWidth',0.2,'Color',[0.95,0.95,0.95]);
end

for y_grid = linspace(0,1,8+1)
   plot([frame_start-0.05,frame_end+0.05],[y_grid,y_grid],'LineWidth',0.2,'Color',[0.95,0.95,0.95]); 
end

% Plot velocity
max_velocity = max(q(5,:));
min_velocity = min(q(5,:));
plot(frame_start + (frame_start-frame_end)*(t(i_start:i_end)-t_start)/(t_start-t_end),  (q(5,i_start:i_end)-min_velocity)/(max_velocity-min_velocity),'Color','#FF00FF','LineWidth',2);


% Plot steering
min_steering = min(u(1,:));
max_steering = max(u(1,:));
plot(frame_start + (frame_start-frame_end)*(t(i_start:i_end)-t_start)/(t_start-t_end),  (u(1,i_start:i_end)-min_steering)/(max_steering-min_steering),'Color','#00FFFF','LineWidth',2);

% Plot throttle
throttle = u(2,:);
throttle(throttle<0) = 0;
throttle(throttle>1) = 1.0;
min_throttle = min(throttle);
max_throttle = max(throttle);
plot(frame_start + (frame_start-frame_end)*(t(i_start:i_end)-t_start)/(t_start-t_end),  0.5*(throttle(i_start:i_end)-min_throttle)/(max_throttle-min_throttle),'Color','#00FF00','LineWidth',2);

% Plot brake
brake = -u(2,:);
brake(brake<0) = 0;
brake(brake>1) = 1.0;
min_brake = min(brake);
max_brake = max(brake);
plot(frame_start + (frame_start-frame_end)*(t(i_start:i_end)-t_start)/(t_start-t_end),  0.5*(brake(i_start:i_end)-min_brake)/(max_brake-min_brake),'Color','#FF0000','LineWidth',2);


plot(frame_start + (frame_start-frame_end)*([t(i),t(i)]-t_start)/(t_start-t_end),[-0.05,1.05],'--r','LineWidth',1);
end
