function h=plot_run_dashboard_v2(i,t,simulation_data, x_center, y_center, x_left, y_left,...
    x_right, y_right, vehicle_data, Fz_max, skin)

x = simulation_data.("chassis.position.x");
y = simulation_data.("chassis.position.y");

screen_size = get(0,'ScreenSize');
h = figure('Position',[0 0 screen_size(3) 1080.0/1920.0*screen_size(3)]);
hold on;
% (1) Plot track
patch([x_left, x_right], [y_left, y_right], [13/255, 17/255, 23/255]);

axis equal
h.CurrentAxes.Visible = 'off';
ax = gca;
ax.Position = [0,0,1,1];

% (2) Plot throttle up to i
col = -simulation_data.("chassis.throttle")(1:i)';  % This is the color, vary with x in this case.
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
ax.CLim = [-1,1];

% (n) Draw car
x_rr = simulation_data.("rear-axle.right-tire.position.x")(i);
y_rr = simulation_data.("rear-axle.right-tire.position.y")(i);
psi  = simulation_data.("chassis.attitude.yaw")(i);

plot_f1(x_rr,-y_rr,rad2deg(psi)+180,1.6+1.8, 0.73*2, skin);

% (3) Set camera
camera_width = 40.0;
xlim([simulation_data.x_center(i)-1.777*camera_width*0.5,simulation_data.x_center(i)+1.777*camera_width*0.5])
ylim([simulation_data.y_center(i)-camera_width*0.5,simulation_data.y_center(i)+camera_width*0.5])

% (n) Dashboard
ax_dashboard = axes('Position',[0.0 0.4 1.0 0.59]);

hold on

% (n.1) Throttle and brake
plot_throttle_brake(simulation_data.("chassis.throttle")(i),simulation_data.("front-axle.steering-angle")(i)*20,simulation_data.("chassis.brake-bias")(i));

% (n.2) Acceleration
plot_acceleration(simulation_data.("chassis.acceleration.x"),simulation_data.("chassis.acceleration.y"),i);

% (n.3) Tires
understeer_ind = rad2deg(simulation_data.("chassis.understeer_oversteer_indicator"));
plot_tires(simulation_data,i,vehicle_data,Fz_max, understeer_ind, skin);

% (n.4) Track map
plot_track_map(x_center,y_center,x(i),-y(i));

% (n.5) Telemetry
plot_telemetry(i,t,simulation_data)

ax_dashboard.XLim = [-3.3,  8.1];
ax_dashboard.YLim = [ax_dashboard.YLim(1)-ax_dashboard.YLim(2)+1.2,  1.2];
ax_dashboard.Visible = 'off';
end

function plot_throttle_brake(throttle_raw,delta_raw,brake_bias_raw)

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

% Plot brake bias
plot([0.35,1.10,1.10,0.35,0.35],[0.05,0.05,0.1,0.1,0.05],'-k');
bb_mapping = @(bb)( 0.35 + (1.10-0.35)*bb );
patch([bb_mapping(brake_bias_raw),bb_mapping(brake_bias_raw)+0.025,bb_mapping(brake_bias_raw)-0.025,bb_mapping(brake_bias_raw)],...
    [0.08,0.0,0.0,0.08],[0.05,0.05,0.05]);
text(1.10,-0.005,'F','FontWeight','bold','HorizontalAlignment', 'center');
text(0.35,-0.005,'R','FontWeight','bold','HorizontalAlignment', 'center');
text(0.5*(0.35+1.10),0.15,'brake-bias','FontName','Courier','FontWeight','bold','HorizontalAlignment', 'center');
axis equal


end

function plot_acceleration(ax,ay,i)
patch([1.2,1.2,3.2,3.2,1.2],[-0.1,1.1,1.1,-0.1,-0.1],[1,1,1])
plot([1.2,1.2,3.2,3.2,1.2],[-0.1,1.1,1.1,-0.1,-0.1],'-k','LineWidth',3)
plot([1.3,3.1],[0.6,0.6],'-k')
plot([2.2,2.2],[0.0,1],'-k')
% Choose how many values to retain
n_previous = 50;
i_start = max(1,i-n_previous+1);

i_values = i_start:i;
a_x = ax(i_start:i)/9.81;
a_y = ay(i_start:i)/9.81;

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
col_surf = col(:)';
col_surf(end) = 0;
col_surf(length(col)-1) = 6;
surface([2.2+a_y(:)'/8;2.2+a_y(:)'/8],[0.6+a_x(:)'/8;0.6+a_x(:)'/8],[z(:)';z(:)'],[col_surf;col_surf],...s =
    'facecol','no',...
    'edgecol','interp',...
    'linew',4);
plot(2.2+a_y(end)/8,0.6+a_x(end)/8,'o','MarkerEdgeColor',[0,0,0],'MarkerSize',10,'MarkerFaceColor',interp1([0;3;6],[0 1 0; 1 1 0; 1 0 0],col(end)));
colormap(cMap)



end

function plot_tires(simulation_data,i,vehicle_data,Fz_max, understeer_ind, skin)

ref_load_1 = vehicle_data.front_tire.reference_load_1;
ref_load_2 = vehicle_data.front_tire.reference_load_2;
xlb = -3.2;
xub = -0.2;
xspan = xub-xlb;

wb = 2;
track = 0.4*2;

r_rr = [xlb+0.5*xspan+0.5*track,-1.8];
r_rl = r_rr - [track,0.0];
r_fr = r_rr + [0,wb];
r_fl = r_fr - [track,0.0];

patch([xlb,xlb,xub,xub,xlb],[-2.5,1.1,1.1,-2.5,-2.5],[1,1,1]);
plot([xlb,xlb,xub,xub,xlb],[-2.5,1.1,1.1,-2.5,-2.5],'-k','LineWidth',3);


% Plot f1
plot_f1(r_rr(1),r_rr(2),90,wb,track,skin);

% (1) Get normal forces
Fz_fl = -simulation_data.("chassis.Fz_fl")(i)*vehicle_data.chassis.mass*9.81;
Fz_fr = -simulation_data.("chassis.Fz_fr")(i)*vehicle_data.chassis.mass*9.81;
Fz_rl = -simulation_data.("chassis.Fz_rl")(i)*vehicle_data.chassis.mass*9.81;
Fz_rr = -simulation_data.("chassis.Fz_rr")(i)*vehicle_data.chassis.mass*9.81;

% (4) Get maximum kappa and lambda
mu_x_max_fl = (Fz_fl - ref_load_1)*(vehicle_data.front_tire.mu_x_max_2  - vehicle_data.front_tire.mu_x_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.mu_x_max_1;
mu_x_max_fr = (Fz_fr - ref_load_1)*(vehicle_data.front_tire.mu_x_max_2  - vehicle_data.front_tire.mu_x_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.mu_x_max_1;
mu_x_max_rl = (Fz_rl - ref_load_1)*(vehicle_data.rear_tire.mu_x_max_2  - vehicle_data.rear_tire.mu_x_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.mu_x_max_1;
mu_x_max_rr = (Fz_rr - ref_load_1)*(vehicle_data.rear_tire.mu_x_max_2  - vehicle_data.rear_tire.mu_x_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.mu_x_max_1;

mu_y_max_fl = (Fz_fl - ref_load_1)*(vehicle_data.front_tire.mu_y_max_2  - vehicle_data.front_tire.mu_y_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.mu_y_max_1;
mu_y_max_fr = (Fz_fr - ref_load_1)*(vehicle_data.front_tire.mu_y_max_2  - vehicle_data.front_tire.mu_y_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.mu_y_max_1;
mu_y_max_rl = (Fz_rl - ref_load_1)*(vehicle_data.rear_tire.mu_y_max_2  - vehicle_data.rear_tire.mu_y_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.mu_y_max_1;
mu_y_max_rr = (Fz_rr - ref_load_1)*(vehicle_data.rear_tire.mu_y_max_2  - vehicle_data.rear_tire.mu_y_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.mu_y_max_1;

kappa_max_fl = (Fz_fl - ref_load_1)*(vehicle_data.front_tire.kappa_max_2  - vehicle_data.front_tire.kappa_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.kappa_max_1;
kappa_max_fr = (Fz_fr - ref_load_1)*(vehicle_data.front_tire.kappa_max_2  - vehicle_data.front_tire.kappa_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.kappa_max_1;
kappa_max_rl = (Fz_rl - ref_load_1)*(vehicle_data.rear_tire.kappa_max_2  - vehicle_data.rear_tire.kappa_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.kappa_max_1;
kappa_max_rr = (Fz_rr - ref_load_1)*(vehicle_data.rear_tire.kappa_max_2  - vehicle_data.rear_tire.kappa_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.kappa_max_1;

lambda_max_fl = deg2rad((Fz_fl - ref_load_1)*(vehicle_data.front_tire.lambda_max_2  - vehicle_data.front_tire.lambda_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.lambda_max_1);
lambda_max_fr = deg2rad((Fz_fr - ref_load_1)*(vehicle_data.front_tire.lambda_max_2  - vehicle_data.front_tire.lambda_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.lambda_max_1);
lambda_max_rl = deg2rad((Fz_rl - ref_load_1)*(vehicle_data.rear_tire.lambda_max_2  - vehicle_data.rear_tire.lambda_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.lambda_max_1);
lambda_max_rr = deg2rad((Fz_rr - ref_load_1)*(vehicle_data.rear_tire.lambda_max_2  - vehicle_data.rear_tire.lambda_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.lambda_max_1);

% (5) Get current kappa and lambda
kappa_fl = simulation_data.("front-axle.left-tire.true_kappa")(i); 
kappa_fr = simulation_data.("front-axle.right-tire.true_kappa")(i); 
kappa_rl = simulation_data.("rear-axle.left-tire.true_kappa")(i);
kappa_rr = simulation_data.("rear-axle.right-tire.true_kappa")(i); 

lambda_fl = simulation_data.("front-axle.left-tire.lambda")(i); 
lambda_fr = simulation_data.("front-axle.right-tire.lambda")(i); 
lambda_rl = simulation_data.("rear-axle.left-tire.lambda")(i);
lambda_rr = simulation_data.("rear-axle.right-tire.lambda")(i); 


% (6) Compute rho
rho_fl = sqrt((kappa_fl/kappa_max_fl)^2 + (lambda_fl/lambda_max_fl)^2);
rho_fr = sqrt((kappa_fr/kappa_max_fr)^2 + (lambda_fr/lambda_max_fr)^2);
rho_rl = sqrt((kappa_rl/kappa_max_rl)^2 + (lambda_rl/lambda_max_rl)^2);
rho_rr = sqrt((kappa_rr/kappa_max_rr)^2 + (lambda_rr/lambda_max_rr)^2);


scale = 1/8;
Fref = vehicle_data.chassis.mass*9.81;


% (8) Get tire forces
Fx_fl = simulation_data.("front-axle.left-tire.force.x")(i);
Fx_fr = simulation_data.("front-axle.right-tire.force.x")(i);
Fx_rl = simulation_data.("rear-axle.left-tire.force.x")(i);
Fx_rr = simulation_data.("rear-axle.right-tire.force.x")(i);

Fy_fl = simulation_data.("front-axle.left-tire.force.y")(i);
Fy_fr = simulation_data.("front-axle.right-tire.force.y")(i);
Fy_rl = simulation_data.("rear-axle.left-tire.force.y")(i);
Fy_rr = simulation_data.("rear-axle.right-tire.force.y")(i);

% (3) Draw maximum grip ellipses
delta = simulation_data.("front-axle.steering-angle")(i);

patch(r_fl(1) + scale*(-simulation_data.("chassis.Fz_fl")(i))*mu_y_max_fl*cos(linspace(0,2*pi,100)), r_fl(2)+scale*(-simulation_data.("chassis.Fz_fl")(i))*mu_x_max_fl*sin(linspace(0,2*pi,100)),[1,1,1]);
patch(r_fr(1) + scale*(-simulation_data.("chassis.Fz_fr")(i))*mu_y_max_fr*cos(linspace(0,2*pi,100)), r_fr(2)+scale*(-simulation_data.("chassis.Fz_fr")(i))*mu_x_max_fr*sin(linspace(0,2*pi,100)),[1,1,1]);
patch(r_rl(1) + scale*(-simulation_data.("chassis.Fz_rl")(i))*mu_y_max_rl*cos(linspace(0,2*pi,100)), r_rl(2)+scale*(-simulation_data.("chassis.Fz_rl")(i))*mu_x_max_rl*sin(linspace(0,2*pi,100)),[1,1,1]);
patch(r_rr(1) + scale*(-simulation_data.("chassis.Fz_rr")(i))*mu_y_max_rr*cos(linspace(0,2*pi,100)), r_rr(2)+scale*(-simulation_data.("chassis.Fz_rr")(i))*mu_x_max_rr*sin(linspace(0,2*pi,100)),[1,1,1]);

F_fl_rotated = Fx_fl*[cos(delta),sin(delta)] + Fy_fl*[-sin(delta),cos(delta)];
F_fr_rotated = Fx_fr*[cos(delta),sin(delta)] + Fy_fr*[-sin(delta),cos(delta)];

draw_arrow([r_fl(1),r_fl(1)+scale*F_fl_rotated(2)/Fref], [r_fl(2), r_fl(2) + scale*F_fl_rotated(1)/Fref]);
draw_arrow([r_fr(1),r_fr(1)+scale*F_fr_rotated(2)/Fref], [r_fr(2), r_fr(2) + scale*F_fr_rotated(1)/Fref]);
draw_arrow([r_rl(1),r_rl(1)+scale*Fy_rl/Fref], [r_rl(2), r_rl(2) + scale*Fx_rl/Fref]);
draw_arrow([r_rr(1),r_rr(1)+scale*Fy_rr/Fref], [r_rr(2), r_rr(2) + scale*Fx_rr/Fref]);

qa = [simulation_data.("chassis.Fz_fl")'; simulation_data.("chassis.Fz_fr")'; simulation_data.("chassis.Fz_rl")'; simulation_data.("chassis.Fz_rr")'];

plot([r_fl(1) -  1.5*scale*(-qa(1,i))*mu_y_max_fl*sin(delta), r_fl(1) +  1.5*scale*(-qa(1,i))*mu_y_max_fl*sin(delta)],...
    [r_fl(2) -  1.5*scale*(-qa(1,i))*mu_y_max_fl*cos(delta), r_fl(2) +  1.5*scale*(-qa(1,i))*mu_y_max_fl*cos(delta)],'-.k')

plot([r_fr(1) -  1.5*scale*(-qa(2,i))*mu_y_max_fr*sin(delta), r_fr(1) +  1.5*scale*(-qa(2,i))*mu_y_max_fr*sin(delta)],...
    [r_fr(2) -  1.5*scale*(-qa(2,i))*mu_y_max_fr*cos(delta), r_fr(2) +  1.5*scale*(-qa(2,i))*mu_y_max_fr*cos(delta)],'-.k')

% Draw slip measurement
opts = optimoptions('fminunc','Display','off');
max_rho = fminunc(@(x)(-sin(vehicle_data.front_tire.Qy*atan(x*pi/(2.0*atan(vehicle_data.front_tire.Qy))))),0,opts);

plot_slip_bar(max_rho,r_fl, qa(1,i), mu_y_max_fl, rho_fl, scale);
plot_slip_bar(max_rho,r_fr, qa(2,i), mu_y_max_fr, rho_fr, scale);
plot_slip_bar(max_rho,r_rl, qa(3,i), mu_y_max_rl, rho_rl, scale);
plot_slip_bar(max_rho,r_rr, qa(4,i), mu_y_max_rr, rho_rr, scale);

% Draw normal load bar
plot_normal_load_bar(r_fl+[scale*(-qa(1,i))*mu_y_max_fl,0], -qa(1,i), Fz_max)
plot_normal_load_bar(r_fr+[scale*(-qa(2,i))*mu_y_max_fr,0], -qa(2,i), Fz_max)
plot_normal_load_bar(r_rl+[scale*(-qa(3,i))*mu_y_max_rl,0], -qa(3,i), Fz_max)
plot_normal_load_bar(r_rr+[scale*(-qa(4,i))*mu_y_max_rr,0], -qa(4,i), Fz_max)

% Write tire properties
dissipation_fl = simulation_data.("front-axle.left-tire.dissipation")(i);
dissipation_fr = simulation_data.("front-axle.right-tire.dissipation")(i);
dissipation_rl = simulation_data.("rear-axle.left-tire.dissipation")(i);
dissipation_rr = simulation_data.("rear-axle.right-tire.dissipation")(i);

write_tire_properties(r_fl-[scale*(-qa(1,i))*mu_y_max_fl,0],true,kappa_fl/(kappa_max_fl*max_rho),lambda_fl/(lambda_max_fl*max_rho),dissipation_fl,simulation_data.energy_fl(i));
write_tire_properties(r_fr+[scale*(-qa(2,i))*mu_y_max_fr,0],false,kappa_fr/(kappa_max_fr*max_rho),lambda_fr/(lambda_max_fr*max_rho),dissipation_fr,simulation_data.energy_fr(i));
write_tire_properties(r_rl-[scale*(-qa(3,i))*mu_y_max_rl,0],true,kappa_rl/(kappa_max_rl*max_rho),lambda_rl/(lambda_max_rl*max_rho),dissipation_rl,simulation_data.energy_rl(i));
write_tire_properties(r_rr+[scale*(-qa(4,i))*mu_y_max_rr,0],false,kappa_rr/(kappa_max_rr*max_rho),lambda_rr/(lambda_max_rr*max_rho),dissipation_rr,simulation_data.energy_rr(i));

u = simulation_data.("chassis.velocity.x")(i);
v = simulation_data.("chassis.velocity.y")(i);
omega = simulation_data.("chassis.omega.z")(i);
text(-1.2,-0.8,['u=',num2str(u*3.6,'%.2f'),'km/h'],'FontName','Courier');
text(-1.2,-0.95,['v=',num2str(v*3.6,'%.2f'),'km/h'],'FontName','Courier');
text(-1.2,-1.1,['\beta=',num2str(rad2deg(v/u),'%.1f'),'deg'],'FontName','Courier');
text(-1.2,-1.25,['\omega=',num2str(rad2deg(omega),'%.2f'),'deg/s'],'FontName','Courier');

% Write the understeer/oversteer indicator

if ( abs(understeer_ind) < 0.1 )
    % Neutral steering
    patch([-1.1,-0.3,-0.3,-1.1,-1.1],[0.7,0.7,0.9,0.9,0.7],[0.9290, 0.6940, 0.1250],'LineStyle','none');
    text(-0.7,0.8,'Neutral','HorizontalAlignment','Center','FontName','Courier','FontWeight','bold');    
elseif ( understeer_ind < 0.0 )
    % Under steering
    patch([-1.1,-0.3,-0.3,-1.1,-1.1],[0.7,0.7,0.9,0.9,0.7],[0, 0.4470, 0.7410],'LineStyle','none');
    text(-0.7,0.8,'Understeer','Color',[1,1,1],'HorizontalAlignment','Center','FontName','Courier','FontWeight','bold');            
else
    % Over steering
    patch([-1.1,-0.3,-0.3,-1.1,-1.1],[0.7,0.7,0.9,0.9,0.7],[0.8500, 0.3250, 0.0980],'LineStyle','none');
    text(-0.7,0.8,'Oversteer','Color',[1,1,1],'HorizontalAlignment','Center','FontName','Courier','FontWeight','bold');                
end
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
bar_len = 0.4;
patch([r_c(1)+0.01,r_c(1)+0.01,r_c(1)+0.08,r_c(1)+0.08],[r_c(2)-bar_len,r_c(2),r_c(2),r_c(2)-bar_len],[1,1,1])
plot([r_c(1)+0.01,r_c(1)+0.01,r_c(1)+0.08,r_c(1)+0.08,r_c(1)+0.01],[r_c(2)-bar_len,r_c(2),r_c(2),r_c(2),r_c(2)],'-k')

x_surf = [(r_c(1)+0.01)*ones(1,100);(r_c(1)+0.08)*ones(1,100)];
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

function plot_telemetry(i,t,simulation_data)
patch([3.2,3.2,8,8,3.2],[-0.1,1.1,1.1,-0.1,-0.1],[13/255, 17/255, 23/255])
plot([3.2,3.2,8,8,3.2],[-0.1,1.1,1.1,-0.1,-0.1],'-k','LineWidth',3)
frame_start = 3.3;
frame_end = 7.9;
t_span = 30;

t_end = t(i) ;
t_start = t(i) - t_span;

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
max_velocity = max(simulation_data.("chassis.velocity.x"));
min_velocity = min(simulation_data.("chassis.velocity.x"));
plot(frame_start + (frame_start-frame_end)*(t(i_start:i_end)-t_start)/(t_start-t_end),  (simulation_data.("chassis.velocity.x")(i_start:i_end)-min_velocity)/(max_velocity-min_velocity),'Color','#FF00FF','LineWidth',2);

% Plot steering
min_steering = min(simulation_data.("front-axle.steering-angle"));
max_steering = max(simulation_data.("front-axle.steering-angle"));
plot(frame_start + (frame_start-frame_end)*(t(i_start:i_end)-t_start)/(t_start-t_end),  (simulation_data.("front-axle.steering-angle")(i_start:i_end)-min_steering)/(max_steering-min_steering),'Color','#00FFFF','LineWidth',2);

% Plot throttle
throttle = simulation_data.("chassis.throttle");
throttle(throttle<0) = 0;
throttle(throttle>1) = 1.0;
min_throttle = min(throttle);
max_throttle = max(throttle);
plot(frame_start + (frame_start-frame_end)*(t(i_start:i_end)-t_start)/(t_start-t_end),  0.5*(throttle(i_start:i_end)-min_throttle)/(max_throttle-min_throttle),'Color','#00FF00','LineWidth',2);

% Plot brake
brake = -simulation_data.("chassis.throttle");
brake(brake<0) = 0;
brake(brake>1) = 1.0;
min_brake = min(brake);
max_brake = max(brake);
plot(frame_start + (frame_start-frame_end)*(t(i_start:i_end)-t_start)/(t_start-t_end),  0.5*(brake(i_start:i_end)-min_brake)/(max_brake-min_brake),'Color','#FF0000','LineWidth',2);
end

function write_tire_properties(r_c,b_left,kappa,lambda,dissipation,energy)
    box_width = 0.75;
    box_height = 0.2;
    v_space = 0.15;
    if ( b_left ) 
        r_c(1) = r_c(1) - box_width;
    else
        r_c(1) = r_c(1) + 0.2;
    end
    
    text(r_c(1),r_c(2)+box_height,['\kappa=',num2str(round(kappa*100.0),'%3d'),'%'],'FontName','Courier');
    text(r_c(1),r_c(2)+box_height-v_space,['\lambda=',num2str(round(-lambda*100.0),'%3d'),'%'],'FontName','Courier');
    text(r_c(1),r_c(2)+box_height-2*v_space,['P=',num2str(-dissipation/1000.0,'%.2f'),'kW'],'FontName','Courier');
    text(r_c(1),r_c(2)+box_height-3*v_space,['E=',num2str(-energy/1000000.0,'%.2f'),'MJ'],'FontName','Courier');
end
