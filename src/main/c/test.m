clear all
clc
close all


loadlibrary('libfastestlapc','../../include/fastestlapc.h')

kart = calllib('libfastestlapc','create_vehicle',[],'kart7','lot 2016 kart','cartesian', '/media/beegfs/home/r660/r660391/rs/race-sim/database/roberto-lot-kart-2016.xml');

Nx = 80;
Ny = 80;
ax = linspace(-7,7,Nx);
ay = linspace(0,15,Ny);
vtot = 050/3.6;
fval = zeros(Nx,Ny);


for i = 1 : length(ax)
    for j = 1 : length(ay)
        x0 = [vtot/0.139, 0.02, 0.0, 0.0, 0.0, 0.0];
        %%% Inputs x = { w_axle, z, phi, mu, psi, delta }
        [~,ff] = fsolve(@(x)(f(kart,x,vtot,ax(i),ay(j))), x0);
        fval(i,j) = norm(ff);
    end 
end

[AX,AY] = meshgrid(ax,ay);
contourf(AY,AX,abs(fval(:,:)')>1.0e-3)

unloadlibrary libfastestlapc;

function out = f(kart,in,vtot,ax,ay)
    equations = libpointer('doublePtr',zeros(1,6));
    calllib('libfastestlapc','steady_state_equations',equations,kart,6,vtot,ax,ay,in);

    out = equations.Value;
    
end
