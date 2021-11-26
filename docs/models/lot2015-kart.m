clear all
clc
close all

fprintf("Derivation of the equations for the go-kart\n\n\n")

syms x y z psi phi mu h;
syms dx dy dz dpsi dphi dmu u v Omega;
syms d2x d2y d2z dOmega d2psi d2phi d2mu du dv;
syms mass Ixx Iyy Izz Ixz;
syms Fx Fy Fz Tx Ty Tz;

% Set which variables are small
small_vars = {z,dz,v,dv,mu,dmu,d2mu,phi,dphi,d2phi};

tras = @(x,y,z)([1,0,0,x;0,1,0,y;0,0,1,z;0,0,0,1]);
rotx = @(phi)([1,0,0,0;0,cos(phi),-sin(phi),0;0,sin(phi),cos(phi),0;0,0,0,1]);
roty = @(mu)([cos(mu),0,sin(mu),0;0,1,0,0;-sin(mu),0,cos(mu),0;0,0,0,1]);
rotz = @(psi)([cos(psi),-sin(psi), 0, 0; sin(psi), cos(psi),0,0;0,0,1,0;0,0,0,1]);

smallrotx = @(phi)([1,0,0,0;0,1,-(phi),0;0,(phi),1,0;0,0,0,1]);
smallroty = @(mu)([1,0,(mu),0;0,1,0,0;-(mu),0,1,0;0,0,0,1]);
smallrotz = @(psi)([1,-(psi), 0, 0; (psi), 1,0,0;0,0,1,0;0,0,0,1]);

Wc0 = tras(x,y,0)*rotz(psi);
Wc0c = tras(0,0,z-h)*rotx(phi)*roty(mu);
Wcc0 = roty(-mu)*rotx(-phi)*tras(0,0,h-z);
Wc=Wc0*Wc0c;

% Dynamics: Wc = f(x,y,z,psi,phi,mu)
Wc_dot = differentiate(Wc,{x,y,z,psi,phi,mu},{u,v,dz,Omega,dphi,dmu});

% Rotational dynamics
Momega = transpose(Wc(1:3,1:3))*Wc_dot(1:3,1:3);
omega = [Momega(3,2), Momega(1,3), Momega(2,1)];

% simplify omega:
Inertia = [Ixx, 0, -Ixz; 0, Iyy, 0; -Ixz, 0, Izz];

% Newton's equations in Wc0
vG = [u,v,dz]; 
aG = [du,dv,d2z] + cross([0,0,Omega],vG); 

% Euler equations: MA = dLA + vA^(mVg). Written in Wc0
vA = [u,v,0];
vA_cross_PG = transpose(cross(vA,mass*vG));

% König's theorem: LA = Lcom + Lrel
Lrel = (Inertia*transpose(omega));
dLrel = differentiate(Lrel,{psi,Omega,mu,phi,dmu,dphi},{Omega,dOmega,dmu,dphi,d2mu,d2phi}) + ...
        transpose(cross(omega,Lrel));

dLrel_0 = Wc0c(1:3,1:3)*dLrel;

AG = [0;0;z-h];
Lcom0 = transpose(mass*cross(AG,vG));

dLcom0 = differentiate(Lcom0,{u,v,z,dz},{du,dv,dz,d2z}) + transpose(cross([0,0,Omega],Lcom0));

dL = dLrel_0 + dLcom0;

MOE = simplify(dL + vA_cross_PG);
 
MOE_LIN=linearize(MOE,small_vars);

% Check the results!
MOE1_REF = mass*(dv*h+(h-z)*Omega*u)+Ixx*d2phi-(Iyy-Izz+Ixx)*Omega*dmu - (Iyy-Izz)*Omega^2*phi...
    -(Ixx-Izz)*dOmega*mu - Ixz*dOmega;

MOE2_REF = mass*((z-h)*du+h*Omega*v)+Iyy*d2mu + (Iyy-Izz+Ixx)*Omega*dphi - (Ixx-Izz)*Omega^2*mu...
    +(Iyy-Izz)*dOmega*phi-Ixz*Omega^2;

MOE3_REF = Izz*dOmega-Ixz*(d2phi + 2*mu*dOmega + 2*dmu*Omega);
MOE_REF=[MOE1_REF;MOE2_REF;MOE3_REF];

fprintf("error")
display(simplify(MOE_REF-MOE_LIN))

fprintf("\n\t\t\t\t=> However, this time I think they are wrong!\n")
% Write the equations to be coded: q = (x,y,z,u,v,dz,psi,mu,phi,Omega,dmu,dphi)
% Mdq/dt = f
q  = {x,y,z,psi,mu,phi,u,v,dz,Omega,dmu,dphi};
dq = {u,v,dz,Omega,dmu,dphi,du,dv,d2z,dOmega,d2mu,d2phi};
M = sym(eye(12));
f(1:3) = eye(3,4)*rotz(psi)*[u;v;dz;0];
f(4) = Omega;
f(5) = dmu;
f(6) = dphi;
f(7) = -subs(mass*aG(1),{du},{0}) + Fx;
f(8) = -subs(mass*aG(2),{dv},{0}) + Fy;
f(9) = -subs(mass*aG(3),{d2z},{0}) + Fz;
f(10) = -subs(MOE_LIN(1),{dOmega,d2mu,d2phi,du,dv},{0,0,0,0,0}) + Tx;
f(11) = -subs(MOE_LIN(2),{dOmega,d2mu,d2phi,du,dv},{0,0,0,0,0}) + Tx;
f(12) = -subs(MOE_LIN(3),{dOmega,d2mu,d2phi,du,dv},{0,0,0,0,0}) + Tx;
M(7,:) = grad(mass*aG(1),dq,zeros(1,12));
M(8,:) = grad(mass*aG(2),dq,zeros(1,12));
M(9,:) = grad(mass*aG(3),dq,zeros(1,12));
M(10,:) = grad(MOE_LIN(1),dq,zeros(1,12));
M(11,:) = grad(MOE_LIN(2),dq,zeros(1,12));
M(12,:) = grad(MOE_LIN(3),dq,zeros(1,12));

invM = simplify(inv(M));