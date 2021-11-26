%clear all
%clc
close all

%mex -v -I/Users/juanmanzanero/Documents/software/fastest-lap/ CXXFLAGS='$CXXFLAGS -std=c++17' rear_axle_mex.cpp
addpath('/Users/juanmanzanero/Documents/software/fastest-lap/build/main/matlab');


nblocks = 2;
N = 10;
%ncontrol = nblocks*(N+1) - nblocks + 1; 
ncontrol = nblocks*(N+1); 
x0 = zeros(1,ncontrol);


options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations', 100000,'StepTolerance',1.0e-15); 
torque_vals = fmincon(@(x)(-rear_axle(1,x)),x0,[],[],[],[],0*ones(1,ncontrol),200*ones(1,ncontrol),[],options);

[t,q] = rear_axle(3,torque_vals);

kappa = (q(1,:)*0.139-q(5,:))./q(5,:);
    