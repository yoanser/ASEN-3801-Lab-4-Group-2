close all;
clc;
clear;

g = 9.81;

%Just testing values here
m=10;
d = 3;
km = 2;
nu = 0.3;
mu = 0.8;
I = [1 0 0; 0 1 0; 0 0 1];
motortrim = (g*m)/4;  %Calculating trim motor force for absolutely steady flight 
motor_forces = [motortrim;motortrim;motortrim;motortrim];


tspan = [0 200]; %Testing 0 to 200 seconds
y0 = [0 0 0 0 0 0 0 0 0 0 0 0]'; %Initital conditions of 0's

%This outputs trim condition!! z value remains very small
[t,var] = ode45(@(t,var)QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), tspan, y0);

