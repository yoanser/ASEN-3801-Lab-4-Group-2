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

function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
%% Extracting variables from state vector
x = var(1);
y = var(2);
z = var(3);
phi = var(4);
theta = var(5);
psi = var(6);
u = var(7);
v = var(8);
w = var(9);
p = var(10);
q = var(11);
r = var(12);

%% assigning inertia values from I matrix
Ix = I(1,1);
Iy = I(2,2);
Iz = I(3,3);

%% Creating rotation matrices
%Standard dcm

Q = [cos(theta)*cos(psi) (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)); ...
cos(theta)*sin(psi) (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)); ...
-sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];

%Euler rotationa angle matrix

Qeuler = [1 sin(phi)*tan(theta) cos(phi)*tan(theta); ...
          0 cos(phi) -sin(phi); ... 
          0 sin(phi)*sec(theta) cos(phi)*cos(theta)];

%% Control forces and moments;
droot2 = d/sqrt(2);
%control equation
ControlVect = [-1 -1 -1 -1; -(droot2)  -(droot2) droot2 droot2; droot2 -(droot2) -(droot2) droot2; km -km km -km] * motor_forces;

%Split variables for ease of analysis
Zc = ControlVect(1);
Lc = ControlVect(2);
Mc = ControlVect(3);
Nc = ControlVect(4);



%% Translational newton's second law
udot_E = (r.*v - q.*w) + g.*(-sin(theta));
vdot_E = (p.*w - r.*u) + g.*(cos(theta)*sin(phi));
wdot_E = (q.*u - p.*v) + g.*(cos(theta)*cos(phi)) + (1/m)*(Zc);


%% Rotational newton's second law

pdot = ((Iy - Iz)/Ix).*q.*r + (1/Ix).*Lc;
qdot = ((Iz - Ix)/Iy).*p.*r + (1/Iy).*Mc;
rdot = ((Ix - Iy)/Iz).*p.*q + (1/Iz).*Nc;


%% DCM stuff

InertialAccel = Q*[u;v;w];

ax_E = InertialAccel(1);
ay_E = InertialAccel(2);
az_E = InertialAccel(3);

InertialRollAccel = Qeuler*[p;q;r];

phidot = InertialRollAccel(1);
thetadot = InertialRollAccel(2);
psidot = InertialRollAccel(3);













var_dot = [ax_E,ay_E,az_E,phidot,thetadot,psidot,udot_E,vdot_E,wdot_E,pdot,qdot,rdot]';


end