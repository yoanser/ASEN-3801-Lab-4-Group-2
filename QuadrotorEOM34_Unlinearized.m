function var_dot = QuadrotorEOM34_Unlinearized(t, var, g, m, I, nu, mu)

% Task 3.4
% inputs: time t, 12x1 state var, a/c mass m, gravity g, 3x3 inertia matrix
% I, nu, mu
% outputs: state dot

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

%% CHANGED: Control forces and moments with task 3 control laws!
[lat_gains, long_gains] = Task31_gains(I);

[Fc, Gc] = InnerLoopFeedback(var, lat_gains, long_gains, m, g);

%Split variables for ease of analysis
Zc = Fc(3);
Lc = Gc(1);
Mc = Gc(2);
Nc = Gc(3);



%% Translational newton's second law
RWindVec = [u v w]; %Used for drag calculations
udot_E = (r.*v - q.*w) + g.*(-sin(theta)) + (1/m).*(-nu.*norm(RWindVec).*u); 
vdot_E = (p.*w - r.*u) + g.*(cos(theta)*sin(phi)) + (1/m).*(-nu.*norm(RWindVec).*v);
wdot_E = (q.*u - p.*v) + g.*(cos(theta)*cos(phi)) + (1/m).*(-nu.*norm(RWindVec).*w) + (1/m)*(Zc);


%% Rotational newton's second law
rotationVec = [p q r]; %Again used for drag

pdot = ((Iy - Iz)/Ix).*q.*r + (1/Ix).*Lc + (1/Ix).*-mu.*norm(rotationVec).*p;
qdot = ((Iz - Ix)/Iy).*p.*r + (1/Iy).*Mc + (1/Iy).*-mu.*norm(rotationVec).*q;
rdot = ((Ix - Iy)/Iz).*p.*q + (1/Iz).*Nc + (1/Iz).*-mu.*norm(rotationVec).*r;


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
