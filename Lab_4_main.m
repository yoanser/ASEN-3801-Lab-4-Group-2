%% Lab 4 main
% 
% Inputs
% 
% Outputs
% 
% Methodology

clc
clear
close all

%% Constants/Equations

load("RSdata_nocontrol.mat")

const.m = 0.068; % kg             quadrotor mass
const.d = 0.060; % m              radial distance from CG to Prop
const.km = 0.0024; % N*m/N        contorl moment coefficient
const.I_x = 5.8e-5; % kg*m^2      body x-axis MOI
const.I_y = 7.2e-5; % kg*m^2      body y-axis MOI
const.I_z = 1e-4; % kg*m^2        body z-axis MOI
const.nu = 1e-3; % N/(m/s)^2      aero force coefficient
const.mu = 2e-6; % N*m/(rad/s)^2  aero moment coefficient
const.g = 9.81; % m/s^s           accel due to gravity

const.I = [ const.I_x      0           0;
               0        const.I_y      0;
               0           0        const.I_z];

control_matrix = [  -1          -1          -1          -1;
                    -const.d/sqrt(2)  -const.d/sqrt(2)  const.d/sqrt(2)   const.d/sqrt(2);
                    const.d/sqrt(2)   -const.d/sqrt(2)  -const.d/sqrt(2)  const.d/sqrt(2);
                    const.km          -const.km         const.km          -const.km;];

% Zc = 
% Lc = 
% Mc = 
% Nc = 

% moment_vector = [Zc; Lc; Mc; Nc];
% motor_forces = m*g/4*[1;1;1;1];
% moment_vector = @(f1,f2,f3,f4) control_matrix*[f1; f2; f3; f4]; % [Zc; Lc; Mc; Nc]
% force_vector = control_matrix'*moment_vector;

% tspan = [0 10];

% y0 = [000; 000; 000; 005; 00005; 005; 000; 000; 000; 0.1; 0.1; 0.1]; 
%    % [ x ;  y ;  z ; phi; theta; psi;  u ;  v ;  w ;  p ;  q ;  r ]


%% Task 1.1
function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

    
    % Inertial Position
        figure(fig(1))
        subplot(311)
        plot(time, aircraft_state_array(1,:), col); hold on
        title('Inertial Position')
        ylabel('x (m)')
        xlabel('Time (s)')
        
        subplot(312)
        plot(time, aircraft_state_array(2,:), col); hold on
        ylabel('y (m)')
        xlabel('Time (s)')
        
        subplot(313)
        plot(time, aircraft_state_array(3,:), col); hold on
        ylabel('z (m)')
        xlabel('Time (s)')
        set(gca, 'YDir','reverse')
    
    
    % Euler Angles
        figure(fig(2))
        subplot(311)
        plot(time, aircraft_state_array(4,:), col); hold on
        title('Euler Angles')
        ylabel('\phi (rad)')
        xlabel('Time (s)')
        
        subplot(312)
        plot(time, aircraft_state_array(5,:), col); hold on
        ylabel('\theta (rad)')
        xlabel('Time (s)')
        
        subplot(313)
        plot(time, aircraft_state_array(6,:), col); hold on
        ylabel('\psi (rad)')
        xlabel('Time (s)')
    
    
    % Body Velocities
        figure(fig(3))
        subplot(311)
        plot(time, aircraft_state_array(7,:), col); hold on
        title('Body Velocities')
        ylabel('u (m/s)')
        xlabel('Time (s)')
        
        subplot(312)
        plot(time, aircraft_state_array(8,:), col); hold on
        ylabel('v (m/s)')
        xlabel('Time (s)')
        
        subplot(313)
        plot(time, aircraft_state_array(9,:), col); hold on
        ylabel('w (m/s)')
        xlabel('Time (s)')
        set(gca, 'YDir','reverse')
        
    
    % Angular Velocities
        figure(fig(4))
        subplot(311)
        plot(time, aircraft_state_array(10,:), col); hold on
        title('Angular Velocities')
        ylabel('p (rad/s)')
        xlabel('Time (s)')
        
        subplot(312)
        plot(time, aircraft_state_array(11,:), col); hold on
        ylabel('q (rad/s)')
        xlabel('Time (s)')
        
        subplot(313)
        plot(time, aircraft_state_array(12,:), col); hold on
        ylabel('r (rad/s)')
        xlabel('Time (s)')
        
    
    % Control Inputs
        figure(fig(5))
        subplot(411)
        plot(time, control_input_array(1,:), col); hold on
        title('Control Inputs')
        ylabel('Z_c')
        xlabel('Time (s)')
        
        subplot(412)
        plot(time, control_input_array(2,:), col); hold on
        ylabel('L_c')
        xlabel('Time (s)')
        
        subplot(413)
        plot(time, control_input_array(3,:), col); hold on
        ylabel('M_c')
        xlabel('Time (s)')
        
        subplot(414)
        plot(time, control_input_array(4,:), col); hold on
        ylabel('N_c')
        xlabel('Time (s)')
        
    
    % 3D Path
        figure(fig(6))
        plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), aircraft_state_array(3,:), col)
        hold on
        plot3(aircraft_state_array(1,1), aircraft_state_array(2,1), aircraft_state_array(3,1),'go')
        plot3(aircraft_state_array(1,end), aircraft_state_array(2,end), aircraft_state_array(3,end),'ro')
        
        xlabel('x (m)')
        ylabel('y (m)')
        zlabel('z (m)')
        grid on
        title('Aircraft Path')
        set(gca, 'ZDir','reverse')

end


% This is the run function

time = linspace(0,10,100);

aircraft_state_array = zeros(12,length(time));
control_input_array = zeros(4,length(time));

fig = [1 2 3 4 5 6];

PlotAircraftSim(time, aircraft_state_array, control_input_array, fig,'b-')

%% Task 1.2
% Force of gravity divided by num of motors
p1_2.motor_forces = (const.m)*(const.g)/4*[1;1;1;1];

% 10 sec time span
p1_2.tspan = [0 10];

% state vector, 0 motion/acc
p1_2.y0 = [000; 000; 000; 000; 00000; 000; 000; 000; 000; 0.0; 0.0; 0.0]; 
        % [ x ;  y ;  z ; phi; theta; psi;  u ;  v ;  w ;  p ;  q ;  r ]

% ode45 with consts except nu (no drag: nu=0)
[p1_2.t, p1_2.var] = ode45(@(t,var)QuadrotorEOM(t, var, const.g, const.m, ...
        const.I, const.d, const.km, 0, const.mu, p1_2.motor_forces),...
        p1_2.tspan, p1_2.y0);

% control input array for 1.2
p1_2.control_input_array = control_matrix*p1_2.motor_forces; % [Zc; Lc; Mc; Nc]

% plot
fig = [1 2 3 4 5 6];
PlotAircraftSim(p1_2.t', p1_2.var', p1_2.control_input_array, fig,'b-')

% Quadrotor EOM ODE Function
function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)

    x = var(1);
    y = var(2);
    z = var(3);
    phi = deg2rad(var(4));
    theta = deg2rad(var(5));
    psi = deg2rad(var(6));
    u = var(7);
    v = var(8);
    w = var(9);
    p = var(10);
    q = var(11);
    r = var(12);

    I_x = I(1,1);
    I_y = I(2,2);
    I_z = I(3,3);

    control_matrix = [  -1          -1          -1          -1;
                    -d/sqrt(2)  -d/sqrt(2)  d/sqrt(2)   d/sqrt(2);
                    d/sqrt(2)   -d/sqrt(2)  -d/sqrt(2)  d/sqrt(2);
                    km          -km         km          -km;];

    control_input_array = control_matrix*motor_forces; % [Zc; Lc; Mc; Nc]

    L = -mu*sqrt(p^2+q^2+r^2)*p;
    M = -mu*sqrt(p^2+q^2+r^2)*q;
    N = -mu*sqrt(p^2+q^2+r^2)*r;

    Zc = control_input_array(1); 
    Lc = control_input_array(2); 
    Mc = control_input_array(3); 
    Nc = control_input_array(4);

    X = - nu*sqrt(u^2+v^2+w^2)*u;
    Y = - nu*sqrt(u^2+v^2+w^2)*v;
    Z = - nu*sqrt(u^2+v^2+w^2)*w;

    P_dot = [cos(theta)*cos(psi)    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)  cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
            cos(theta)*sin(psi)     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)  cos(phi)*sin(theta)*sin(phi)-sin(phi)*cos(psi);
            -sin(theta)             sin(phi)*cos(theta)                             cos(phi)*cos(theta)]*...
            [u; v; w];
    O_dot = [1  sin(phi)*tan(theta) cos(phi)*tan(theta);
            0   cos(phi)            -sin(phi);
            0   sin(phi)*sec(theta) cos(phi)*sec(theta)]*...
            [p; q; r];
    V_dot = [r*v-q*w; p*w-r*u; q*u-p*v] + ...
            g*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)] + ...
            1/m*[X; Y; Z+Zc];
    omega_dot = [(I_y-I_z)/I_x*q*r; (I_z-I_x)/I_y*p*r; (I_x-I_y)/I_z*q*p]+...
            [L/I_x; M/I_y; N/I_z] + [Lc/I_x; Mc/I_y; Nc/I_z];

    var_dot = [P_dot; O_dot; V_dot; omega_dot];
end
%% Task 1.3
% Force of gravity divided by num of motors
p1_3.motor_forces = const.m*const.g/4*[1;1;1;1];

% 10 sec time span
p1_3.tspan = [0 10];

% state vector, 0 motion/acc
p1_3.y0 = [000; 000; 000; 000; 00000; 000; 000; 000; 000; 0.0; 0.0; 0.0]; 
        % [ x ;  y ;  z ; phi; theta; psi;  u ;  v ;  w ;  p ;  q ;  r ]

% ode45 with consts with nu (drag: nu=/=0)
[p1_3.t, p1_3.var] = ode45(@(t,var)QuadrotorEOM(t, var, const.g, const.m, ...
        const.I, const.d, const.km, const.nu, const.mu, p1_3.motor_forces),...
        p1_3.tspan, p1_3.y0);

% control input array for 1.2
p1_3.control_input_array = control_matrix*p1_3.motor_forces; % [Zc; Lc; Mc; Nc]

% plot
fig = [1 2 3 4 5 6];
PlotAircraftSim(p1_3.t', p1_3.var', p1_3.control_input_array, fig,'b-')

%% Task 1.4

%% Task 1.5


%% Task 2.1
% Force of gravity divided by num of motors
p2_1.motor_forces = const.m*const.g/4*[1;1;1;1];

% 10 sec time span
p2_1.tspan = [0 10];

% state vector, small deviation
p2_1.y0 = [000; 000; 000; 005; 00000; 000; 000; 000; 000; 0.0; 0.0; 0.0]; 
% p2_1.y0 = [000; 000; 000; 000; 00005; 000; 000; 000; 000; 0.0; 0.0; 0.0]; 
% p2_1.y0 = [000; 000; 000; 000; 00000; 005; 000; 000; 000; 0.0; 0.0; 0.0]; 
% p2_1.y0 = [000; 000; 000; 000; 00000; 000; 000; 000; 000; 0.1; 0.0; 0.0]; 
% p2_1.y0 = [000; 000; 000; 000; 00000; 000; 000; 000; 000; 0.0; 0.1; 0.0]; 
% p2_1.y0 = [000; 000; 000; 000; 00000; 000; 000; 000; 000; 0.0; 0.0; 0.1]; 
        % [ x ;  y ;  z ; phi; theta; psi;  u ;  v ;  w ;  p ;  q ;  r ]

% ode45 with consts with nu (drag: nu=/=0)
[p2_1.t, p2_1.var] = ode45(@(t,var)QuadrotorEOM(t, var, const.g, const.m, ...
        const.I, const.d, const.km, const.nu, const.mu, p2_1.motor_forces),...
        p2_1.tspan, p2_1.y0);

% control input array for 1.2
p2_1.control_input_array = control_matrix*p2_1.motor_forces; % [Zc; Lc; Mc; Nc]

% plot
fig = [1 2 3 4 5 6];
PlotAircraftSim(p2_1.t', p2_1.var', p2_1.control_input_array, fig,'b-')

%% Task 2.2
function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
%% Extracting variables from state vector
    deltax = var(1);
    deltay = var(2);
    deltaz = var(3);
    deltaphi = deg2rad(var(4));
    deltatheta = deg2rad(var(5));
    deltapsi = deg2rad(var(6));
    deltau = var(7);
    deltav = var(8);
    deltaw = var(9);
    deltap = var(10);
    deltaq = var(11);
    deltar = var(12);

    %Inertias
    Ix = I(1,1);
    Iy = I(2,2);
    Iz = I(3,3);

    %Control pertubations
    deltaXc = deltaFc(1);
    deltaYc = deltaFc(2);
    deltaZc = deltaFc(3);
    deltaLc = deltaGc(1);
    deltaMc = deltaGc(2);
    deltaNc = deltaGc(3);

 % Linearized Equations!

 deltaInertialVelocity = [deltau;deltav;deltaw];

 deltaEulerAngleRates = [deltap;deltaq;deltar];

 deltaBodyAccelerations = g.*[-deltatheta;deltaphi;0] + (1/m).*[deltaXc;deltaYc;deltaZc];

 deltaBodyAngleAccelerations = [(1/Ix).*deltaLc;(1/Iy).*deltaMc;(1/Iz).*deltaNc];

 var_dot = [deltaInertialVelocity; deltaEulerAngleRates; deltaBodyAccelerations; deltaBodyAngleAccelerations];

   
end

% deviations for controls
p2_2.deltaFc = [0 0 0];
p2_2.deltaGc = [0 0 0];

% linearized eom function
[p2_2.t, p2_2.var] = ode45(@(t,var)QuadrotorEOM_Linearized(t, var, const.g, const.m, ...
        const.I, p2_2.deltaFc, p2_2.deltaGc), p2_1.tspan, p2_1.y0);
% plot
PlotAircraftSim(p2_2.t', p2_2.var', p2_1.control_input_array, fig,'r-')

%% Task 2.3
function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)

% inputs: 12x1 state var, a/c mass m, gravity g
% outputs: control force vec Fc, control moment vector Gc

%force
Fc = [0;0;-m*g];

%moments
gain = 0.004; %Nm/(rad/s)
Gc = -1 .* gain .* var(10:12);

end
%% Task 2.4
function motor_forces = ComputeMotorForces(Fc, Gc, d, km)

X_c = Fc(1); % Likely to stay 0 the majority of the time
Y_c = Fc(2); % ^Ditto
Z_c = Fc(3);

L_c = Gc(1);
M_c = Gc(2);
N_c = Gc(3);


f_1 = -1 * (Z_c + L_c + M_c + N_c);
f_2 = (d / sqrt(2)) * ((-Z_c - L_c) + (M_c + N_c));
f_3 = (d / sqrt(2)) * ((-L_c - M_c) + (Z_c + N_c));
f_4 = (km) * ((-L_c - N_c) + (Z_c + M_c));



motor_forces = [f_1, f_2, f_3, f_4];

end
%% Task 2.5

function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu)

% Task 2.5
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

%% CHANGED: Control forces and moments;
[Fc, Gc] = RotationDerivativeFeedback(var, m, g);

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


%% Task 3.1
close all;
function [lat_gains, long_gains] = Task31_gains (I)

%MOI
I_x = I(1,1); % kg*m^2 
I_y = I(2,2); % kg*m^2    

%time constants
tau1 = 0.5;
tau2 = 0.05;

%sigma
sigma1 = -1/tau1;
sigma2 = -1/tau2;

%set them as eigenvalues (aka roots)
roots = [sigma1 sigma2];
coeffs = poly(roots);

%lateral
k1_lat = coeffs(2)*I_x; %derivative gain
k2_lat = coeffs(3)*I_x; %proportional gain
lat_gains = [k1_lat k2_lat];

%longitudinal
k1_long = coeffs(2)*I_y;
k2_long = coeffs(3)*I_y;
long_gains = [k1_long k2_long];



end



%% Task 3.2
function [Fc, Gc] = InnerLoopFeedback(var,lat_gains,long_gains,m,g)

%force
Fc = [0;0;-m*g];

%moments
gain = 0.004; %Nm/(rad/s)
deltaLc = -lat_gains(1)*var(10) -lat_gains(2)*var(4);
deltaMc = -long_gains(1)*var(11) -long_gains(2)*var(5);
Gc = [deltaLc;deltaMc;-gain*var(12)];

end
%% Task 3.3

tspan = [0 10];

%various initial conditions
% [ x ;  y ;  z ; phi; theta; psi;  u ;  v ;  w ;  p ;  q ;  r ]
p33_y0_1 = [0 0 0 deg2rad(5) 0 0 0 0 0 0 0 0]';
p33_y0_2 = [0 0 0 0 deg2rad(5) 0 0 0 0 0 0 0]';  
p33_y0_3 = [0 0 0 0 0 0 0 0 0 0.1 0 0]';
p33_y0_4 = [0 0 0 0 0 0 0 0 0 0 0.1 0]';

p33_y0 = [p33_y0_1, p33_y0_2, p33_y0_3, p33_y0_4];

[lat_gains, long_gains] = Task31_gains(const.I);

for j = 1:size(p33_y0, 2)
    clear Fc Gc motor_forces
    [t, var] = ode45(@(t,v) QuadrotorEOMlinearizedCLGains(t, v, const.g, const.m, ...
        const.I, const.nu, const.mu), tspan, p33_y0(:,j));
    for i = 1:length(t)
        [Fc(i,:), Gc(i,:)] = InnerLoopFeedback(var(i,:)', lat_gains, long_gains, const.m, const.g);
        motor_forces(i,:) = ComputeMotorForces(Fc(i,:)', Gc(i,:)', const.d, const.km)';
    end
    control_input_array = [Fc(:,3)'; Gc(:,1)'; Gc(:,2)'; Gc(:,3)'];
    PlotAircraftSim(t', var', control_input_array, [1 2 3 4 5 6] + (j-1)*6, 'b-')

    figure((j-1)*6 + 6)  % grab the 6th figure for this case
    ah = gca;
    ah.Children(2).DisplayName = 'Start';  % green dot
    ah.Children(1).DisplayName = 'End';    % red dot
    legend
    
    figure(24 + (j-1)*2 + 1)
    subplot(311)
    plot(t, Fc(:,1)); hold on
    ylabel('X (N)')
    xlabel('Time (s)')
    title('Control Forces')
    subplot(312)
    plot(t, Fc(:,2)); hold on
    ylabel('Y (N)')
    xlabel('Time (s)')
    subplot(313)
    plot(t, Fc(:,3)); hold on
    ylabel('Z (N)')
    xlabel('Time (s)')

    figure(24 + (j-1)*2 + 2)
    subplot(311)
    plot(t, Gc(:,1)); hold on
    ylabel('X (Nm)')
    xlabel('Time (s)')
    title('Control Moments')
    subplot(312)
    plot(t, Gc(:,2)); hold on
    ylabel('Y (Nm)')
    xlabel('Time (s)')
    subplot(313)
    plot(t, Gc(:,3)); hold on
    ylabel('Z (Nm)')
    xlabel('Time (s)')


end

% save plots
for f = 1:32
    figure(f)
    exportgraphics(gcf, sprintf('Task_33_figure_%d.png', f), 'Resolution', 300)
end

%% Task 3.4

%% Task 3.5
%outermost loop: velocity reference
%locus is real on x axis and imaginary on y
%slide 24
%vary k3 over a range of values
%run eig command and root locus plot 
%time constant can't be more than 1.25s so that's constraint



%% Task 3.6
function [Fc, Gc] = VelocityReferenceFeedback(t, var)

end
%% Task 3.7
%repeat 3.1 but with k3



