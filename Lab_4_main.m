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


% clear
% clc
% 
% time = linspace(0,10,100);
% 
% aircraft_state_array = zeros(12,length(time));
% control_input_array = zeros(4,length(time));
% 
% fig = [1 2 3 4 5 6];
% 
% PlotAircraftSim(time, aircraft_state_array, control_input_array, fig,'b-')
% 
%% Task 1.2
% Force of gravity divided by num of motors
p1_2.motor_forces = const.m*const.g/4*[1;1;1;1];

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

end
%% Task 2.4
function motor_forces = ComputeMotorForces(Fc, Gc, d, km)

end
%% Task 2.5
function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu)

end

%% Task 3.1

%% Task 3.2
function [Fc, Gc] = InnerLoopFeedback(var)

end
%% Task 3.3

%% Task 3.4

%% Task 3.5

%% Task 3.6
function [Fc, Gc] = VelocityReferenceFeedback(t, var)

end
%% Task 3.7
