close all;
clc;
clear;

g = 9.81;

%Drone values
m=0.068; %kg
d = 0.060; %m
km = 0.0024; %N*m/N
nu = 1e-3; %N/(m/s)^2
mu = 2e-6; %N*m/(rad/s)^2
I = [5.8e-5 0 0; 0 7.2e-5 0; 0 0 1e-4];
motortrim = (g*m)/4  %Calculating trim motor force for absolutely steady flight (in hover)


%initPhi = atan((25*nu)./(m.*g))
initPhi = deg2rad(5);
initP = 0.1;
initQ = 0.1;
initR = 0.1;

newmotortrim = (m.*g)/(4*cos(initPhi))
motor_forces = [motortrim;motortrim;motortrim;motortrim];


tspan = [0 10]; %Testing 0 to 10 seconds
y0 = [0 0 0 0 0 0 0 0 0 0 0 initR]'; %Initital conditions of 0's

%This outputs trim condition!! z value remains very small
[t_control,var_control] = ode45(@(t,var)QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu), tspan, y0);
[t_uncontrol, var_uncontrol] = ode45(@(t,var)QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), tspan, y0);

motorForcearrayUncontrolled = ones(1, length(t_uncontrol)) * motortrim;

%% Computes motor forces with time

for i = 1:size(var_control,1)

[Fc(i,:),Gc(i,:)] = RotationDerivativeFeedback(var_control(i,:),m,g);
motor_forceswithControl(i,:) = ComputeMotorForces(Fc(i,:), Gc(i,:), d, km);
end

time = t_control';
outputVar = var_control';


controlArray_control = [Fc(:,3), Gc(:,1), Gc(:,2), Gc(:,3)]';
controlArray_uncontrol = [ones(size(var_uncontrol,1),1) .* (-motortrim.*4), zeros(size(var_uncontrol,1),1) , zeros(size(var_uncontrol,1),1) ,zeros(size(var_uncontrol,1),1)]';

fig = [1 2 3 4 5 6];

colcontrol.Color = 'b';
colcontrol.LineStyle='--';
colcontrol.DisplayName = 'Controlled Drone';

coluncontrol.Color = 'r';
coluncontrol.LineStyle='-';
coluncontrol.DisplayName = 'Uncontrolled Drone';
PlotAircraftSim(t_uncontrol',var_uncontrol',controlArray_uncontrol,fig,coluncontrol)
PlotAircraftSim(t_control', var_control', controlArray_control,fig, colcontrol)


%% Plot motor forces
figure();


subplot(4,1,1)
    plot(time,motor_forceswithControl(:,1),'--b'); hold on
    plot(t_uncontrol, motorForcearrayUncontrolled,'r');
    title('Motor Forces in Newtons, k1')
    xlabel('Time (s)')
    ylabel('N')
    legend('Controlled', 'Uncontrolled')
    axis padded;
subplot(4,1,2)
    
    plot(time,motor_forceswithControl(:,2),'--b'); hold on
    plot(t_uncontrol, motorForcearrayUncontrolled,'r');
    title('k2')
    axis padded;
subplot(4,1,3)
    
    plot(time,motor_forceswithControl(:,3),'--b'); hold on
    plot(t_uncontrol, motorForcearrayUncontrolled,'r');
    title('k3')
    axis padded;
subplot(4,1,4)
    
    plot(time,motor_forceswithControl(:,4),'--b'); hold on
    plot(t_uncontrol, motorForcearrayUncontrolled,'r');
    title('k4')
    axis padded;


%% No control motor forces
figure();

subplot(4,1,1)
    
    title('Motor Forces in Newtons, k1')
    xlabel('Time (s)')
    ylabel('N')

subplot(4,1,2)
    
    plot(t_uncontrol, motorForcearrayUncontrolled);
    title('k2')
subplot(4,1,3)
    
    plot(t_uncontrol, motorForcearrayUncontrolled);
    title('k3')

subplot(4,1,4)
    
   plot(t_uncontrol, motorForcearrayUncontrolled);
    title('k4')