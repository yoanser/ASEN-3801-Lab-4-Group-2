

function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)



%% Inertial Position
figure(fig(1))
subplot(311)
plot(time, aircraft_state_array(1,:), col); hold on
ylabel('x (m)')
xlabel('Time (s)')
title('Inertial Positions')
subplot(312)
plot(time, aircraft_state_array(2,:), col); hold on
ylabel('y (m)')
xlabel('Time (s)')
legend

subplot(313)
plot(time, aircraft_state_array(3,:), col); hold on
ylabel('z (m)')
xlabel('Time (s)')



%% Euler Angles
figure(fig(2))
subplot(311)
plot(time, aircraft_state_array(4,:), col); 
hold on
ylabel('\phi (rad)')
xlabel('Time (s)')
title('Euler Angles')
legend
subplot(312)
plot(time, aircraft_state_array(5,:), col); hold on
ylabel('\theta (rad)')
xlabel('Time (s)')

subplot(313)
plot(time, aircraft_state_array(6,:), col); hold on
ylabel('\psi (rad)')
xlabel('Time (s)')



%% Body Velocities
figure(fig(3))
subplot(311)
plot(time, aircraft_state_array(7,:), col); hold on
ylabel('u (m/s)')
xlabel('Time (s)')
title('Body Velocities')
legend
subplot(312)
plot(time, aircraft_state_array(8,:), col); hold on
ylabel('v (m/s)')
xlabel('Time (s)')

subplot(313)
plot(time, aircraft_state_array(9,:), col); hold on
ylabel('w (m/s)')
xlabel('Time (s)')



%% Angular Velocities
figure(fig(4))
subplot(311)
plot(time, aircraft_state_array(10,:), col); hold on
ylabel('p (rad/s)')
xlabel('Time (s)')
title('Angular Velocities')
legend

subplot(312)
plot(time, aircraft_state_array(11,:), col); hold on
ylabel('q (rad/s)')
xlabel('Time (s)')

subplot(313)
plot(time, aircraft_state_array(12,:), col); hold on
ylabel('r (rad/s)')
xlabel('Time (s)')



%% Control Inputs
figure(fig(5))

subplot(411)
plot(time, control_input_array(1,:), col); hold on
ylabel('Z_c')
xlabel('Time (s)')
title('Control Inputs')
legend



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



%% 3D Path
figure(fig(6))
plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), aircraft_state_array(3,:), col)
hold on
plot3(aircraft_state_array(1,1), aircraft_state_array(2,1), aircraft_state_array(3,1),'go','DisplayName','Start')
plot3(aircraft_state_array(1,end), aircraft_state_array(2,end), aircraft_state_array(3,end),'ro','DisplayName','End')

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
grid on
title('Aircraft Path')
legend

end


