%% Task 3.5
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
    
    control_matrix = [  -1                  -1                  -1                  -1;
                        -const.d/sqrt(2)    -const.d/sqrt(2)     const.d/sqrt(2)     const.d/sqrt(2);
                         const.d/sqrt(2)    -const.d/sqrt(2)    -const.d/sqrt(2)     const.d/sqrt(2);
                         const.km           -const.km            const.km           -const.km;];

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

[p3_5.lat_gains_3_1, p3_5.long_gains_3_1] = Task31_gains(const.I);

%% 3.5 - Latitudinal Gain K3
% sweep
p3_5.k3_vals = linspace(-0.01,0.1,200000);

for i = 1:length(p3_5.k3_vals)
    k3 = p3_5.k3_vals(i);

    A = [   0                const.g                          0;
            0                0                                1;
            -k3/const.I_x   -p3_5.lat_gains_3_1(2)/const.I_x -p3_5.lat_gains_3_1(1)/const.I_x];
    lat_eigenvalues(:,i) = eig(A);
end

figure
hold on
grid on

for i = 1:3
    plot(real(lat_eigenvalues(i,:)), imag(lat_eigenvalues(i,:)))
end

xlabel('Real')
ylabel('Imaginary')
title('Lateral Eigenvalue Locus vs k_3')
xline(-0.8, '--r', 'Time Constant Limit')
xlim([-1.1 -0.7])

j = 0;
for i = 1:length(lat_eigenvalues)
    if imag(lat_eigenvalues(:,i)) < 1E-5
        if real(lat_eigenvalues(:,i)) < -0.8
            j = j+1;
            lat_valid_eig(:,j) = lat_eigenvalues(:,i);
            lat_Valid_k3(j) = p3_5.k3_vals(i); 
        end
    end
end

scatter(real(lat_valid_eig), imag(lat_valid_eig)); %, "ZData", Valid_k3);


%% 3.5 - Longitudinal Gain K3
% sweep
p3_5.k3_vals = linspace(-0.01,0.1,200000);

for i = 1:length(p3_5.k3_vals)
    k3 = p3_5.k3_vals(i);

    A = [   0                const.g                          0;
            0                0                                1;
            -k3/const.I_y   -p3_5.long_gains_3_1(2)/const.I_y -p3_5.long_gains_3_1(1)/const.I_y];
    long_eigenvalues(:,i) = eig(A);
end

figure
hold on
grid on

for i = 1:3
    plot(real(long_eigenvalues(i,:)), imag(long_eigenvalues(i,:)))
end

xlabel('Real')
ylabel('Imaginary')
title('Longitudinal Eigenvalue Locus vs k_3')
xline(-0.8, '--r', 'Time Constant Limit')
xlim([-1.1 -0.7])

j = 0;
for i = 1:length(long_eigenvalues)
    if imag(long_eigenvalues(:,i)) < 1E-5
        if real(long_eigenvalues(:,i)) < -0.8
            j = j+1;
            long_valid_eig(:,j) = long_eigenvalues(:,i);
            long_Valid_k3(j) = p3_5.k3_vals(i); 
        end
    end
end

scatter(real(long_valid_eig), imag(long_valid_eig)); %, "ZData", Valid_k3);
