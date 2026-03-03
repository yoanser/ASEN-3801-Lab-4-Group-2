% Author: Parinie Gupta
% Date: 03/03/26
clc;
clear;
close all;

%% Lab Task 1

% load data file
load("RSdata_nocontrol.mat")

%create structure of constants for quadrotor

QR = struct();
QR.mass = 0.068; %kg
QR.d_cg= 0.060; % m (radial distance from CG to propeller)
QR.km = 0.0024; % N*m/(N) (control moment coefficient)
QR.Ix = 5.8e-5; %kgm2 (MOI for body-x)
QR.Iy = 7.2e-5; %kgm2 (MOI for body-y)
QR.Iz = 1.0e-5; %kgm2 (MOI for body-z)
QR.nu = 1e-3; % N/(m/s)^2 (force coefficient)
QR.mu = 2e-6; % N*m/(rad/s)^2 (aero moment coefficient)

%% Part 2



