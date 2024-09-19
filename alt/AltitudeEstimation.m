% AltitudeEstimation.m
% Author: Daniel Tohti
% Description: Predicts the apogee of a flight given velocity and altitude.
% Assumes a linear relationship between altitude vs. density, and a purely
% vertical velocity component.
clear; clc
format long
%% [Defining and Loading Variables]
g = 9.82; % m/s^2
m = 24.446; % mass of rocket, kg
A = pi*(0.1524/2)^2; % Cross-sectional area of rocket, m^2
C_d = 0.3; % Coefficient of drag of rocket

%% [Calculation]
%% Initial Conditions
t1 = 0; % s
t2 = 10; % s
h0 = 160; % m
v0 = 155.75; % m/s

%% Using Orbit Equation to Solve
[tvals, yvals] = ode45(@(t, x) ApogeeEQ(t, x, C_d, A, m, g), [t1, t2], [h0, v0]);

%% [Functions]
function dydt = ApogeeEQ(t, y, C_d, A, m, g)
    %Unpacking the state
    y1 = y(1);
    y2 = y(2);
    
    %Calculating Density
    density = rho(y1);
    
    %Derivative
    dydt1 = y2;
    dydt2 = -g-1./(2.*m).*density.*A.*C_d.*y2.^2;
    %Packing
    dydt = [dydt1; dydt2];
end
