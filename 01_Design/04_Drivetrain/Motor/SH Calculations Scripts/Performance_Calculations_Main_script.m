clc
clear all
close all

%% ADHESION CALCULATIONS MAIN SCRIPT

%{

This script has as inputs the parameters of the locomotive and as output
the maximum force, acceleration, torque and power. This serves as a upper
limit for our preliminary sizing

This script is made with the following ASSUMPTIONS:

1.Each friction coefficient corresponds to a certain scenario.
2.Each friction coefficient holds up to the max velocity (15 km/h)
%}

%Note: Fixed parameters are based on the Technical Specifications 2025 by
%IMech

%% Parameters

% Fixed Parameters

max_velocity=4.17; %m/s
adhesion_coeff= 0.15:0.05:0.6; % Lower and Upper bounds of the friction coefficient betwenn the wheels and rails
max_trailing_mass=1800; %kg, specified in technical specifications
Rolling_coeff=0.004;%Rolling Ressístance coeff
g=9.81; %m/s^2
Aerodynamic_drag=5.1; % N
max_gradient=2; % 

%Variable Parameters

G=2;
loco_mass=400 ; %KG
driven_wheels=8; % 
nr_motors=2;% one motor per bogie
transmission_eff=0.85;%Average value
wheel_radius=0.100; %m - Minimum wheel diameter is 200 mm

%% RESULTS
[f_motive, f_wheel] = forces(max_gradient, loco_mass,max_trailing_mass, g, driven_wheels, ...
    adhesion_coeff, Rolling_coeff, Aerodynamic_drag);

[max_accel_loaded,max_accel_unloaded,time_15kmh] = accelerations(f_motive,adhesion_coeff,loco_mass,max_trailing_mass, max_velocity);

[motor_torque,max_motor_speed,max_motor_power] = MotorParameters(f_wheel,wheel_radius,transmission_eff,G,adhesion_coeff,max_velocity);