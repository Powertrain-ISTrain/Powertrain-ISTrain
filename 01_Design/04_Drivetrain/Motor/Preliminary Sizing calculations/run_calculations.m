%% RUN PRELIMINARY SIZING

%% PARAMETERS FOR Adhesion_verification.m
% Fixed Parameters

max_velocity=4.17; %m/s (definido em [1])
adhesion_coeff= 0.15:0.05:0.6; % definido em [1]
max_trailing_mass=1800; %kg, (definido em [1])
Rolling_coeff=0.004;%Rolling Ressístance coeff (definido em [1])
g=9.81; %m/s^2
Aerodynamic_drag=5.1; % N % Defined em [2] 
max_gradient=1.19; % 1:84 Defined em [1] 

%Variable Parameters

G=5;
loco_mass=600 ; % (defined in [4])
driven_wheels=8; % (defined in [3])
nr_motors=2;% One motor per bogie (defined in [3])
transmission_eff=0.85;%Average value (defined in [2])
wheel_radius=0.100; %m - Minimum wheel diameter is 200 mm (defined in [4])


%% PARAMETERS FOR peak_motor_calc.m

% Fixed Parameters

v_max=4.17; %m/s (definido em [1])
m_trail=1800; %kg, (definido em [1])
Crr=0.004;%Rolling Ressístance coeff (definido em [1])
g=9.81; %m/s^2
grade=1.19;% 1:84 Defined em [1]

%Variable Parameters

G=5;
m_loco=600 ; % (defined in [4])
n_mot=2;% One motor per bogie (defined in [3])
r_w=0.100; %m - Minimum wheel diameter is 200 mm (defined in [4])
eta_mech = 0.85; %Average value (defined in [2])
eta_elec = 0.90; %Average value (defined in [2])
a=0.2 ; % Maximum Acceleration (prescribed value)
%% RUN FILES
% Adhesion_verification
peak_motor_calc