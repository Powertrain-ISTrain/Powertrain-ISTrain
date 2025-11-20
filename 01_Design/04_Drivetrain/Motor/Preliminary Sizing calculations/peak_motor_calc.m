%% PEAK CALCULATIONS

%Description: This script calculates the Peak Power, Peak Torque and speed
%to obtain a certain acceleration of the locomotive . It is assumed a worst
%case track scenario, that is , maximum gradient


%% REFERENCES


%[1]- 2025 Railway Challenge Technical Specifications
%[2]- SouthHampton Report
%[3]- Powertrain Design report
%[4]-


%% 1) Total mass
m_total = m_loco + m_trail;

%% 2) Resistance forces at stand‑start (no aero) and at v_max
alpha = atan(grade / 100);
F_grade = m_total * g *sin(alpha);  % grade
F_rr    = m_total * g * Crr*cos(alpha);% rolling
F_a=m_total*a; % Force due to a imposed acceleration
F_req   = F_grade + F_rr+F_a;       % total traction force

%% 3) Wheel torque required at start
T_wheel = F_req * r_w;        

%% 4) Motor torque (per motor) for start
T_motor = (T_wheel / n_mot) / (G * eta_mech);
T_motor = 1.15 * T_motor;       % +10% margin

%% 5) Motor speed at v_max
omega_wheel = v_max / r_w;           
omega_motor = G * omega_wheel;       
rpm_motor   = omega_motor * 60/(2*pi);

%% 6) Mechanical power at wheels & apply safety margin
P_mech_wheel = F_req * v_max;        % P = F * v
P_mech_wheel = 1.15 * P_mech_wheel;  % +10% margin

%% 7) Mechanical power at motor shaft (total & per‑motor)
P_mech_motor_total = P_mech_wheel / eta_mech;
P_mech_motor_per   = P_mech_motor_total / n_mot;

%% 8) Electrical power draw (total & per‑motor)
P_elec_total = P_mech_motor_total / eta_elec;
P_elec_per   = P_elec_total / n_mot;

%% 9) Display results
fprintf('--- Peak Sizing ---\n');
fprintf('Required force (no aero): %.1f N\n', F_req);
fprintf('Wheel torque @ start: %.2f Nm\n', T_wheel);
fprintf('Motor torque (per motor): %.2f Nm\n', T_motor);
fprintf('Motor speed @ v_max: %.0f rpm\n', rpm_motor);
fprintf('Mechanical power @ wheels (with margin): %.1f W\n', P_mech_wheel);
fprintf('Mechanical power @ motor shaft: %.1f W total, %.1f W/motor\n', ...
        P_mech_motor_total, P_mech_motor_per);
fprintf('Electrical power draw: %.1f W total, %.1f W/motor\n\n', ...
        P_elec_total, P_elec_per);