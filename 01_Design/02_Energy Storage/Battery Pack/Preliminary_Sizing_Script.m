%% B. Continuous Cruise & Energy Sizing (split efficiencies)
% Given parameters (re‑use or re‑define as needed)
m_loco         = 600;            % kg
m_cruise_trail = 400;            % kg
Crr            = 0.004;
g              = 9.81;
grade          = 0.02;
v_cruise       = 5/3.6;          % m/s (5 km/h)
eta_mech       = 0.90;           % mechanical efficiency
eta_elec       = 0.85;           % motor+inverter efficiency
t_run          = 3;              % hours
n_mot          = 2;

%% 1) Total cruise mass
m_tot_cruise = m_loco + m_cruise_trail;

%% 2) Resistance forces at cruise speed (no aero if negligible)
F_grade2 = m_tot_cruise * g * grade;
F_rr2    = m_tot_cruise * g * Crr;
F_tot    = F_grade2 + F_rr2;     % omit F_aero if small

%% 3) Wheel‑power required
P_mech_wheel = F_tot * v_cruise;     
P_mech_wheel = 1.15 * P_mech_wheel;   % +15% margin

%% 4) Shaft‑power at the motor
P_mech_motor_total = P_mech_wheel / eta_mech;
P_mech_motor_per   = P_mech_motor_total / n_mot;

%% 5) Electrical power draw
P_elec_total = P_mech_motor_total / eta_elec;
P_elec_per   = P_elec_total / n_mot;

%% 6) Energy for t_run hours
E_Wh  = P_elec_total * t_run;   % Wh
E_kWh = E_Wh / 1000;            % kWh

%% 7) Battery current & C‑rate (example at V_pack)
V_pack = 72;                   % V nominal
I_cont = P_elec_total / V_pack;% A
Ah_req = E_Wh / V_pack;        % Ah
C_rate = I_cont / Ah_req;      

%% 8) Display results
fprintf('--- Continuous/Endurance Sizing (Revised) ---\n');
fprintf('Total constant force: %.1f N\n', F_tot);
fprintf('Mechanical wheel‑power: %.1f W\n', P_mech_wheel);
fprintf('Mechanical motor‑shaft power: %.1f W total, %.1f W/motor\n', ...
        P_mech_motor_total, P_mech_motor_per);
fprintf('Electrical power draw: %.1f W total, %.1f W/motor\n', ...
        P_elec_total, P_elec_per);
fprintf('Energy for %g h: %.1f Wh (%.2f kWh)\n', ...
        t_run, E_Wh, E_kWh);
fprintf('Battery: ~%.1f Ah @%g V, C‑rate=%.2f\n\n', ...
        Ah_req, V_pack, C_rate);
 
