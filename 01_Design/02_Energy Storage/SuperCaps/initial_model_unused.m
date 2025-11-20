%% ISTrain — Energy Recovery Challenge (charge + discharge with Eq. 5.4.1)
% - Phase A (your original): regenerative braking from 15 km/h to 0 and charging the supercap
% - Phase B (new): accelerating the loco using ONLY the supercap, with
%   a(t) = min(mu*g, P_wheel/(m*v))  (Eq. 5.4.1)

clc; clear; close all;

%% -------------------- Vehicle / track -----------------------
m_loco   = 600+1800;   % [kg]
g        = 9.81;       % [m/s^2]
Crr      = 0.004;      % [-]
rw       = 0.100;      % [m]
mech_eff = 0.85;       % [-] drivetrain
G        = 10;         % [-] gearbox
mu       = 0.35;       % [-] wheel-rail adhesion
n_m      = 2;          % motors
SF       = 30;         % [%] torque safety margin
a_max_sys = 0.3;   % [m/s^2] max allowed acceleration of the locomotive

v_max_test = 15/3.6;   % [m/s] top speed in the challenge
a_cmd_brk  = 1.3;      % [m/s^2] target decel for braking phase

%% -------------------- Electrical limits & efficiencies ---------------
I_max      = 300;      % [A] DC bus current limit (charge & discharge)
Vbus       = 45;       % [V] DC bus nominal
Vmin_cap   = 44;       % [V] minimum allowed cap voltage on discharge (converter limit)

eta_sc     = 0.90;     % supercap coulombic/charge eff
eta_dc     = 0.85;     % DC/DC converter eff (both directions)
eta_motor  = 0.90;     % motor+inverter eff (regen or motoring)
eta_tx     = mech_eff; % drivetrain
eta_reg    = eta_tx * eta_motor;               % wheels -> DC bus (regen)
eta_dis    = eta_dc * eta_motor * eta_tx;      % cap -> wheels (motoring)
eta_path   = eta_reg * eta_dc * eta_sc;        % wheels -> stored (info)

%% -------------------- Supercapacitor choice --------------------------
% cap_choice = 1: 3x16V58F series (19.33F)
% cap_choice = 2: 48V 83F
% cap_choice = 3: 48V 165F
cap_choice = 3;

switch cap_choice
    case 1
        C   = 19.33;      ESR = 3*0.0029;  cap_name = '3x16V 58F (19.33F)';
    case 2
        C   = 83;         ESR = 0.0022;    cap_name = '48V 83F';
    case 3
        C   = 165;        ESR = 0.0011;    cap_name = '48V 165F';
    otherwise, error('cap_choice inválido');
end

%% -------------------- Pre-charge V0 for braking (Phase A) -------------
use_optimal_V0 = true;
Erecov_event   = 7.8e3;      % [J] target energy per braking event (example)

if use_optimal_V0
    V0 = sqrt(max(Vbus^2 - (2*Erecov_event)/C, 0));  % clamp by energy headroom
else
    V0 = 0;
end
V0 = 44;

%% -------------------- Timeline ---------------------------------------
dt = 0.01;

%% ==================== PHASE A — BRAKING (charge) ======================
tA_stop = v_max_test / a_cmd_brk;
tA      = (0:dt:tA_stop).';   N = numel(tA);

vA   = max(v_max_test - a_cmd_brk.*tA, 0);
xA   = cumtrapz(tA, vA);
F_rr = Crr * m_loco * g;
F_adh= mu  * m_loco * g;

VcapA   = zeros(N,1);  VcapA(1) = V0;
EcapA   = zeros(N,1);  EcapA(1) = 0.5*C*V0^2;
PrecA   = zeros(N,1);
IbusA   = zeros(N,1);
FregenA = zeros(N,1);
TperMA  = zeros(N,1);

for k = 1:N-1
    F_req_total = m_loco * a_cmd_brk;                 % [N]
    F_need_motor= max(F_req_total - F_rr, 0);         % [N]
    F_by_adh    = min(F_need_motor, F_adh);
    T_req_per_m = (F_by_adh*rw)/(G*mech_eff)/n_m;
    T_adh_per_m = (F_adh*rw)/(G*mech_eff)/n_m;
    T_lim_per_m = (1-SF/100)*T_adh_per_m;
    scale       = min(1, T_lim_per_m / max(T_req_per_m, eps));
    F_used      = F_by_adh * scale;

    FregenA(k)  = F_used;
    TperMA(k)   = (F_used*rw)/(G*mech_eff)/n_m;

    Pwheels     = F_used * vA(k);                     % [W]
    PtoDC       = eta_reg * Pwheels;                  % [W]

    % Charging limits: current & ESR (buck to Vbus while Vcap<Vbus)
    I_esr       = max((Vbus - VcapA(k)) / max(ESR,1e-6), 0);
    I_allow     = min(I_max, I_esr);
    P_Ilim      = Vbus * I_allow;

    P_cap       = min(PtoDC, P_Ilim) * eta_dc * eta_sc;
    if VcapA(k) >= Vbus - 1e-3, P_cap = 0; end

    dE          = P_cap * dt;
    EcapA(k+1)  = EcapA(k) + dE;
    VcapA(k+1)  = min(sqrt(2*EcapA(k+1)/C), Vbus);

    PrecA(k)    = P_cap;
    IbusA(k)    = (P_cap>0) * P_cap / max(Vbus,1e-6);
end

E_recovered = EcapA(end);
Vcap_end_A  = VcapA(end);

%% ==================== PHASE B — ACCELERATION (discharge) ==============
% Start at rest; accelerate using ONLY the supercap until:
% - v reaches v_max_test, or
% - Vcap hits Vmin_cap, or
% - energy is exhausted.
tB_max = 12;                      % [s] just a guard
tB     = (0:dt:tB_max).';  NB = numel(tB);

vB   = zeros(NB,1);               % starts from rest
xB   = zeros(NB,1);
VcapB= zeros(NB,1);  VcapB(1) = Vcap_end_A;   % start from Phase A final voltage
EcapB= zeros(NB,1);  EcapB(1) = 0.5*C*VcapB(1)^2;

PwheelsB = zeros(NB,1);
IbusB    = zeros(NB,1);
aB       = zeros(NB,1);

for k = 1:NB-1
    if VcapB(k) <= Vmin_cap || vB(k) >= v_max_test, 
        % freeze states once any stop condition is hit
        vB(k+1:end)    = vB(k);
        xB(k+1:end)    = xB(k);
        VcapB(k+1:end) = VcapB(k);
        EcapB(k+1:end) = EcapB(k);
        aB(k+1:end)    = 0;
        break
    end

    % ------- Available electric power out of the cap (limits) -------
    I_esr_dis  = VcapB(k) / max(ESR,1e-6);      % avoid huge ESR heating
    I_allow    = min(I_max, I_esr_dis);         % current clamp
    P_cap_out  = I_allow * VcapB(k);            % electrical power drawn from cap

    % ------- Wheel power available (conversion losses) --------------
    Pwheels    = eta_dis * P_cap_out;
    PwheelsB(k)= Pwheels;
    IbusB(k)   = I_allow;

    % ------- Eq. (5.4.1): acceleration limited by traction & power --
    a_trac = min(mu*g, a_max_sys);                                      % = μR/m with R≈mg
    a_pow  = Pwheels / max(m_loco * max(vB(k),1e-3), 1e-6);% P/(m v)
    a      = min(a_trac, a_pow);
    aB(k)  = a;

    % ------- Integrate kinematics -----------------------------------
    vB(k+1) = vB(k) + a*dt;
    vB(k+1) = min(vB(k+1), v_max_test);                    % test speed cap
    xB(k+1) = xB(k) + vB(k)*dt;

    % ------- Update supercap energy/voltage --------------------------
    dE          = (P_cap_out)*dt;                          % energy drawn from cap
    EcapB(k+1)  = max(EcapB(k) - dE, 0);
    VcapB(k+1)  = sqrt(2*EcapB(k+1)/C);
end

% Trim B to the last updated index
lastB = find(diff(vB)~=0 | diff(VcapB)~=0, 1, 'last');
if isempty(lastB), lastB = 1; end
tB = tB(1:lastB+1); vB = vB(1:lastB+1); xB = xB(1:lastB+1);
VcapB = VcapB(1:lastB+1); EcapB = EcapB(1:lastB+1);
PwheelsB = PwheelsB(1:lastB); IbusB = IbusB(1:lastB); aB = aB(1:lastB);

%% -------------------- Report ------------------------------------------
fprintf('--- ISTrain Energy Recovery ---\n');
fprintf('Capacitor: %s | C=%.2f F | ESR=%.4f ohm\n', cap_name, C, ESR);
fprintf('PHASE A (brake): V0=%.2f V  ->  V_end=%.2f V | E_recovered=%.1f J (%.2f Wh)\n', ...
        V0, Vcap_end_A, E_recovered, E_recovered/3600);
fprintf('PHASE B (accel): V_start=%.2f V  ->  V_end=%.2f V | v_end=%.2f km/h | t=%.2f s\n', ...
        VcapB(1), VcapB(end), 3.6*vB(end), tB(end));
fprintf('Overall charge path efficiency (wheel->stored): %.1f %%\n', 100*eta_path);

%% -------------------- Plots -------------------------------------------
figure('Name','ISTrain — Energy Recovery (Charge + Discharge)','Color','w');

% --- Velocity-time (unchanged) ---
subplot(2,3,1);
plot(tA, vA, 'LineWidth', 1.8); hold on; grid on;
plot(tB + tA(end), vB, 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Velocity [km/h]');
title('Locomotive Velocity — charge (↓) then discharge (↑)');
legend('Braking (regen)','Acceleration (supercap)','Location','best');

% --- Power: use tB(1:end-1) to match PwheelsB length ---
subplot(2,3,2);
plot(tA, PrecA/1000, 'LineWidth', 1.8); hold on; grid on;
plot(tB(1:end-1) + tA(end), PwheelsB/1000, 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Power [kW]');
title('Power: stored (A) / delivered to wheels (B)');
legend('P_{cap,in} (Phase A)','P_{wheels} (Phase B)','Location','best');

% --- Cap voltage (states -> tB) ---
subplot(2,3,3);
plot(tA, VcapA, 'LineWidth', 1.8); hold on; grid on;
plot(tB + tA(end), VcapB, 'LineWidth', 1.8);
yline(Vbus,'--','V_{bus}'); yline(Vmin_cap,':','V_{min,cap}');
xlabel('Time [s]'); ylabel('V_{cap} [V]');
title('Supercap Voltage');

% --- Energy (states -> tB) ---
subplot(2,3,4);
plot(tA, EcapA/1000, 'LineWidth', 1.8); hold on; grid on;
plot(tB + tA(end), EcapB/1000, 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Energy [kJ]');
title('Stored Energy (A) and Remaining Energy (B)');
legend('Phase A','Phase B','Location','best');

% --- Braking forces (Phase A) ---
subplot(2,3,5);
plot(tA, FregenA, 'LineWidth', 1.8); hold on; grid on;
yline(F_rr,'--','F_{rr}'); yline(F_adh,':','F_{adh}');
xlabel('Time [s]'); ylabel('Force [N]');
title('Braking forces (regen, rolling, adhesion)');
legend('F_{regen}','F_{rr}','F_{adh}','Location','best');

% --- Acceleration over whole event (Phase A + B) ---
subplot(2,3,6); hold on; grid on;

% Phase A: braking deceleration (negative)
plot(tA, -a_cmd_brk*ones(size(tA)), 'LineWidth', 1.8);

% Phase B: acceleration from supercap
plot(tB(1:end-1) + tA(end), aB, 'LineWidth', 1.8);

% Traction limit line
yline(mu*g,'--','\mu g (traction limit)');
yline(-mu*g,'--','- \mu g (traction limit)');  % symmetric for braking

xlabel('Time [s]');
ylabel('a [m/s^2]');
title('Acceleration: braking (−) and motoring (+)');
legend('Braking decel (cmd)','Accel (supercap)','Traction limit','Location','best');
