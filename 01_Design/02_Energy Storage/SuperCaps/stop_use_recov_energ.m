%% ISTrain — Energy Recovery: brake -> stop 0.5 s -> run only on recovered energy
clc; clear;

%% -------------------- Vehicle / track parameters ------------------------
m_loco   = 600;      % [kg]
g        = 9.81;     % [m/s^2]
Crr      = 0.004;    % [-] rolling resistance coeff
rw       = 0.100;    % [m] wheel radius
mech_eff = 0.85;     % [-] drivetrain mech efficiency
G        = 10;       % [-] gearbox ratio
mu       = 0.35;     % [-] adhesion coeff
n_m      = 2;        % [-] number of motors
SF       = 30;       % [%] torque safety margin

v_max_kmh= 15;                 % [km/h] speed limit
v_max    = v_max_kmh/3.6;      % [m/s]
a_cmd    = 1.3;                % [m/s^2] target decel (TS.1b)

%% -------------------- Electrical limits & efficiencies ------------------
I_max     = 300;     % [A] DC bus current limit (charge/discharge)
Vbus      = 48;      % [V]

% Efficiencies
eta_sc_ch = 0.90;    % supercap charge path
eta_sc_ds = 0.90;    % supercap discharge path
eta_dc_ch = 0.85;    % DC/DC (charge)
eta_dc_ds = 0.85;    % DC/DC (discharge)
eta_tx    = mech_eff;
eta_m_reg = 0.90;    % motor+inverter in regen
eta_m_mot = 0.90;    % motor+inverter in motoring

eta_reg_path = eta_tx * eta_m_reg;                 % wheel -> DC bus (regen)
eta_store    = eta_reg_path * eta_dc_ch * eta_sc_ch; % wheel -> stored (info)
eta_drive    = eta_dc_ds * eta_m_mot * eta_tx;     % cap -> wheel (motoring)

%% -------------------- Supercapacitor selection --------------------------
% 1: 3x16V58F (19.33F);  2: 48V 83F;  3: 48V 165F
cap_choice = 3;

switch cap_choice
    case 1, C=19.33; ESR=3*0.0029; cap_name='3x16V 58F (19.33F)';
    case 2, C=83;    ESR=0.0022;   cap_name='48V 83F';
    case 3, C=165;   ESR=0.0011;   cap_name='48V 165F';
    otherwise, error('Invalid cap_choice');
end

%% -------------------- Pre-charge (optimal or manual) --------------------
use_optimal_V0 = true;
Erecov_target  = 7.8e3;  % [J] target event energy (for V0 formula)
if use_optimal_V0
    V0 = sqrt(max(Vbus^2 - (2*Erecov_target)/C,0));
else
    V0 = 0;
end
V0 = min(V0, Vbus);

%% -------------------- Phase A: braking (vmax -> 0) ----------------------
dtA    = 0.01;
t_stop = v_max / a_cmd;
tA     = (0:dtA:t_stop).';  NA=numel(tA);

vA   = max(v_max - a_cmd.*tA, 0);     % speed [m/s]
xA   = cumtrapz(tA, vA);              % distance [m]
F_rr = Crr*m_loco*g;                  % rolling resistance [N]
F_adh= mu*m_loco*g;                   % adhesion limit [N]

VA   = zeros(NA,1); VA(1)=V0;
EA   = zeros(NA,1); EA(1)=0.5*C*V0^2;
PA   = zeros(NA,1); IA=zeros(NA,1); FR=zeros(NA,1); Tm=zeros(NA,1);

for k=1:NA-1
    F_req    = m_loco*a_cmd;                   % desired braking force
    F_need   = max(F_req - F_rr, 0);           % what motor must create
    F_byAdh  = min(F_need, F_adh);             % adhesion limit

    % torque per motor (motor side) with safety margin
    T_req    = (F_byAdh*rw)/(G*mech_eff)/n_m;
    T_lim    = (1-SF/100)*(F_adh*rw)/(G*mech_eff)/n_m;
    scl      = min(1, T_lim/max(T_req,eps));
    F_use    = F_byAdh*scl; FR(k)=F_use; Tm(k)=T_req*scl;

    % power to DC from wheel (regen path)
    P_wheel  = F_use * vA(k);                  % [W]
    P_toDC   = eta_reg_path * P_wheel;         % [W]

    % charge limits: current and ESR
    I_esr    = max((Vbus - VA(k))/max(ESR,1e-6),0);
    I_allow  = min(I_max, I_esr);
    P_Ilim   = Vbus * I_allow;

    % power stored into cap after DC/DC + cap charge efficiency
    P_cap    = min(P_toDC, P_Ilim) * eta_dc_ch * eta_sc_ch;
    if VA(k) >= Vbus-1e-3, P_cap=0; end

    % update cap state
    dE       = P_cap*dtA;
    EA(k+1)  = EA(k) + dE;
    VA(k+1)  = min(sqrt(2*EA(k+1)/C), Vbus);

    % logs
    PA(k) = P_cap;
    IA(k) = (P_cap>0) .* P_cap / max(Vbus,1e-6);
end

Erec = EA(end);      % [J] recovered
Vend = VA(end);

%% -------------------- Phase B: dwell (0.5 s stop) -----------------------
t_dwell = 0.5;
tB  = (dtA:dtA:t_dwell).'; NB=numel(tB);
vB  = zeros(NB,1);  xB=zeros(NB,1);
VB  = Vend*ones(NB,1); EB=Erec*ones(NB,1);
PB  = zeros(NB,1);  IB=zeros(NB,1);

%% -------------------- Phase C: run only on recovered energy -------------
dtC     = 0.01;
tmaxC   = 120;                       % [s] safety cap on duration
tC      = []; vC = []; xC = [];
VC      = []; EC = []; PC = []; IC = []; FC = [];

% initial states for Phase C
t_now = 0;
v     = 0;                           % [m/s]
x     = 0;                           % [m]
Ecap  = Erec;                        % [J]
Vcap  = Vend;                        % [V]

% static limits (adhesion & torque)
T_adh_per_motor   = (F_adh*rw)/(G*mech_eff)/n_m;      % [Nm]
T_limit_per_motor = (1 - SF/100) * T_adh_per_motor;   % [Nm]
F_torque_lim      = (T_limit_per_motor*n_m*G*mech_eff)/rw; % [N] at wheel
F_trac_abs_lim    = min(F_torque_lim, F_adh);         % [N]

while (t_now < tmaxC) && (Ecap > 0 || v > 0)
    % available cap power limited by current & ESR
    I_esr_dis = Vcap / max(ESR, 1e-6);          % [A]
    I_allow   = min(I_max, I_esr_dis);           % [A]
    P_cap_out = Vbus * I_allow;                  % [W]
    % don't spend more energy than available this step
    P_cap_out = min(P_cap_out, Ecap/dtC);

    % wheel power after discharge path losses
    P_wheel_avail = eta_drive * P_cap_out;       % [W]

    % traction force (power-limited vs adhesion/torque-limited)
    if Ecap > 0 && P_wheel_avail > 0
        if v < 0.05
            F_trac = F_trac_abs_lim;
        else
            F_power = P_wheel_avail / max(v,1e-6); % P = F*v
            F_trac  = min([F_power, F_trac_abs_lim]);
        end
    else
        F_trac = 0;
    end

    % longitudinal dynamics (+ coast-down if no traction)
    F_net = F_trac - F_rr;
    if Ecap <= 0 || P_cap_out <= 0
        F_net = -F_rr;
    end
    a = F_net / m_loco;

    % integrate v & x with speed cap
    v = max(0, min(v + a*dtC, v_max));
    x = x + v*dtC;

    % update cap energy/voltage
    Ecap = max(Ecap - P_cap_out*dtC, 0);
    Vcap = sqrt(max(2*Ecap/C, 0));

    % logs
    tC(end+1,1) = t_now + dtC;
    vC(end+1,1) = v;
    xC(end+1,1) = x;
    VC(end+1,1) = Vcap;
    EC(end+1,1) = Ecap;
    PC(end+1,1) = P_wheel_avail;   % power at wheels (delivered)
    IC(end+1,1) = I_allow;         % DC bus current
    FC(end+1,1) = F_trac;          % traction force

    % stop condition: no energy and stopped
    if Ecap <= 0 && v <= 1e-3
        break;
    end

    t_now = t_now + dtC;
end

%% -------------------- Build full time-series (A | B | C) ----------------
t_full   = [tA;               tA(end)+tB;                 tA(end)+tB(end)+tC];
v_full   = [vA;               vB;                         vC];
x_full   = [xA;               xA(end)+xB;                 xA(end)+xB(end)+xC];
V_full   = [VA;               VB;                         VC];
I_full   = [IA;               IB;                         IC];
P_full   = [PA;               zeros(numel(tB),1);         PC];  % A: stored, C: wheel

% phase boundaries
tA_end = tA(end);
tB_end = tA_end + tB(end);
tC_end = tB_end + tC(end);

%% -------------------- Reports ------------------------------------------
fprintf('--- ISTrain Energy Recovery event ---\n');
fprintf('Capacitor: %s, C=%.2f F, ESR=%.4f ohm\n', cap_name, C, ESR);
fprintf('V0=%.2f V  -> Vend=%.2f V\n', V0, Vend);
fprintf('Recovered energy after braking: %.2f kJ\n', Erec/1000);
fprintf('Distance during braking: %.2f m\n', xA(end));
fprintf('Distance using ONLY recovered energy: %.2f m\n', xC(end));
fprintf('Peak speed on recovered energy: %.2f km/h\n', max(vC)*3.6);

%% -------------------- Phase coloring (RGB, light) -----------------------
colA = [0.85 0.93 1.00];   % Braking
colB = [0.96 0.96 0.86];   % Dwell
colC = [0.88 1.00 0.88];   % Energy-only

%% -------------------- PLOTS (phases clearly divided) --------------------
figure('Name','ISTrain — Energy Recovery (Phases highlighted)','Color','w');

% 1) SPEED vs time
ax1 = subplot(2,3,1);
plot(t_full, v_full*3.6, 'k', 'LineWidth', 1.8); grid on; hold on;
yl = ylim;
patch([0 tA_end tA_end 0],[yl(1) yl(1) yl(2) yl(2)],colA,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax1);
patch([tA_end tB_end tB_end tA_end],[yl(1) yl(1) yl(2) yl(2)],colB,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax1);
patch([tB_end tC_end tC_end tB_end],[yl(1) yl(1) yl(2) yl(2)],colC,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax1);
uistack(ax1.Children(1:3),'bottom'); xline(tA_end,'k--'); xline(tB_end,'k--');
xlabel('Time [s]'); ylabel('Speed [km/h]'); title('Speed vs time');

% 2) POWER vs time (stored in A; delivered to wheels in C)
ax2 = subplot(2,3,2);
plot(t_full, P_full/1000, 'k', 'LineWidth', 1.8); grid on; hold on;
yl = ylim;
patch([0 tA_end tA_end 0],[yl(1) yl(1) yl(2) yl(2)],colA,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax2);
patch([tA_end tB_end tB_end tA_end],[yl(1) yl(1) yl(2) yl(2)],colB,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax2);
patch([tB_end tC_end tC_end tB_end],[yl(1) yl(1) yl(2) yl(2)],colC,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax2);
uistack(ax2.Children(1:3),'bottom'); xline(tA_end,'k--'); xline(tB_end,'k--');
xlabel('Time [s]'); ylabel('Power [kW]');
title('Power vs time (A: stored, C: delivered at wheels)');

% 3) VOLTAGE vs time
ax3 = subplot(2,3,3);
plot(t_full, V_full, 'k', 'LineWidth', 1.8); grid on; hold on;
yl = ylim;
patch([0 tA_end tA_end 0],[yl(1) yl(1) yl(2) yl(2)],colA,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax3);
patch([tA_end tB_end tB_end tA_end],[yl(1) yl(1) yl(2) yl(2)],colB,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax3);
patch([tB_end tC_end tC_end tB_end],[yl(1) yl(1) yl(2) yl(2)],colC,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax3);
uistack(ax3.Children(1:3),'bottom'); xline(tA_end,'k--'); xline(tB_end,'k--');
xlabel('Time [s]'); ylabel('V_{cap} [V]'); title('Supercapacitor voltage vs time');

% 4) CURRENT vs time
ax4 = subplot(2,3,4);
plot(t_full, I_full, 'k', 'LineWidth', 1.8); grid on; hold on;
yl = ylim;
patch([0 tA_end tA_end 0],[yl(1) yl(1) yl(2) yl(2)],colA,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax4);
patch([tA_end tB_end tB_end tA_end],[yl(1) yl(1) yl(2) yl(2)],colB,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax4);
patch([tB_end tC_end tC_end tB_end],[yl(1) yl(1) yl(2) yl(2)],colC,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax4);
uistack(ax4.Children(1:3),'bottom'); xline(tA_end,'k--'); xline(tB_end,'k--');
xlabel('Time [s]'); ylabel('I_{bus} [A]'); title('DC bus current vs time');

% 5) DISTANCE vs time
ax5 = subplot(2,3,5);
plot(t_full, x_full, 'k', 'LineWidth', 1.8); grid on; hold on;
yl = ylim;
patch([0 tA_end tA_end 0],[yl(1) yl(1) yl(2) yl(2)],colA,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax5);
patch([tA_end tB_end tB_end tA_end],[yl(1) yl(1) yl(2) yl(2)],colB,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax5);
patch([tB_end tC_end tC_end tB_end],[yl(1) yl(1) yl(2) yl(2)],colC,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax5);
uistack(ax5.Children(1:3),'bottom'); xline(tA_end,'k--'); xline(tB_end,'k--');
xlabel('Time [s]'); ylabel('Distance [m]'); title('Distance vs time');

% 6) (Optional) FORCE vs time in Phase C
ax6 = subplot(2,3,6);
tFC = tA(end)+tB(end)+tC;
plot(tFC, FC, 'k', 'LineWidth', 1.8); grid on; hold on;
yl = ylim;
patch([0 tA_end tA_end 0],[yl(1) yl(1) yl(2) yl(2)],colA,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax6);
patch([tA_end tB_end tB_end tA_end],[yl(1) yl(1) yl(2) yl(2)],colB,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax6);
patch([tB_end tC_end tC_end tB_end],[yl(1) yl(1) yl(2) yl(2)],colC,'EdgeColor','none','FaceAlpha',0.20,'Parent',ax6);
uistack(ax6.Children(1:3),'bottom'); xline(tA_end,'k--'); xline(tB_end,'k--');
xlabel('Time [s]'); ylabel('F_{trac} [N]'); title('Tractive force (Phase C)');

sgtitle(sprintf('Energy Recovery — %s | E_{rec}=%.2f kJ | Peak v=%.2f km/h', ...
    cap_name, Erec/1000, max(vC)*3.6));
