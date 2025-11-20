% chain_06B_sizing.m
% 2-stage chain sizing using 06B (p = 9.525 mm). Both sprung idlers placed on stage-2.
clearvars; close all; clc;

%% ---------- PARAMETERS ----------
Tm = 12;                 % N·m - motor max torque

% Teeth selection (06B solution suggested)
N_stage1_driver = 15;
N_stage1_driven = 46;    % 46T -> ~139.6 mm Dp for 06B
N_stage2_driver = 15;
N_stage2_driven = 49;    % 49T -> ~148.7 mm Dp for 06B

p = 9.525;               % mm - 06B pitch
eta1 = 0.95;
eta2 = 0.95;

% Centers (use your CAD numbers or adjust)
C1_mm = 100;
C2_mm = 350;

% Material / safety (approx for 06B; verify with manufacturer)
tensile_strength_chain = 10000;  % N (approx ultimate for 06B single strand - CHECK vendor)
safety_factor_chain = 2.0;
tau_allow = 50e6;     % Pa % Moderate austenitic steel

% Idler configuration: both idlers on stage-2
num_idlers_stage1 = 0;   % no sprung idlers on stage-1
num_idlers_stage2 = 2;   % both sprung idlers on stage-2 (split preload)

% Idler defaults
idler_travel = 25;     % mm - available travel for each idler
idler_preload_ratio = 0.08; % fraction of working tension used as total preload for stage-2

% Minimum per-stage total preload (keeps tiny systems from choosing too small springs)
min_total_preload_stage1 = 0;   % N (stage1 no idlers)
min_total_preload_stage2 = 80;  % N (total across all idlers on stage2)

dynamic_factor = 1.3;
make_plots = true;

%% ---------- CALCULATIONS ----------
p_m = p/1000;
C1_p = C1_mm / p;
C2_p = C2_mm / p;

ratio1 = N_stage1_driven / N_stage1_driver;
ratio2 = N_stage2_driven / N_stage2_driver;
ratio_total = ratio1 * ratio2;

% Pitch diameters
Dp1_driver = p / sind(180/N_stage1_driver);
Dp1_driven = p / sind(180/N_stage1_driven);
Dp2_driver = p / sind(180/N_stage2_driver);
Dp2_driven = p / sind(180/N_stage2_driven);

% Radii (m)
r1_driver = (Dp1_driver/1000)/2;
r1_driven = (Dp1_driven/1000)/2;
r2_driver = (Dp2_driver/1000)/2;
r2_driven = (Dp2_driven/1000)/2;

% Torques
T_int = Tm * ratio1 * eta1;
T_out = T_int * ratio2 * eta2;

% Conservative forces (driver vs driven) + dynamic factor
F1_driver = Tm / r1_driver;
F1_driven = T_int / r1_driven;
F_stage1 = max(F1_driver, F1_driven) * dynamic_factor;

F2_driver = T_int / r2_driver;
F2_driven = T_out / r2_driven;
F_stage2 = max(F2_driver, F2_driven) * dynamic_factor;

% Capacity and margins
tensile_workable = tensile_strength_chain / safety_factor_chain;
margin1_pct = (tensile_workable - F_stage1) / tensile_workable * 100;
margin2_pct = (tensile_workable - F_stage2) / tensile_workable * 100;

% Shaft diameters (approx torsion)
d_motor_mm = ( (16*Tm/(pi*tau_allow))^(1/3) )*1000;
d_intermediate_mm = ( (16*T_int/(pi*tau_allow))^(1/3) )*1000;
d_output_mm = ( (16*T_out/(pi*tau_allow))^(1/3) )*1000;

d_motor_suggest = pick_std(d_motor_mm);
d_intermediate_suggest = pick_std(d_intermediate_mm);
d_output_suggest = pick_std(d_output_mm);

% Chain lengths
Lp1 = chain_links_from_centres(N_stage1_driver, N_stage1_driven, C1_p);
L1_mm = Lp1 * p;
Lp2 = chain_links_from_centres(N_stage2_driver, N_stage2_driven, C2_p);
L2_mm = Lp2 * p;

% ----------------- IDLER SIZING (both idlers on stage-2) -----------------
% Stage-1: no sprung idlers
total_preload_stage1 = min_total_preload_stage1;   % 0 N
per_idler_preload_stage1 = 0;
k_idler_stage1 = 0;

% Stage-2: total preload (distributed across num_idlers_stage2)
if num_idlers_stage2 <= 0
    error('num_idlers_stage2 must be >= 1 when placing idlers on stage-2.');
end

total_preload_stage2 = max(min_total_preload_stage2, idler_preload_ratio * F_stage2); % N
per_idler_preload_stage2 = total_preload_stage2 / num_idlers_stage2; % each idler supplies this preload
k_idler_stage2 = per_idler_preload_stage2 / idler_travel;   % N/mm per idler

%% ---------- Display ----------
fprintf('--- 06B solution (both idlers on stage-2) ---\n');
fprintf('Total ratio = %.3g (%.3g x %.3g)\n', ratio_total, ratio1, ratio2);
fprintf('Pitch diameters (mm): %dT=%.1f, %dT=%.1f, %dT=%.1f, %dT=%.1f\n', ...
    N_stage1_driver, Dp1_driver, N_stage1_driven, Dp1_driven, N_stage2_driver, Dp2_driver, N_stage2_driven, Dp2_driven);
fprintf('Torques (N·m): motor=%.2f, interm=%.2f, out=%.2f\n', Tm, T_int, T_out);
fprintf('Design chain forces (dyn x%.2f): stage1=%.1f N, stage2=%.1f N\n', dynamic_factor, F_stage1, F_stage2);
fprintf('Chain workable (SF %.2f)= %.1f N => margins: s1=%.1f%% s2=%.1f%%\n', safety_factor_chain, tensile_workable, margin1_pct, margin2_pct);
fprintf('Chain lengths: stage1 %d links (%.1f mm), stage2 %d links (%.1f mm)\n', Lp1, L1_mm, Lp2, L2_mm);

fprintf('\nIdler configuration:\n');
fprintf('  Stage1: num_idlers = %d, total_preload = %.1f N, per-idler = %.1f N, k_per_idler = %.2f N/mm\n', ...
    num_idlers_stage1, total_preload_stage1, per_idler_preload_stage1, k_idler_stage1);
fprintf('  Stage2: num_idlers = %d, total_preload = %.1f N, per-idler = %.1f N, k_per_idler = %.2f N/mm\n', ...
    num_idlers_stage2, total_preload_stage2, per_idler_preload_stage2, k_idler_stage2);

fprintf('\nSuggested shaft diameters (mm): motor=%d, interm=%d, out=%d\n', d_motor_suggest, d_intermediate_suggest, d_output_suggest);

%% plot (optional)
if make_plots
    figure; Tm_vec = linspace(1,30,200);
    plot(Tm_vec./r1_driver * dynamic_factor); hold on;
    plot((Tm_vec*ratio1*eta1)./r2_driver * dynamic_factor);
    yline(tensile_workable,'k--'); legend('Stage1','Stage2','Chain workable');
    xlabel('Motor torque (N·m)'); ylabel('Chain force (N)'); grid on;
end

%% helpers
function Lp = chain_links_from_centres(N1,N2,C_p)
    if C_p <= 0, error('C_p must be > 0'); end
    Lp_calc = 2*C_p + (N1+N2)/2 + ((N2 - N1)^2)/(4*pi^2 * C_p);
    Lp = round(Lp_calc);
    if mod(Lp,2) ~= 0, Lp = Lp + 1; end
end

function s = pick_std(d_mm)
    stds = [6 8 10 12 14 15 16 18 20 22 24 25 28 30 32 35 38 40 45 50];
    idx = find(stds >= ceil(d_mm), 1, 'first');
    if isempty(idx), s = ceil(d_mm); else s = stds(idx); end
end
