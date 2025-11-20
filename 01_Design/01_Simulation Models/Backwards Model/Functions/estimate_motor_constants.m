function [Kt, Ke, R, kc, kf, invEff] = estimate_motor_constants(P_nom, V_nom, I_nom, T_nom, rpm_nom, eta_nom)
% Estimate motor constants from nominal motor data
%
% INPUTS:
%   P_nom   - Nominal electrical power [W]
%   V_nom   - Nominal voltage [V]
%   I_nom   - Nominal current [A]
%   T_nom   - Nominal torque [Nm]
%   rpm_nom - Nominal speed [rpm]
%   eta_nom - Assumed nominal efficiency (e.g., 0.9 for 90%)
%
% OUTPUTS:
%   Kt      - Torque constant [Nm/A]
%   Ke      - EMF constant [Vs/rad]
%   R       - Estimated phase resistance [Ohm]
%   kc      - Core (iron) loss coefficient [W/(rad/s)^2]
%   kf      - Friction loss coefficient [W/(rad/s)]
%   invEff  - Inverse of efficiency (1/eta_nom)

    % Derived quantity: Angular speed
    omega_nom = 2 * pi * rpm_nom / 60;  % rad/s

    % Torque constant
    Kt = T_nom / I_nom;

    % EMF constant
    Ke = V_nom / omega_nom;

    % Mechanical power output
    P_mech = eta_nom * P_nom;

    % Estimate realistic phase resistance:
    % Assume copper losses are about 10-15% of total input power
    P_cu = 0.12 * P_nom;
    R = P_cu / (3 * I_nom^2);

    % Total losses
    P_loss = P_nom - P_mech;

    % Core + mechanical losses = total losses - copper losses
    P_other = max(P_loss - P_cu, 0);

    % Assume 75% iron losses, 25% mechanical losses
    P_core = 0.75 * P_other;
    P_fm   = 0.25 * P_other;

    % Loss coefficients
    kc = P_core / (omega_nom^2);
    kf = P_fm   / omega_nom;

    invEff = 0.95;
end
    