function [Kt, Ke, R, kc, kf, invEff] = estimate_motor_constants(P_nom, V_nom, I_nom, T_nom, rpm_nom, eta_nom)


    % Derived quantity: Angular speed
    omega_nom = 2 * pi * rpm_nom / 60;  % rad/s

    % Torque constant
    Kt = T_nom / I_nom;

    % EMF constant
    Ke = V_nom / omega_nom;

    % Mechanical power output
    P_mech = eta_nom * P_nom;

 
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
    