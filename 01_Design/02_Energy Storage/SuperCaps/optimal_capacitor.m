%% Charging efficiency and optimum pre-charge analysis
clear; clc;

% Energy recovered [J]
Erecov = 7.8e3; % 7.8 kJ

% Capacitor parameters (C in Farads, R in Ohms)
caps(1).name = '3x16V 58F in series (19.33F)';
caps(1).C = 19.33;
caps(1).R = 3 * 0.0029;

caps(2).name = '48V 83F';
caps(2).C = 83;
caps(2).R = 0.0022;

caps(3).name = '48V 165F';
caps(3).C = 165;
caps(3).R = 0.0011;

Vin_values = 40:0.5:48;

% --- Plot 1: Efficiency vs Vin with optimum Vc(0) ---
figure; hold on; grid on;
for k = 1:length(caps)
    C = caps(k).C;
    R = caps(k).R;
    
    tau = R * C;
    t = 5 * tau;
    
    eta_curve = zeros(size(Vin_values));
    
    for i = 1:length(Vin_values)
        Vin = Vin_values(i);
        
        % Optimum pre-charge voltage (Eq. from image)
        V0 = sqrt(Vin^2 - (2 * Erecov) / C);
        
        % Efficiency (Eq. 5.3.7)
        eta = ( V0 + Vin + (V0 - Vin) * exp(-t / tau) ) / ( 2 * Vin );
        eta_curve(i) = eta * 100; % %
    end
    
    plot(Vin_values, eta_curve, 'LineWidth', 2, 'DisplayName', caps(k).name);
end
xlabel('Input Voltage V_{in} (V)');
ylabel('Charging Efficiency (%)');
title('Charging Efficiency vs V_{in} with Optimum Pre-charge Voltage');
legend('Location', 'northwest');


%% Charge efficiency of 48 V 165 F module with variable Vfull and Vin (V0 optimal)
clear; clc;

% Supercap parameters
C = 165;             % [F]
R = 0.0011;          % [ohm]
Erecov = 7.8e3;      % [J] recovered energy per event

Vin_list = 40:1:48;      % Vin values to plot
dV = linspace(-3, 0, 31);% Vfull - Vin (negative up to 0)
colors = lines(numel(Vin_list));

figure; hold on; grid on;

for k = 1:numel(Vin_list)
    Vin = Vin_list(k);

    % Optimal pre-charge voltage for this Vin
    V0_opt = sqrt(Vin^2 - (2 * Erecov) / C);

    % Corresponding Vfull values (always <= Vin)
    Vfull = Vin + dV;

    % Time to reach Vfull from V0_opt using RC charging equation
    t_full = -R*C .* log( 1 - (Vfull - V0_opt) ./ (Vin - V0_opt) );

    % Efficiency (Eq. 5.3.7)
    eta = ( V0_opt + Vin + (V0_opt - Vin) .* exp(-t_full./(R*C)) ) ./ (2*Vin);

    plot(dV, 100*eta, 'LineWidth', 2, 'Color', colors(k,:), ...
        'DisplayName', sprintf('Vin = %d V', Vin));
end

xlabel('V_{full} - V_{in} (V)');            
ylabel('Efficiency (%)');
title('Charge efficiency of 48 V 165 F module vs V_{full} and V_{in} (V_0 optimal)');
legend('Location','northwest');
xlim([min(dV) 0]);

