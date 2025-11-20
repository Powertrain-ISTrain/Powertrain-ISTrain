function [f_motive, f_wheel] = forces(max_gradient, loco_mass,max_trailing_mass, g, driven_wheels, ...
    adhesion_coeff, Rolling_coeff, Aerodynamic_drag)

%{
This function outputs the following:

* Max Motive force (overall driving force)
* Max wheel force (before slip)
* plot of motive force for all coefficients
* plot of wheel force for all coefficients
* plot of traction for all coefficents
%}


%----------------------------------------------------------


alpha = atan(max_gradient / 100); % gradient angle in radians

% Calculating the maximum values
M_wheel = (loco_mass * g)/driven_wheels;
f_wheel = adhesion_coeff .* M_wheel * cos(alpha);
f_traction = driven_wheels .* f_wheel;

f_drag = Rolling_coeff * (loco_mass) * g * cos(alpha) + Aerodynamic_drag; % Total resistive force
f_motive = f_traction - f_drag;


%Printing the maximum values


% Plotting
plot(adhesion_coeff, f_wheel, 'r-', 'LineWidth', 2)
hold on;
plot(adhesion_coeff, f_traction, 'b--', 'LineWidth', 2)
plot(adhesion_coeff, f_motive, 'g-.', 'LineWidth', 2)
hold off;

legend('Wheel force', 'Traction force', 'Motive force')
xlabel('Adhesion coefficient')
ylabel('Force (N)')
title('Forces vs Adhesion')
grid on;

end

