%% ADHESION VERIFICATION


%Description: This script checks the maximum limits for power, torque and
%accelration, for a given adhesion coefficient

%%

%References

%[1]- 2025 Railway Challenge Technical Specifications
%[2]- SouthHampton Report
%[3]- Powertrain Design report
%[4]-



%% FORCE CALCULATION

alpha = atan(max_gradient / 100); % gradient angle in radians

% Calculating the maximum values
M_wheel = (loco_mass * g)/driven_wheels; % weigth oa each wheel
f_wheel = adhesion_coeff .* M_wheel * cos(alpha); % Maxium force on each wheel
f_traction = driven_wheels .* f_wheel; % Total traction force
f_grade  = (loco_mass + max_trailing_mass) * g * sin(alpha);

f_drag = Rolling_coeff * (loco_mass+max_trailing_mass) * g * cos(alpha) ; % Total resistive force
f_motive = f_traction - f_drag-f_grade; % Total motive force

%% ACCELERATION CALCULATION
max_accel_loaded=f_motive/(loco_mass+max_trailing_mass); % maximum acceleration (when loaded)
time_15kmh=max_velocity./max_accel_loaded; % Time to maxium velocity under constant acceleration

% fazer para o caso em que é unloaded

%% TORQUE AND POWER CALCULATION

motor_torque=2*(f_wheel.*wheel_radius)./0.5./(transmission_eff.*G);
max_motor_speed=max_velocity.*60.*G./(pi*wheel_radius*2); %in rpm's
max_motor_power=(max_motor_speed).*motor_torque*((2*pi)/60);



%% TABLE

T = table( ...
    adhesion_coeff(:), ...
    f_wheel(:), ...
    f_traction(:), ...
    f_motive(:), ...
    max_accel_loaded(:), ...
    time_15kmh(:), ...
    motor_torque(:), ...
    max_motor_power(:), ...
    'VariableNames', { ...
      'Adhesion', ...
      'WheelForce_N', ...
      'TractionForce_N', ...
      'MotiveForce_N', ...
      'AccelLoaded_mps2', ...
      'TimeToMax_s', ...
      'MotorTorque_Nm', ...
      'MotorPower_W' ...
    } ...
);

disp('--- Summary Table of Key Quantities vs Adhesion Coefficient ---')
disp(T)



%% PLOTS

%forces

% plot(adhesion_coeff, f_wheel, 'r-', 'LineWidth', 2)
% hold on;
% plot(adhesion_coeff, f_traction, 'b--', 'LineWidth', 2)
% plot(adhesion_coeff, f_motive, 'g-.', 'LineWidth', 2)
% hold off;
% 
% legend('Wheel force', 'Traction force', 'Motive force')
% xlabel('Adhesion coefficient')
% ylabel('Force (N)')
% title('Forces vs Adhesion')
% grid on;


% accelerations

% figure;

% subplot(2,2,1);
% plot(adhesion_coeff, max_accel_loaded);
% title('max acceleration loaded (m/s^2)');
% 
%
% subplot(2,2,2);
% plot(adhesion_coeff, time_15kmh);
% title('time to get to 15km/h (s)');
% 
% grid on;


% Motor

% figure;
% 
% subplot(2,2,1);
% plot(adhesion_coeff, motor_torque);
% title('Motor Torque vs adhesion coefficient');
% 
% subplot(2,2,2);
% plot(adhesion_coeff,max_motor_power);
% title('Motor Power vs adhesion coefficient');
% 
% grid on;





