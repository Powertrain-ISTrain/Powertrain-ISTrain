function [motor_torque,max_motor_speed,max_motor_power] = MotorParameters(f_wheel,wheel_radius,transmission_eff,G,adhesion_coeff,max_velocity)

%{
This function outputs the following:

* Max axle torque (N.m)(taking into account the ratio) 
* Max motor  speed (rpm)(taking into account the ratio)
* Max motor power (W)(taking into account the ratio)
* plot of the quantities above (with)
%}


%Calculating the  values

motor_torque=2*(f_wheel.*wheel_radius)./0.5./(transmission_eff.*G);


max_motor_speed=max_velocity.*60.*G./(pi*wheel_radius*2); %in rpm's

max_motor_power=(max_motor_speed).*motor_torque*((2*pi)/60);

%Printing the maximum values



%Plotting

figure;

subplot(2,2,1);
plot(adhesion_coeff, motor_torque);
title('Motor Torque vs adhesion coefficient');

subplot(2,2,2);
plot(adhesion_coeff,max_motor_power);
title('Motor Power vs adhesion coefficient');

grid on;


end