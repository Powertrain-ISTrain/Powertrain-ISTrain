function [max_accel_loaded,max_accel_unloaded,time_15kmh] = accelerations(f_motive,adhesion_coeff,loco_mass,max_trailing_mass, max_velocity)


%{
This function outputs the following:

* Max acceleration unloaded 
* Max acceleration loaded
* Time that it takes to get to 15km/h
* plot of the quantities above
%}


%Calculating the  values
max_accel_loaded=f_motive/(loco_mass+max_trailing_mass);
max_accel_unloaded=f_motive/(loco_mass);
time_15kmh=max_velocity./max_accel_loaded;

%Pritinng the maximum values




%Plotting the results

figure;

subplot(2,2,1);
plot(adhesion_coeff, max_accel_loaded);
title('max acceleration loaded (m/s^2)');

subplot(2,2,2);
plot(adhesion_coeff,max_accel_unloaded);
title('max  acceleration unloaded (m/s^2)');

subplot(2,2,3);
plot(adhesion_coeff, time_15kmh);
title('time to get to 15km/h (s)');

grid on;

end