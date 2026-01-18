function [dist_empty, voltage_final, time_total] = simulate_regen(decel_rate, target_speed_kmh, acceleration, cap_pre_charge)
    % INPUTS: decel_rate, target_speed_kmh, acceleration, cap_pre_charge
    % OUTPUT: dist_empty (Distance travelled until capacitor runs out)

    % --- FIXED CONSTANTS ---
    mass = 2400; c = 165; 
    initial_speed_kmh = 15; % Fixed braking entry speed
    
    wheelradius = 0.1; gearratio = 10; crr = 0.004;
    nominal_voltage = 48; nominal_rpm = 4000; pair_poles = 4;
    B = 1.0; k_h = 0.015; k_e = 0.0003; n = 1.6; m_fe = 7.6*0.35;
    r_circuit = (9.6e-3)/2; efficiency = 0.85;

    % --- SETUP ---
    initial_speed = initial_speed_kmh / 3.6; 
    cap_voltage = cap_pre_charge; % Start at the requested pre-charge
    kv = nominal_rpm/nominal_voltage; kt = 8.27/kv; ke = kt * (3/2);
    
    % --- PHASE 1: BRAKING ---
    dt = 0.01; speed_loop = initial_speed;
    f_braking = mass * decel_rate;
    t_motor = (f_braking * wheelradius) / gearratio;
    brakingcurrent = (t_motor/2) / kt;

    while speed_loop > 0.05
        wheelrpm = (speed_loop * 60) / (2 * pi * wheelradius);
        motor_w = (wheelrpm * gearratio) * (2*pi/60);
        v_motor = motor_w * ke - r_circuit*brakingcurrent;     
        f_e = pair_poles * motor_w / (2*pi);
        p_fe_total = 2 * (k_h * f_e * B^n * m_fe + k_e * f_e^2 * B^2 * m_fe);
        p_motor = v_motor * brakingcurrent * 2 - p_fe_total; 
        p_charging = p_motor * efficiency;
        cap_voltage = cap_voltage + ((p_charging / cap_voltage) * dt / c);
        speed_loop = speed_loop - (decel_rate * dt);
        
        if cap_voltage > 48, dist_empty=0; voltage_final=48; time_total=0; return; end
    end

    % --- PHASE 2: DRIVING ---
    dist_total = 0; time_total = 0;
    dist_empty = 0;
    has_reached_limit = false;

    target_speed = target_speed_kmh/3.6;
    velocity_drive = 0;
    
    % Forces
    force_accel = (mass*acceleration) + (mass*9.81*crr);
    current_accel = (((force_accel*wheelradius)/(gearratio*0.85))/2)/kt;
    force_cruise = mass*9.81*crr;
    current_cruise = (((force_cruise*wheelradius)/(gearratio*0.85))/2)/kt;

    while true
        has_energy = cap_voltage > cap_pre_charge;
        
        if velocity_drive < target_speed, mode=1; current_now=current_accel;
        else, mode=2; current_now=current_cruise; end
        
        if ~has_energy, mode=3; current_now=0; end

        if mode==3, force_push=0; 
        else, force_push = (2*(current_now*kt)*gearratio*0.85)/wheelradius; end
        
        force_resistive = (mass*9.81*crr) + (0.002*mass*9.81);
        velocity_drive = velocity_drive + ((force_push - force_resistive)/mass * dt);

        if velocity_drive > target_speed && mode~=3, velocity_drive=target_speed; end
        if velocity_drive <= 0.05 && mode==3, break; end

        dist_total = dist_total + velocity_drive*dt;
        time_total = time_total + dt;

        wheelrpm = (velocity_drive * 60) / (2 * pi * wheelradius);
        motor_w = (wheelrpm * gearratio) * (2*pi/60);
        v_bemf = motor_w * ke - r_circuit*current_now;
        
        if has_energy
            if (cap_voltage - v_bemf) <= 0, velocity_drive=velocity_drive*0.99; current_now=0; end
            p_fe = 2 * (k_h * (pair_poles*motor_w/(2*pi)) * B^n * m_fe);
            p_draw = ((v_bemf * current_now * 2) + p_fe) / 0.85;
            cap_voltage = cap_voltage - ((p_draw/cap_voltage)*dt/c);
            
            if cap_voltage <= cap_pre_charge && ~has_reached_limit
                dist_empty = dist_total;
                has_reached_limit = true;
            end
        end
    end
    
    if ~has_reached_limit, dist_empty = dist_total; end
    voltage_final = cap_voltage;
end