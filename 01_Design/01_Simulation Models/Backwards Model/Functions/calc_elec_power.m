function [P_elec,P_out, I_phase,E]  = calc_elec_power(T, omega, Kt,Ke,R,kc,kf)
  %This function calculates the electrical power needed for a given
  %mechanical power and motor parameters.

  % Phase current from torque
  I_phase   = T / Kt;

  % Back-EMF
  E = Ke * omega;

  % Copper losses
  P_cu = 3 * R * I_phase^2;

  % Core losses 
  P_core = kc * omega^2;

  % Friction/windage 
  P_fm   = kf * omega;

  % Electrical input power
  P_elec = T*omega + P_cu + P_core + P_fm;
  P_out  = T*omega;
end