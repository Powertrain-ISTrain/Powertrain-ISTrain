clc
clear all
close all

g=9.81;

% induction motor
pp=2;

%% Choose the track
op=menu('Select the track:','A->B','C->D->E->F');
switch op
    case 1
        map = textread(fullfile(pwd,'trackAB.txt'));
        Torque_required=150;
        load('wheels_speedAB.mat')
    case 2
        map = textread(fullfile(pwd,'trackCDEF.txt'));
        Torque_required=40;
        load('wheels_speedCDEF.mat')
    otherwise
        error('no track selected')
end


prompt = {'Car Additional Weight (700 Kg no passangers)','Enter the road angle'};
dlgtitle = 'Inputs';
dims = [1 35];
mechInput = inputdlg(prompt,dlgtitle,dims);

m = 700 + str2double(mechInput{1});
roadAngle = str2double(mechInput{2})*pi/180;


% Modelo FIAT Cinquecento
%% Car Parameters

carParameters = struct('Mass',m,...
    'frontalArea',2.1414,...
    'wheelRadius',0.1651,...
    'wheelInertia',0.25,...
    'motorInertia',0.0025,...
    'gearRatio',8,...
    'wheelFriction',0.01,...
    'dragCoefficient',0.33,...
    'airDensity',1.225,...
    'gravity',9.8,...
    'VoF',1);

V_F=carParameters.VoF;
Cd=carParameters.dragCoefficient; % FIAT Cinquecento wikipedia https://en.wikipedia.org/wiki/Fiat_Cinquecento
A= carParameters.frontalArea; % area from wikipedia
rw=0.1651;
lf=3.23/2;
lr=3.23/2;

% wheel
fr=carParameters.wheelFriction; % rolling resistance coeff.
m_wheel=3;
Iw=1/2*m_wheel*rw^2; % or m?

% parametros
rho=carParameters.airDensity;
%% inertia
totalInertia = carParameters.wheelInertia + (carParameters.motorInertia)*(carParameters.gearRatio)^2;
equivalentMass = ((carParameters.Mass) + (totalInertia/(carParameters.wheelRadius^2)));


%% old
w1=2200;
w2=6500;
w3=9000;
% v_v=[1 w1 w1*1.8 w2 w3 w3];
% v_v=[1 w1 w1*2 w2 w3 w3];
% v_t=[0 1.5 3 6 12 20];

v_v=wheels_speed.Data*8*60/(2*pi); % to rpm and x8 because of the gearbox
v_t=wheels_speed.Time; 
t_max=wheels_speed.Time(end);
t_sample=0.1;

% motor = VIENAGUI.InductionMachine;
% 
% EQSolutions = @(v,f,r) motor.EQSolutions(v,f,r);

%% follow path
% Create Map, Steering Controller and prepare Figures
Map = FIAT.CreateMap('waypoints',map(:,1:2));
close(Map.figure_handle);

frameStep = 24;
look_ahead = 2;
dt=0.01;

x0=Map.map_points(1,1);
y0=Map.map_points(1,2);
x2=Map.map_points(2,1);
y2=Map.map_points(2,2);
psi0 = atan2((x2-x0),(y2-y0));

wheel_base = 2.2;
wheel_angle = 0;

Steering = FIAT.SteeringController('method','pure pursuit');
Steering.path_look_ahead = look_ahead;
Steering.load_car(x0,y0,atan2((y2-y0),(x2-x0)));
Steering.load_map(Map.map_points);
Steering.sampling_time = dt;
endpoint = Steering.map_points(end,:);

ShowCar = FIAT.CarPlotter;
ShowCar.prepare_figure(Map.map_points, Steering.car_position, Steering.car_pose);

%% motor dq
%--------------------------------------------------------------------------
% Motor Parameters
%--------------------------------------------------------------------------
% base values
Vb = 76;
fb = 76;
wb = 2*pi*fb;
Pp = 2;
Ib = 160;
Imb = 120/sqrt(2);
% Equivalent Circuit Parameters
pp=2;
Rs = 8.56e-3;
ls = 0.06292e-3;
LM = 1.0122e-3;
Rr = 2*5.10e-3;
lr = 0.06709e-3;
% stator and rotor self inductances for dq model
LS = ls + LM;
LR = lr + LM;
% dq model Matrices
LMatrix = [LS 0 LM 0; 0 LS 0 LM; LM 0 LR 0; 0 LM 0 LR];
RMatrix = [Rs 0 0 0; 0 Rs 0 0; 0 0 Rr 0; 0 0 0 Rr];
RLInv = RMatrix/(LMatrix);

%--------------------------------------------------------------------------
% FOC
%--------------------------------------------------------------------------
ratedFluxPeak = sqrt(2)*Imb*LM;
maxTorque = 65;
maxID = sqrt(2)*Imb;
maxIQ = sqrt(2*Ib^2 - Imb^2);

%% sim

sim('VIENA_part4.slx')

hold on
plot(x_v0.Data,y_v0.Data)

%% resultados
figure(2)
subplot(1,2,1)
plot(wheels_speed.Time,wheels_speed.Data*8*60/(2*pi),'LineWidth',2) %linear speed
hold on
plot(rpm_motor.Time,rpm_motor.Data,'--','LineWidth',2)
format_2column
ylabel('Motor speed [rpm]')
legend('input','reference')

subplot(1,2,2)
plot(rpm_motor.Time,Torque_motor.Data,'LineWidth',2)
hold on
plot(rpm_motor.Time,Torque_motor.Data*8,'LineWidth',2)
format_2column
ylabel('Torque [Nm]')
legend('Motor Torque','Wheels Torque')


figure(3)
subplot(1,2,1)
yyaxis left
plot(Voltage_motor.V1.Time,Voltage_motor.V1.Data,'-r')
hold on
plot(Voltage_motor.V2.Time,Voltage_motor.V2.Data,'-b')
plot(Voltage_motor.V3.Time,Voltage_motor.V3.Data,'-g')
format_2column
title('Voltage [V]')
subplot(1,2,2)
plot(Current_motor.Time,Current_motor.Data,'LineWidth',2)

title('Current [A]')
