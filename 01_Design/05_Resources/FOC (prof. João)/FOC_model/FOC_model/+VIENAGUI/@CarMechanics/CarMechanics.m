% Copyright (c) 2019 F. Ferreira da Silva
%                    João F. P. Fernandes
%                    Bruno Tibério
%
% Permission is hereby granted, free of charge, to any person obtaining a
% copy of this software and associated documentation files (the
% "Software"), to deal in the Software without restriction, including
% without limitation the rights to use, copy, modify, merge, publish,
% distribute, sublicense, and/or sell copies of the Software, and to permit
% persons to whom the Software is furnished to do so, subject to the
% following conditions:
%
% The above copyright notice and this permission notice shall be included
% in all copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
% NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
% DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
% OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
% USE OR OTHER DEALINGS IN THE SOFTWARE.

classdef CarMechanics
    %CARMECHANICS car model to simulate its driving
    
    %--------------------------------------------------------------------------
    %-PROPERTIES---------------------------------------------------------------
    %--------------------------------------------------------------------------
    
    properties (SetAccess = private)
        %Drive Course
        roadCircuit
        roadAngle
        roadAngleTime_
        roadIncline
        roadInclineTime_
        targetVelocity
        targetVelocityTime_
        maxTime
        
    end
    
    properties (Constant)
        %Car Properties (FIAT Seicento)
        carMass = 700; %[kg]
        wheelBase = 2.200 %[m]
        frontArea = 1.508*1.420;  %[m^2]
        
        %Wheel
        wheelMass = 3; %[kg]
        wheelRadius = 0.28; %[m]
        
        rollFriction = 0.01;
        cornerStiff = 60; %not used
        
        %Air Drag
        dragCoeff = 0.33;
        airDensity = 1.2930; %[kg m^-3]
        %         airTemperature = 273; %[K]
        %         airPressure = 101325; %[Pa]
        %         airSpecificGasConst = 287.058; %[J kg^-1 K^-1]
        %         airDensity = airPressure/(airSpecificGasConst*airTemperature); %[kg m^-3]
        
        %Motor
        motor = VIENAGUI.InductionMachine;
    end
    
    properties (Dependent)
       wheelInertia
    end
    
    %--------------------------------------------------------------------------
    %-METHODS------------------------------------------------------------------
    %--------------------------------------------------------------------------
    
    methods
        %Constructor
        function newCarMech = CarMechanics()
            newCarMech.maxTime = 300;
            newCarMech.roadAngle = [0 0 3 3 0 0 3 3]*pi/180;
            newCarMech.roadAngleTime_ = [0 0.1 0.101 0.5 0.501 0.8 0.801 1]*newCarMech.maxTime;
            newCarMech.roadIncline = [0 0]*pi/180;
            newCarMech.roadInclineTime_ = [0 1]* newCarMech.maxTime;
            newCarMech.targetVelocity = [0 500 500 1000 -1000 -500 0];
            newCarMech.targetVelocityTime_ = [0 0.1 0.3 0.4 0.6 0.7 1] * newCarMech.maxTime;
        end
        
        function wheelInertia = get.wheelInertia(newCarMech)
            wheelInertia = 1/2 * newCarMech.wheelMass * newCarMech.wheelRadius^2; %[Kg m^2]
        end
        
        function simulate(newCarMech)
            
            maxTime = newCarMech.maxTime;
            t_sample = 1;
            
            roadAngle = newCarMech.roadAngle;
            roadAngleTime_ = newCarMech.roadAngleTime_;
            
            roadIncline = newCarMech.roadIncline;
            roadInclineTime_ = newCarMech.roadInclineTime_;
            
            targetVelocity = newCarMech.targetVelocity;
            targetVelocityTime_ = newCarMech.targetVelocityTime_;
            
            Motor = newCarMech.motor;
            
            wheelBase = newCarMech.wheelBase; %[m]
            frontArea = newCarMech.frontArea;  %[m^2]
            
            %Wheel
            wheelMass = newCarMech.wheelMass; %[kg]
            wheelRadius = newCarMech.wheelRadius; %[m]
            wheelInertia = newCarMech.wheelInertia; %[Kg m^2]
            rollFriction = newCarMech.rollFriction;
            cornerStiff = newCarMech.cornerStiff; %not used
            
            %Air Drag
            dragCoeff = newCarMech.dragCoeff;
            airDensity = newCarMech.airDensity; %[kg m^-2]
            
            VoF = 1;
            
            sim('full_model_car.slx');
        end
    end
end   
