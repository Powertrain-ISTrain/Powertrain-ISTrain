% Copyright (c) 2018 F. Ferreira da Silva
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

classdef InductionMachine
    %INDUCTIONMACHINE general description of Induction Machine.
    %   Gives nameplate information as well as derived information from
    %   nameplate data.
    %
    %   The electrical parameters are from the steinmetz equivalent
    %   circuit
    
    
%--------------------------------------------------------------------------
%-PROPERTIES---------------------------------------------------------------
%--------------------------------------------------------------------------

    properties (SetAccess = private)
        %NAMEPLATE DATA
        
        nameplateData
        
        %Structure containing:
        %   Frequency
        %   Voltage
        %   Current
        %   Power
        %   Speed
        %   PowerFactor
        %   Torque
        %   maxSpeed
        %   etc...
    end
        
    properties
        %STEINMETZ EQUIVALENT CIRCUIT PARAMETERS
        
        EQCircuit
        
        %Structure containing:
        %   stator Resistance
        %   stator LeakageResistance
        %   Magnetic Resistance
        %   Magnetic Inductance
        %   Rotor Resistance
        %   Rotor Leakage Inductance  
    end
    
    properties (Dependent)
        polePair
    end

%--------------------------------------------------------------------------
%-METHODS------------------------------------------------------------------
%--------------------------------------------------------------------------
    methods
        %Constructor
        function newIM = InductionMachine(nameplateData,EQCircuit)
            switch nargin
                case 0
                    newIM.nameplateData = ...
                        struct('Frequency',76,...
                               'Voltage',76,...
                               'Current',157,...
                               'Power',15e3,...
                               'Speed',2200,...
                               'PowerFactor',0.85,...
                               'Torque',65,...
                               'maxSpeed',10e3);
                    newIM.EQCircuit = ...
                              [8.56e-3,...       %Rs
                               0.06292e-3,...    %Ls
                               inf,...           %RM
                               1.01220e-3,...    %LM
                               5.10e-3,...       %Rr
                               0.06709e-3];      %Lr
                case 1
                    %check if arguments are structures
                    flagStructND = isstruct(nameplateData);
                    assert(flagStructND,'Variable nameplateData not a structure.')

                    %check if nameplate data has at least the first 5 fields.
                    NDFieldNames = fieldnames(nameplateData);
                    flagFieldNamesND = ismember(NDFieldNames,...
                        {'Frequency','Voltage','Current','Power','Speed'});
                    assert(sum(flagFieldNamesND) == 5,...
                        'Structure nameplateData does not have one of the following fields: Frequency, Voltage, Current, Power or Speed.');
                    newIM.nameplateData = nameplateData;
                case 2
                    %check if arguments are structures and array
                    flagStructND = isstruct(nameplateData);
                    flagArrayEC = isnumeric(EQCircuit);
                    assert(flagStructND,'Variable nameplateData not a structure.')
                    assert(flagArrayEC,'Variable EQCircuit not an array.')
                    %check if nameplate data has at least the first 5 fields.
                    NDFieldNames = fieldnames(nameplateData);
                    flagFieldNamesND = ismember(NDFieldNames,...
                        {'Frequency','Voltage','Current','Power','Speed'});
                    assert(sum(flagFieldNamesND) == 5,...
                        'Structure nameplateData does not have one of the following fields: Frequency, Voltage, Current, Power or Speed.');
                    %check if EQCircuit is a 6-array
                    checkEQCDim = numel(EQCircuit);
                    assert(checkEQCDim == 6,...
                        'Missing %d element in variable EQCircuit',6-numel(EQCircuit))
                    newIM.nameplateData = nameplateData;
                    newIM.EQCircuit = EQCircuit;
                otherwise
                    error('exceeded number of inputs')
            end
        end
        
%--------------------------------------------------------------------------
%-OVERLOAD-----------------------------------------------------------------
%--------------------------------------------------------------------------
        
%-------DISPLAY------------------------------------------------------------
        function disp(newIM)  
            fieldsIM = fieldnames(newIM);
            if any(ismember(fieldsIM,'nameplateData'))
                fprintf(1,'--------------------------\n')
                fprintf(1,'Nameplate Data\n')
                fprintf(1,'--------------------------\n')
                for field = fieldnames(newIM.nameplateData)'
                    fprintf(1,[field{1} ':% g\n'],newIM.nameplateData.(field{1}));
                end   
            end
            if any(ismember(fieldsIM,'EQCircuit'))
                fprintf(1,'--------------------------\n')
                fprintf(1,'Equivalent Circuit Data\n')
                fprintf(1,'--------------------------\n')
                fieldName = {'Rs','Ls','RM','LM','Rr','Lr'};
                for field = 1:numel(fieldName)
                    fprintf(1,[fieldName{field} ':% g\n'],newIM.EQCircuit(field));
                end
                fprintf(1,'--------------------------\n\n')
            end
        end
%-------END-DISPLAY--------------------------------------------------------
                
%-------PLOT---------------------------------------------------------------
        function plot(newIM,varargin)
            %check how many input arguments are
            %set voltage, frequency and flag
            switch nargin
                case 1
                    voltage = newIM.nameplateData.Voltage;
                    frequency = newIM.nameplateData.Frequency;
                    FULL_FLAG = false;
                case 2
                    voltage = newIM.nameplateData.Voltage;
                    frequency = newIM.nameplateData.Frequency;
                    assert(strcmpi(varargin{1},'full'),'flag not known. Did you mean: full')
                    FULL_FLAG = true;
                case 3
                    voltage = varargin{1};
                    frequency = varargin{2};
                    FULL_FLAG = false;
                case 4
                    voltage = varargin{1};
                    frequency = varargin{2};
                    assert(strcmpi(varargin{3},'full'),'flag not known. Did you mean: full')
                    FULL_FLAG = true;
                otherwise
                    error('Too many input arguments')
            end
            %check if voltage and frequency are positive
            [voltage,frequency] = newIM.CheckVoltageFrequency(voltage,frequency);
            %calculate solutions for each rotor speed
            statorFreq = frequency/newIM.polePair;
            %make rotor speed array for plot
            numberOfPoints = 1000;
            switch frequency
                case 0
                    rotorSpeed = 0;
                otherwise
                    rotorSpeed = linspace(0,statorFreq,numberOfPoints)'*60;
            end
            %get solutions
            solutions = SweepSolutions(newIM,voltage,frequency,rotorSpeed);
            %figure
            newIM.plottorquecurrent(voltage,frequency,rotorSpeed,solutions);
            %figure with full_flag enabled
            if FULL_FLAG
                figure;
                newIM.subplotpowercurrent(voltage,frequency,rotorSpeed,solutions);
            end
        end
%-------END-PLOT-----------------------------------------------------------

%-------SURFACE------------------------------------------------------------
        function surface(newIM,varargin)
            %flags for different plots
            FLAG_STABLE = false;
            FLAG_STARTING = false;
            FLAG_MAX = false;
            %check how many input arguments and set voltage, frequency and flag
            switch nargin
                case 1
                    finalKvof = 1;
                    finalFrequency = newIM.nameplateData.Frequency*1.10;
                case 2
                    finalKvof = 1;
                    finalFrequency = newIM.nameplateData.Frequency*1.10;
                    switch lower(varargin{1})
                        case 'starting'
                            FLAG_STARTING = true;
                        case 'max'
                            FLAG_MAX = true;
                        case 'stable'
                            FLAG_STABLE = true;
                        otherwise
                            error('flag not known. use starting, max or stable.')
                    end
                case 3
                    finalKvof = varargin{1};
                    finalFrequency = varargin{2}*1.10;
                case 4
                    switch lower(varargin{3})
                        case 'starting'
                            FLAG_STARTING = true;
                        case 'max'
                            FLAG_MAX = true;
                        case 'stable'
                            FLAG_STABLE = true;
                        otherwise
                            error('flag not known. use starting, max or stable.')
                    end
                otherwise
                    error('Too many input arguments')
            end            
            %Check if voltage and scalar command are positive
            if ~newIM.CheckIfPositive(finalKvof,finalFrequency)
                warning('One of the inputs is negative. It was changed to positive.')
                finalKvof = abs(finalKvof);
                finalFrequency = abs(finalFrequency);
            end
            %Make meshgrid
            numberOfPoints = 1000;
            frequency = linspace(0,finalFrequency,numberOfPoints);
            %Check what kind of plot is
            if all(~[FLAG_STARTING FLAG_MAX])
                %Normal torque and efficiency map
                statorFreq = finalFrequency/newIM.polePair;
                rotorSpeed = linspace(0,statorFreq,numberOfPoints)*60;
                [freqMesh,rotorMesh] = meshgrid(frequency,rotorSpeed);
                %Compute Torque and Efficiency for each pair value of frequency and rotor speed
                Torque = NaN(size(freqMesh));
                eff = NaN(size(freqMesh));
                for iFreq = frequency
                    %Compute the voltage
                    voltage = finalKvof*iFreq;
                    %Only check solutions for which the rotorspeed is less than the stator frequency
                    iStatorFreq = iFreq/newIM.polePair*60;
                    auxRotorSpeed = rotorMesh(freqMesh == iFreq & rotorMesh <= iStatorFreq);
                    %Compute the solutions
                    solutions = SweepSolutions(newIM,voltage,iFreq,auxRotorSpeed);
                    Torque(freqMesh == iFreq & rotorMesh <= iStatorFreq) = solutions(:,4);
                    eff(freqMesh == iFreq & rotorMesh <= iStatorFreq) = solutions(:,end);
                end
                if FLAG_STABLE
                    [~,maximRotorSpeed] = newIM.getMaxTorque(finalKvof*frequency(1,:),frequency(1,:));
                    Torque(rotorMesh < maximRotorSpeed) = NaN;
                    eff(rotorMesh < maximRotorSpeed) = NaN;
                end
                    %make freq,rotorspeed,torque map
                    figure;
                    surf(freqMesh,rotorMesh,Torque,'EdgeColor','none')
                    %Labeling
                    grid minor
                    xlabel('Freq [Hz]')
                    ylabel('Speed [RPM]')
                    zlabel('Torque [Nm]')
                    title(['Torque Map, VoF = ' num2str(finalKvof)])
                    %Make freq,rotorspeed,efficiency map
                    newIM.ploteffmap(finalKvof,freqMesh,rotorMesh,eff);
            else
                Kvof = linspace(0,finalKvof*1.1,numberOfPoints);
                [freqMesh,KvofMesh] = meshgrid(frequency,Kvof);
                voltageMesh = freqMesh.*KvofMesh;
                %Used STARTING flag
                if FLAG_STARTING
                    startingTorque = newIM.getStartingTorque(voltageMesh,freqMesh);
                    figure;
                    surf(freqMesh,KvofMesh,startingTorque,'EdgeColor','none') 
                end
                %Used MAX flag
                if FLAG_MAX
                    maxTorque = newIM.getMaxTorque(voltageMesh,freqMesh);
                    figure;
                    surf(freqMesh,KvofMesh,maxTorque,'EdgeColor','none') 
                end
                %Labeling
                grid minor
                xlabel('Freq [Hz]')
                ylabel('V/F')
                zlabel('Torque [Nm]')
                title('Max Torque Map')
            end      
        end
%-------END-SURFACE--------------------------------------------------------

%--------------------------------------------------------------------------
%-SET METHODS--------------------------------------------------------------
%--------------------------------------------------------------------------

        function newIM = set.EQCircuit(newIM,EQCircuit)
            %check if EQCircuit is a strucutre
            flagStructEC = isnumeric(EQCircuit);
            assert(flagStructEC,'Variable EQCircuit not an array.')
            %check if EQCircuit is a 6-array
            checkEQCDim = numel(EQCircuit);
            assert(checkEQCDim == 6,'Missing %d element in variable EQCircuit',6-numel(EQCircuit))
            newIM.EQCircuit = EQCircuit;
        end
        
%--------------------------------------------------------------------------
%-GET METHODS--------------------------------------------------------------
%--------------------------------------------------------------------------
        
        %POLEPAIR
        function polePair = get.polePair(newIM)
            nomFrequencyRPM = newIM.nameplateData.Frequency*60;
            nomSpeed = newIM.nameplateData.Speed;
            polePair = round(nomFrequencyRPM/nomSpeed);
        end
        
%--------------------------------------------------------------------------
%-PUBLIC METHODS-----------------------------------------------------------
%--------------------------------------------------------------------------

%-------EQSOLUTIONS--------------------------------------------------------
        function solution = EQSolutions(newIM,voltage,frequency,rotorSpeed)
%         %Calculations using equivalent circuit (ADAPTED FROM ALEXANDRE MONTEIRO'S WORK)

            %if frequency is zero, the solutions are 0
            if frequency == 0
                solution = zeros(1,7);
                return
            end
            %make calculations
            angFreq = 2*pi*frequency;
            statorFreq = frequency*60/newIM.polePair;
            slip = (statorFreq - rotorSpeed)/(statorFreq);
            eqvC = newIM.EQCircuit;
            % eqvC(1) == Rs 
            % eqvC(2) == Ls
            % eqvC(3) == RM
            % eqvC(4) == LM
            % eqvC(5) == Rr
            % eqvC(6) == Lr
            %Compute Z Matrix of the EQCircuit
            % U = ZI
            % U = [Us Us  0]
            % I = [Is IM Ir]
            % Z = [Zs ZM  0
            %      Zs  0 Zr
            %      -1  1  1]
            Zs = eqvC(1)+1j*angFreq*eqvC(2);
            %check if RM exists or not
            if eqvC(3) == inf
                Zm = 1j*angFreq*eqvC(4);
            else
                Z1 = 1j*angFreq*eqvC(4);
                Z2 = eqvC(3);
                Zm = (Z1*Z2)/(Z1+Z2);
            end
            
            U = voltage/sqrt(3);
            switch slip
                % no load condition
                case 0
                    Z = Zs + Zm;
                    I = [U/Z;U/Z;0];
                    outputPower = 0;
                    Torque= 0;
                otherwise
                    Zr = eqvC(5)/slip+1j*angFreq*eqvC(6);
                    Zeq = Zs + (Zr*Zm)/(Zr+Zm);
                    Is = U/Zeq;
                    Ir = (U-Zs*(U/Zeq))/Zr;
                    Im = (U-Zs*(U/Zeq))/Zm;
                    I = [Is;Im;Ir];
                    outputPower = 3*(abs(I(3))^2)*eqvC(5)*(1-slip)/slip;
                    Torque = 3*(abs(I(3))^2)*eqvC(5)/(slip*(angFreq/newIM.polePair));
            end
            inputPower = real(3*U(1)*conj(I(1)));
            efficiency = outputPower/inputPower;
            solution = [abs(I(1)),...    %Is
                        abs(I(2)),...    %IM
                        abs(I(3)),...    %Ir
                        Torque,...       %T
                        inputPower,...   %Pin
                        outputPower,...  %Pout
                        efficiency];     %eff            
        end
%-------END-EQSOLUTIONS----------------------------------------------------
        
%-------GETROTORSPEED------------------------------------------------------
        function [solution, STABLE_FLAG] = getRotorSpeed(newIM,voltage,...
                frequency,prevRotorSpeed,loadTorque,varargin) %need to check negative values of rotor speed!!
            %check if voltage and frequency are positive
            [voltage,frequency] = newIM.CheckVoltageFrequency(voltage,frequency);
            %Pre-conditioned solution
            solution = [];
            STABLE_FLAG = 0;
            %check if there is a non-zero frequency
            if frequency == 0
                return
            end
            %get starting torque and rotorspeed maximizer
            [maxTorque, maximRotorSpeed] = newIM.getMaxTorque(voltage,frequency);
            %check if loadtorque is higher than maxtorque
            if loadTorque > maxTorque
                return
            end
            %convert to slip and make slip condition
            maximSlip = newIM.RotorSpeed2slip(frequency,maximRotorSpeed);
            prevSlip = newIM.RotorSpeed2slip(frequency,prevRotorSpeed);
            %initialize function handles to fsolve
            torqueHandle = newIM.TorqueFunctionHandle(voltage,frequency);
            objFunction = @(s) abs(torqueHandle(s) - loadTorque);
            %if in stable region, check intersection
            slipCondition = prevSlip < maximSlip;
            torqueDifference = torqueHandle(prevSlip) - loadTorque;
            if slipCondition || (torqueDifference > 0)
                loadSlip = fminbnd(objFunction,0,maximSlip,varargin{:});
                loadRotorSpeed = newIM.Slip2RotorSpeed(frequency,loadSlip);
            else
                return
            end
            if ~isnan(loadRotorSpeed)
                EQSolutions = newIM.EQSolutions(voltage,frequency,loadRotorSpeed);
                solution = [loadRotorSpeed, EQSolutions(:,1), EQSolutions(:,5)];
                STABLE_FLAG = 1;
            end
        end
%-------END-GETROTORSPEED--------------------------------------------------

%-------GETTORQUE----------------------------------------------------------
        function Torque = getTorque(newIM,voltage,frequency,rotorSpeed)
            tHandle = newIM.TorqueFunctionHandle(voltage,frequency);
            slip = newIM.RotorSpeed2slip(frequency,rotorSpeed);
            Torque = tHandle(slip);
        end
%-------END-GETTORQUE------------------------------------------------------

%-------GETSTARTINGTORQUE--------------------------------------------------
        function startingTorque = getStartingTorque(newIM,voltage,frequency)
            %initialize variables
            t = 2*pi;
            we = t*frequency;
            pp = newIM.polePair;
            ws = t*frequency/pp;
            eqc = newIM.EQCircuit;
            Rr = eqc(5);
            Xr = we*eqc(6);
            [VTH , ZTH] = newIM.getThevenin(voltage,frequency);
            Ir = abs(VTH./(ZTH + 1j.*Xr + Rr));
            startingTorque = 3./ws .* Ir.^2 .* Rr;
            startingTorque(frequency == 0) = 0;
        end
%-------END-GETSTARTINGTORQUE----------------------------------------------

%-------GETMAXTORQUE-------------------------------------------------------
        function [maxTorque, maximRotorSpeed] = getMaxTorque(newIM,voltage,frequency)
            %initialize variables
            t = 2*pi;
            we = t*frequency;
            pp = newIM.polePair;
            ws = t*frequency/pp;
            %get equivalent circuit parameters
            eqc = newIM.EQCircuit;
            Rr = eqc(5);
            Xr = we*eqc(6);
            %get thevenin equivalent
            [VTH , ZTH] = newIM.getThevenin(voltage,frequency);
            RTH = real(ZTH);
            XTH = imag(ZTH);
            %compute max torque
            maxTorque = 3./(2*ws).*abs(VTH).^2*1./(RTH+sqrt(RTH.^2+(XTH+Xr).^2));
            maxTorque(frequency == 0) = 0;
            %compute dlip maximizer
            maximSlip = Rr*1./sqrt(RTH.^2+(XTH+Xr).^2);
            maximSlip(frequency == 0) = 0;
            %convert to rotor speed
            maximRotorSpeed = newIM.Slip2RotorSpeed(frequency,maximSlip);
        end
%-------END-GETMAXTORQUE---------------------------------------------------

    end
%--------------------------------------------------------------------------
%-HIDDEN METHODS-----------------------------------------------------------
%--------------------------------------------------------------------------
    methods (Hidden = true)

%-------GETVALUES4APP------------------------------------------------------
        function appValues = getValues4App(newIM,voltage,frequency)
            %check if voltage and frequency are positive
            [voltage,frequency] = newIM.CheckVoltageFrequency(voltage,frequency);
            %calculate solutions for each rotor speed
            statorFreq = frequency/newIM.polePair;
            %make rotor speed array for plot
            numberOfPoints = 1000;
            switch frequency
                case 0
                    rotorSpeed = 0;
                otherwise
                    rotorSpeed = linspace(0,statorFreq,numberOfPoints)*60;
            end
            %get solutions
            solutions = SweepSolutions(newIM,voltage,frequency,rotorSpeed);
            appValues = [solutions,rotorSpeed'];
        end
%-------END-GETVALUES4APP--------------------------------------------------

%-------DEBUGEQSOLUTIONS----------------------------------------------------
        function solution = debug_EQSolutions(newIM,voltage,frequency,rotorSpeed)
        %Calculations using equivalent circuit (ADAPTED FROM ALEXANDRE MONTEIRO'S WORK)
            %check if EQCircuit exists
            fieldsIM = fieldnames(newIM);
            assert(any(ismember(fieldsIM,'EQCircuit')),...
                'equivalent circuit parameters are needed (missing EQCircuit structure')
            %check if operationValues are not empty and are numeric
            assert(all(isnumeric([voltage frequency rotorSpeed])),...
                'Operation Values are not numeric.')
            %if frequency is zero, the solutions are 0
            if frequency == 0
                solution = zeros(1,7);
                return
            end
            %make calculations
            angFreq = 2*pi*frequency;
            statorFreq = frequency*60/newIM.polePair;
            slip = (statorFreq - rotorSpeed)/(statorFreq);
            eqvC = newIM.EQCircuit;
            % eqvC(1) == Rs 
            % eqvC(2) == Ls
            % eqvC(3) == RM
            % eqvC(4) == LM
            % eqvC(5) == Rr
            % eqvC(6) == Lr
            %Compute Z Matrix of the EQCircuit
            % U = ZI
            % U = [Us Us  0]
            % I = [Is IM Ir]
            % Z = [Zs ZM  0
            %      Zs  0 Zr
            %      -1  1  1]
            Zs = eqvC(1)+1j*angFreq*eqvC(2);
            %check if RM exists or not
            if eqvC(3) == inf
                Zm = 1j*angFreq*eqvC(4);
            else
                Z1 = 1j*angFreq*eqvC(4);
                Z2 = eqvC(3);
                Zm = (Z1*Z2)/(Z1+Z2);
            end
            switch slip
                % no load condition
                case 0
                    U = voltage/sqrt(3);
                    Z = Zs + Zm;
                    I = [Z\U;Z\U;0];
%                     outputPower = 0;
%                     Torque= 0;
                otherwise
                    U = [voltage voltage 0]/sqrt(3);
                    Zr = eqvC(5)/slip+1j*angFreq*eqvC(6);
                    Z = [Zs     Zm      0;
                         Zs     0       Zr;
                         -1     1       1];
                    I = Z\U';
%                     outputPower = 3*(abs(I(3))^2)*eqvC(5)*(1-slip)/slip;
%                     Torque = 3*(abs(I(3))^2)*eqvC(5)/(slip*(angFreq/newIM.polePair));
            end
%             inputPower = real(3*U(1)*conj(I(1)));
%             efficiency = outputPower/inputPower;
            solution = [I(1),...    %Is
                        I(2),...    %IM
                        I(3),...    %Ir
                        I(2)*Zm,... %Um
                        U-I(2)*Zm]; %Urs
        end
%-------END-DEBUGEQSOLUTIONS-----------------------------------------------

    end
%--------------------------------------------------------------------------
%-PRIVATE METHODS----------------------------------------------------------
%--------------------------------------------------------------------------
    
    methods (Access = private)
        
%-------CHECKIFPOSITIVE----------------------------------------------------
        function bool = CheckIfPositive(~,varargin)
            if all([varargin{:}] >=0)
                bool = true;
            else
                bool = false;
            end
        end
%-------END-CHECKIFPOSITIVE------------------------------------------------

%-------CHECKVOLTAGEFREQUENCY----------------------------------------------
        function [v,f] = CheckVoltageFrequency(newIM,voltage,frequency)
            v = voltage;
            f = frequency;
            if ~newIM.CheckIfPositive(frequency)
                warning('frequency negative. changing it to positive')
                f = abs(frequency);
            end
            if ~newIM.CheckIfPositive(voltage)
                warning('voltage negative. changing it to positive')
                v = abs(voltage);
            end
        end
%-------END-CHECKIFPOSITIVE------------------------------------------------

%-------SWEEPSOLUTIONS-----------------------------------------------------
        function solutions = SweepSolutions(newIM,voltage,frequency,rotorSpeed)
            solutions = NaN(numel(rotorSpeed),7);
            for k = 1:numel(rotorSpeed)
                solutions(k,:) = EQSolutions(newIM,voltage,frequency,rotorSpeed(k));
            end
        end
%-------END-SWEEPSOLUTIONS-------------------------------------------------

%-------SLIP2ROTORSPEED----------------------------------------------------
        function rotorSpeed = Slip2RotorSpeed(newIM,frequency,slip)
            pp = newIM.polePair;
            statorSpeed = frequency*60/pp;
            rotorSpeed = (1-slip).*statorSpeed;
        end
%-------END-SLIP2ROTORSPEED------------------------------------------------

%-------SLIP2ROTORSPEED----------------------------------------------------
        function slip = RotorSpeed2slip(newIM,frequency,RotorSpeed)
            pp = newIM.polePair;
            statorSpeed = frequency*60/pp;
            slip = 1 - RotorSpeed./statorSpeed;
        end
%-------END-SLIP2ROTORSPEED------------------------------------------------
    
%-------GETTHEVENIN--------------------------------------------------------
        function [VTH,ZTH] = getThevenin(newIM,voltage,frequency)
            %parameter initialization
            v = voltage/sqrt(3);
            we = 2*pi.*frequency;
            %get equivalent circuit parameters
            eqc = newIM.EQCircuit;           
            Rs = eqc(1);
            Xs = we*eqc(2);
            XM = we*eqc(4);
            %compute thevenin equivalent circuit
            VTH = v.*1j.*XM./(Rs + 1j.*(Xs + XM));
            ZTH = 1j.*XM.*(Rs + 1j.*Xs)./(Rs + 1j.*(Xs + XM));
        end
%-------END-GETTHEVENIN----------------------------------------------------
 
%-------GETTORQUEFUNCTIONHANDLE--------------------------------------------
        function TorqueHandle = TorqueFunctionHandle(newIM,voltage,frequency)
            %check if voltage and frequency are positive
            [voltage,frequency] = newIM.CheckVoltageFrequency(voltage,frequency);
            %initialize variables
            t = 2*pi;
            we = t*frequency;
            pp = newIM.polePair;
            ws = t*frequency/pp;
            %get equivalent circuit parameters
            eqc = newIM.EQCircuit;
            Rr = eqc(5);
            Xr = we*eqc(6);
            %compute thevenin equivalent circuit
            [VTH , ZTH] = newIM.getThevenin(voltage,frequency);
            %get torque handle function
            Ir = @(s) abs(VTH/(ZTH + 1j*Xr + Rr/s));
            TorqueHandle = @(s) 3/ws * (Ir(s))^2 * Rr/s;
        end
%-------END-GETTORQUEFUNCTIONHANDLE----------------------------------------

%-------PLOTEFFMAP---------------------------------------------------------
        function ploteffmap(~,Kvof,freqMesh,rotorMesh,eff)
            %make freq,rotorspeed,efficiency map
            figure;
            surf(freqMesh,rotorMesh,eff,'EdgeColor','none')
            hold on
            %contour map
            contour3(freqMesh,rotorMesh,eff,'ShowText','on','LineColor','k');
            %labeling
            grid minor
            xlabel('Freq [Hz]')
            ylabel('Speed [RPM]')
            zlabel('%')
            title(['Efficiency Map, VoF = ' num2str(Kvof)])
        end
%-------END-PLOTEFFMAP-----------------------------------------------------

%-------PLOTTORQUECURRENT--------------------------------------------------
        function plottorquecurrent(~,voltage,frequency,rotorSpeed,...
                EQCSolutions)
            %leftAxis
            yyaxis left
            plot(rotorSpeed',EQCSolutions(:,4));
            ylabel('Torque [Nm]')
            %rightAxis
            yyaxis right
            plot(rotorSpeed',EQCSolutions(:,1),'r');
            ylabel('Stator Current [A]')
            %general labeling
            grid minor
            xlabel('speed [RPM]')
            titleString = sprintf('U = %g [V], F = %g [Hz], V/F = %0.2g',...
                voltage,frequency, voltage/frequency);
            title(titleString)
        end
%-------END-PLOTTORQUECURRENT----------------------------------------------

%-------SUBPLOTPOWERCURRENT------------------------------------------------
        function subplotpowercurrent(~,voltage,frequency,rotorSpeed,EQCSolutions)
            [maxEff,maxIndex] = max(EQCSolutions(:,end));
            ax(1) = subplot(2,3,1:3);
            ax(2) = subplot(2,3,4);
            ax(3) = subplot(2,3,5);
            ax(4) = subplot(2,3,6);
            % Rotor speed vs. (POut,Eff)
            yyaxis(ax(1),'left')
            plot(ax(1),rotorSpeed,EQCSolutions(:,6)*1e-3)
            ylabel(ax(1),'Output Power [kW]')
            yyaxis(ax(1),'right')
            plot(ax(1),rotorSpeed,EQCSolutions(:,end),rotorSpeed(maxIndex),maxEff,'*k')
            ylabel(ax(1),'eff [%]')
            %Rotor speed vs. (Is,IM,Ir)
            plot(ax(2),rotorSpeed,EQCSolutions(:,1))
            ylabel(ax(2),'Stator Current [A]')
            plot(ax(3),rotorSpeed,EQCSolutions(:,2),rotorSpeed(maxIndex),EQCSolutions(maxIndex,2),'*k')
            ylabel(ax(3),'Mag. Current [A]')
            plot(ax(4),rotorSpeed,EQCSolutions(:,3))
            ylabel(ax(4),'Rotor Current [A]')
            for aHandle = ax
                xlabel(aHandle,'Speed [RPM]')
                grid(aHandle,'minor')
            end
            titleString = sprintf('U = %g [V], F = %g [Hz], V/F = %0.2g',voltage/frequency, voltage/frequency);
            title(ax(1),titleString)
        end
%-------END-SUBPLOTPOWERCURRENT--------------------------------------------
    end    
end

