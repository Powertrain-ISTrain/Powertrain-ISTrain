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

classdef MQTTController < mqttio.Mqtt
    
    %--------------------------------------------------------------------------
    %-PROPERTIES---------------------------------------------------------------
    %--------------------------------------------------------------------------
    
    properties (SetAccess = private)
    end
    
    properties
        
        %SINAMICS
        SIN_state
        SIN_targetVelocity
        SIN_vofVoltage
        SIN_vofFrequency
        SIN_vofVoltageMin
        SIN_EMCY
        SIN_velocity
        SIN_temperature
        SIN_currentSmoothed
        SIN_torqueSmoothed
        SIN_connected
        
        %EPOS
        EPOS_state
        EPOS_connected
        EPOS_PID
        EPOS_target_position
        EPOS_position
        EPOS_angle
        EPOS_MAX_FOLLOWING_ERROR
        EPOS_EMCY
        
        %CAN
        CAN_connected
        
        %Raspberry Pi
        RPI_connected
        
        %Log
        LOG
        
    end
    
    properties (Dependent)
    end
    
    %--------------------------------------------------------------------------
    %-METHODS------------------------------------------------------------------
    %--------------------------------------------------------------------------
    methods
        %Constructor
        function newMQTT = MQTTController(protocol,host,port,varargin)
            if nargin == 0
                protocol = 'ws';
                host = '192.168.31.56';
                port = 8080;
            end
            brokerAddress = [protocol '://' host];
            newMQTT@mqttio.Mqtt(brokerAddress,'ClientID','MATLAB','Port',port);

        end
        
        %Deconstructor
        function delete(newMQTT)
            newMQTT.unsubscribeAll
            newMQTT.disconnect
        end
        
        function newSub = MQTTSubscribe(newMQTT,Topic,QoS,handler)
            
            newSub = newMQTT.subscribe(Topic,'QoS',QoS,'Callback',handler);
        end
        
    end
    
    methods (Access = public)        
        function showMessage(newMQTT,topic,data)
            switch topic
                case char('VIENA/SINAMICS/state/read')
                    newMQTT.SIN_state = data; %MUDAR O TIPO DE DADOS DO SIN_state
                    
                case char('VIENA/SINAMICS/target_velocity/read')
                    newMQTT.SIN_targetVelocity = typecast(uint8(data.char),'int32'); %int32
                    
                case char('VIENA/SINAMICS/vof_voltage/read')
                    newMQTT.SIN_vofVoltage = typecast(uint8(data.char),'single'); %float
                    
                case char('VIENA/SINAMICS/vof_freq/read')
                    newMQTT.SIN_vofFrequency = typecast(uint8(data.char),'single'); %float
                    
                case char('VIENA/SINAMICS/vof_min/read')
                    newMQTT.SIN_vofVoltageMin = typecast(uint8(data.char),'single'); %float
                    
                case char('VIENA/SINAMICS/EMCY')
                    newMQTT.SIN_EMCY = data.char; %char
                    
                case char('VIENA/SINAMICS/velocity')
                    newMQTT.SIN_velocity = typecast(uint8(data.char),'int32'); %int32
                    
                case char('VIENA/SINAMICS/temperature')
                    newMQTT.SIN_temperature = typecast(uint8(data.char),'int32'); %int32 ver o que é depois
                    
                case char('VIENA/SINAMICS/current_smoothed')
                    newMQTT.SIN_currentSmoothed = typecast(uint8(data.char),'single'); %float
                    
                case char('VIENA/SINAMICS/torque_smoothed')
                   newMQTT.SIN_torqueSmoothed = typecast(uint8(data.char),'single'); %float
                    
                case char('VIENA/SINAMICS/connected')
                    newMQTT.SIN_connected = typecast(uint8(data.char),'uint8'); %uint8
                    
                case char('VIENA/General/canopen')
                    newMQTT.CAN_connected = typecast(uint8(data.char),'uint8'); %uint8
                    fprintf('CAN_connected - %d\n',newMQTT.CAN_connected)
                    
                case char('VIENA/General/rpi')
                    newMQTT.RPI_connected = typecast(uint8(data.char),'uint8'); %uint8
                    fprintf('RPI_connected - %d\n',newMQTT.RPI_connected)
                    
                case char('VIENA/General/log')
                    newMQTT.LOG = data;
                    fprintf('LOG - %s\n',newMQTT.LOG)
                    
                otherwise
                    warning('no topic of that name.')   
            end 

        end
    end   
end