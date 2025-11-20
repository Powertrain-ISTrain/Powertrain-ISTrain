% Copyright (c) 2019 J. Sequeira
%                    Bruno Tibério
%                    F. Ferreira da Silva
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
%
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAR TRANSVERSAL CONTROL - STEERING ANGLE CONTROL

classdef SteeringController < handle
    properties
        %------------------------------------------------------------------
        % Constants section
        %------------------------------------------------------------------
        debug;
        % distance between axis
        wheel_base = 2.2;
        % look ahead number of points;
        path_look_ahead = 3;
        window_start = [];
        window_end = [];
        %------------------------------------------------------------------
        % state vector containing pose of vehicle
        %------------------------------------------------------------------
        car_position = [];
        car_pose=[];
        path_index = [];
        % sampling time
        % this value is important for stability
        % e.g., check the funny effects when h=0.1
        sampling_time = 0.01;
        sampling_spacer = 0.001;
        % wheel angle
        last_wheel_angle = 0;
        % wheel angle velociy hard limiter
        ws_limiter = deg2rad(45); %0.3; 10.9;
        % max steering angle
        max_angle = deg2rad(28);
        % selection of method. 
        method = 'pure pursuit';
        map_points = [];
        num_points = [];
        % wheel velocity proportional gain;
        Kp_ws = 4;
        Kd_ws = 10;
    end
    properties(SetAccess = private)
        %------------------------------------------------------------------
        % figure handles for rapid plotting and updating
        %------------------------------------------------------------------
        figure_handle;
        car_position_handle;
        car_direction_handle;
        min_error_handle;
        look_ahead_handle;  
    end
    
    methods
        %% Public methods
        function obj = SteeringController(varargin)
            % Constructor
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SampleTime'), obj.sampling_time = varargin{i+1};
                elseif strcmp(varargin{i}, 'maxAngle'), obj.max_angle = deg2rad(varargin{i+1});
                elseif strcmp(varargin{i}, 'wheelBase'), obj.wheel_base = varargin{i+1};
                elseif strcmp(varargin{i}, 'method'), obj.method = varargin{i+1};
                else
                    error('Invalid argument');
                end
            end
		end
            
        function angle = clip_angle(obj, angle)
            %
            % Verify if the hard limits for max angle are atained
            % If so, limit it to plus minus max angle
            % :param angle:
            % :return: Angle in radians
            %
            if (abs(angle) > obj.max_angle)
                angle = sign(angle)*obj.max_angle;
            end
        end
        function speed = clip_speed(obj, speed)
            %
            % clip speed between max values.
            % :param speed:
            % :return: speed in radians
            %
            speed= sign(speed)*min(abs(speed), obj.ws_limiter);
        end
        
        function create_map(obj, varargin)
            for i = 2:2:nargin
                if  strcmp(varargin{i}, 'waypoints'), waypoints = varargin{i+1};
                elseif strcmp(varargin{i}, 'method'), interp_method = varargin{i+1};
                else
                    error('Invalid argument');
                end
            end
            %--------------------------------------------------------------
            % check if interp methods was supplied or select default
            %--------------------------------------------------------------
            if(~exist('interp_method','var'))
                interp_method = 'spline';
            end
            %--------------------------------------------------------------
            % check if figure is still active or not
            %--------------------------------------------------------------
            if isempty(obj.figure_handle) || ~isvalid(obj.figure_handle)
                % create a new one
                obj.figure_handle = figure();
            else
                % bring to front and clear it
                figure(obj.figure_handle);
                clf(obj.figure_handle);
            end
            %--------------------------------------------------------------
            % check if waypoints were supplied or create new waypoints
            %--------------------------------------------------------------
            if(~exist('waypoints','var'))
                %--------------------------------------------------------------------------
                % define the reference trajectory using mouse and cubic interpolation
                %--------------------------------------------------------------------------
                % "real" trajectories are hard to project without axis equal.
                ax_handle = newplot(obj.figure_handle);
                axis equal;
                axis([-200,200,-200,200]);
                % necessary to not reset axis config
                ax_handle.NextPlot = 'add';
                ax_handle.ColorOrderIndex=2;
                disp('use the mouse to input via points for the reference trajectory');
                disp('Last point using right click');
                button = 1;
                % number of points used
                point_index = 1;
                
                while button==1
                    [x(point_index),y(point_index),button] = ginput(1);
                    if point_index == 1
                        plot_handle = plot(ax_handle, x, y, '+');
                    else
                        plot_handle.XData = x;
                        plot_handle.YData = y;
                        drawnow;
                    end
                    point_index = point_index + 1;
                end
                disp([ num2str(point_index-1), ' points to interpolate from '])
                circuit_index = 0:0.01:point_index-2;
                xx = interp1(0:point_index-2, x, circuit_index, interp_method);
                yy = interp1(0:point_index-2, y, circuit_index, interp_method);
                
                % revert to normal red line plot without markers
                plot_handle.XData = xx;
                plot_handle.YData = yy;
                plot_handle.Marker = 'none'; 
                plot_handle.LineStyle = '-';
                drawnow;
                plot(x(1), y(1), 'Marker','s', 'MarkerFaceColor',[0.9290    0.6940    0.1250]);
                plot(x(end), y(end), 'Marker','s', 'MarkerFaceColor',[0.4660    0.6740    0.1880] );
                legend('Ref', 'Start', 'Stop', 'location','bestoutside');
                obj.map_points= [xx' yy'];
                obj.num_points= length(obj.map_points);
                
            else
                circuit_index = 0:0.01:length(wayponts)-1;
                xx = interp1(0:length(wayponts)-1, waypoints(1,:), circuit_index, interp_method);
                yy = interp1(0:length(wayponts)-1, waypoints(2,:), circuit_index, interp_method);
                % "real" trajectories are hard to project without axis equal.
                ax_handle = newplot(obj.figure_handle);
                axis equal;
                % necessary to not reset axis config
                ax_handle.NextPlot = 'add';
                ax_handle.ColorOrderIndex=2;
                plot_handle = plot(xx, yy);
                plot(waypoints(1,1), waypoints(1,2), 'Marker','s', 'MarkerFaceColor',[0.9290    0.6940    0.1250]);
                plot(waypoints(end,1), waypoints(end,2), 'Marker','s', 'MarkerFaceColor',[0.4660    0.6740    0.1880] );
                legend('Ref', 'Start', 'Stop', 'location','bestoutside');
                obj.map_points= [xx' yy'];
                obj.num_points= length(obj.map_points);
            end
        end
        
        function find_min(obj)
            if isempty(obj.path_index)
                [~, index] = min(vecnorm(obj.map_points-repmat(obj.car_position,obj.num_points,1),2,2));
                obj.path_index = index;
            else
                
                [~, index] = min(vecnorm(obj.map_points(obj.window_start:obj.window_end,:)-repmat(obj.car_position,obj.path_look_ahead+1,1),2,2));
                obj.path_index = index;
            end
            obj.window_start = obj.path_index;
            obj.window_end = min(obj.path_index+obj.path_look_ahead, obj.num_points);
        end
        
        function new_wheel_angle = update(obj)
            % for easy understanding
            psi = obj.car_pose(1);
            direction_vector= [-sin(psi), cos(psi)];
            obj.find_min();
            direction_look_ahead = [obj.map_points(obj.window_end,:)- obj.car_position];
            angle_look_ahead = atan2(direction_look_ahead(2),direction_look_ahead(1));
            angle_car = atan2(direction_vector(2),direction_vector(1));
            angle_diff = (angle_look_ahead-angle_car);
            angle_diff = obj.normalize_angle(angle_diff);
            
            % steering wheel speed
            ws =  + obj.Kp_ws*angle_diff;
            % limit max velocity
            ws = obj.clip_speed(ws);
            new_wheel_angle = obj.last_wheel_angle + obj.sampling_time*ws;
            new_wheel_angle = obj.clip_angle(new_wheel_angle);
            obj.last_wheel_angle = new_wheel_angle;
        end
        function update_car_readings(obj, car, psi)
            obj.car_position = car;
            obj.car_pose = psi;
        end
        function update_plots(obj)
            if isempty(obj.car_position_handle)
                obj.car_position_handle = plot(obj.car_position(1),...
                    obj.car_position(2), '+');
                obj.look_ahead_handle = plot(obj.map_points(obj.window_end, 1),...
                    obj.map_points(obj.window_end, 2), '+','Color',[0.5 0.5 0.5]);
                obj.car_direction_handle = quiver(obj.car_position(1),...
                    obj.car_position(2),...
                    -sin(obj.car_pose(1)),...
                    cos(obj.car_pose(1)),...
                    5, 'Color','b');
            else
                % update car position
                obj.car_position_handle.XData = obj.car_position(1);
                obj.car_position_handle.YData = obj.car_position(2);
                % update look_ahead point
                obj.look_ahead_handle.XData = obj.map_points(obj.window_end, 1);
                obj.look_ahead_handle.YData = obj.map_points(obj.window_end, 2);
                % update car direction vector
                obj.car_direction_handle.XData = obj.car_position(1);
                obj.car_direction_handle.YData = obj.car_position(2);
                obj.car_direction_handle.UData = -sin(obj.car_pose(1));
                obj.car_direction_handle.VData = cos(obj.car_pose(1));
                drawnow;
            end
        end
        
    end
    methods(Static) 
        function angle = normalize_angle(angle)
            %
            % Normalize an angle to [-pi, pi].
            % :param angle:
            % :return: Angle in radian in [-pi, pi]
            %
            while angle > pi
                angle = angle - 2 * pi;
            end 
            while angle < -pi
                angle = angle + 2 * pi;
            end       
        end
    end
end


% 
% %[xx, yy] = test_mapper();
% % compute the derivative of the reference trajectory
% dot_xx = diff(xx')/sampling_spacer;
% dot_yy = diff(yy')/sampling_spacer;
% psi = atan2(dot_yy,dot_xx);
% psi = [psi; psi(end)];    % just repeat the last point
% dot_psi = diff(psi)/sampling_spacer;
% dot_phi = zeros(size(dot_psi));
% 
% q_ref     = [xx', yy', psi, zeros(size(psi))];
% 
% figure(1);
% clf
% plot( q_ref(:,1),q_ref(:,2),'r');
% hold on
% axis equal;
% % draw initial point
% plot(q_ref(1,1),q_ref(1,2),'ro')
% % draw destination point
% plot(q_ref(end,1),q_ref(end,2),'r+')
% xlabel('x (m)');
% ylabel('y (m)');
% title('reference (red) and trajectory (blue) in the plane');
% 
% %%
% % initial pose of the vehicle given by mouse
% disp('input the initial position for the car')
% [x,y] = ginput(1);
% pHandle = plot(x,y,'b*');
% drawnow
% [x2,y2] = ginput(1);
% directionVector = [x2-x,y2-y];
% directionVector=directionVector/norm(directionVector);
% % plot vector of car direction
% pHandle_car_vec = quiver(x,y,directionVector(1),directionVector(2), 10, 'color',[0,0,0]);
% % plot vector of error
% pHandle_error_vec = quiver(x,y,directionVector(1),directionVector(2), 10, 'color', [0.5 0.5 0.5]);
% pHandle_min_error_point = plot(x,y,'+', 'color',[0.5 0.5 0.5]);
% q(1,:) = [x, y, acos(dot(directionVector,[0 1])), 0];
% 
% % initial velocity
% dot_q(1,:)      = [0, 0, 0, 0];
% 
% 
% %--------------------------------------------------------------------------
% % Find minimum lateral distance between initial position and reference
% % trajectory
% %--------------------------------------------------------------------------
% path_number_points = length(q_ref(:,1));
% [~, path_index] = min(vecnorm(q_ref(:,1:2)-repmat(q(1,1:2),path_number_points,1),2,2));
% 
% % get the first window for look ahead distance
% window = path_index:path_index+path_look_ahead;
% car_index = 1;
% last_delta_angle = 0;
% 
% % Control constants
%  Kl_ws = 0;
%  Kp_ws = 4;
%  Kd_ws = 10;
%  min_index = 1;
%  
%  
% while(1)
%     %----------------------------------------------------------------------
%     % get the point in the reference trajectory that is nearest to the
%     % current car position but use only a small window around the current
%     % time )(otherwise it cant deal with loops)
%     %----------------------------------------------------------------------
%     
%     % reference  value for the linear velocity
%     if 1
%         v(car_index) = 0.5;
%     else
%         v(car_index) = norm(error_q(1:2));
%         if abs(v(car_index))>0.5
%             v(car_index) = 0.5*sign(v(car_index));
%         end
%     end
%     
%     
%     % derivative term
%     dot_delta_angle = (angle_diff - last_delta_angle) / sampling_time;
%     last_delta_angle = angle_diff;
%     
%    
%     
%     % the control law is formed by additive contributions
%     lateral_distance = 0;
%     % car on the left side of the ref traj ??? wrong. If true it would
%     % be negative velocity.

%     % Coordenadas referentes às do mundo
%     dot_q(car_index+1,:) = ([-sin(q(car_index,3)), 0;  
%                                      cos(q(car_index,3)), 0;
%                                      tan(q(car_index,4))/wheel_base, 0;
%                                      0                                     , 1 ]*...
%                                     [v(car_index); ws(car_index)])'; % ws(car_index)])'; %%
%            
%     q(car_index+1,:) = q(car_index,:) + sampling_time*dot_q(car_index+1,:);
%     
%     % wrap to pi
%     q(car_index+1,3:4) = wrapToPi(q(car_index+1,3:4));
% %     
% %     dt = dt + sampling_time;
% % 
% %     
% %     % am I on the road? If true, change to pure pursuit mode
% %     if norm(error_q(1:2))< wheel_base*1.5
% %         current_trajectory_index = min(current_trajectory_index + 1, path_number_points);
% % %     else
% % %         disp('--waiting--')
% %     end
%     
%     
%     else
%         if DEBUG
%             min_values(car_index) = min_index;
%         end
%         car_index = car_index+1;
%         [~, min_index] = min(vecnorm(q_ref(window(1):k_max,1:2)-repmat(q(car_index,1:2),length(window(1):k_max),1)));
%         
%         if(min_index ~= 1)
%             window = window + min_index-1;
%         end
%     end
% end
% 