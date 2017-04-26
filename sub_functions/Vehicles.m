%% vehicle object class
% 위성과 차량은 원 운동을 한다고 가정한다.

classdef Vehicles < handle    % 슈퍼 클래스를 상속 받지 않는 경우 < hadle 표시로 값 클래스임을 정의해준다.
    properties
        r; theta; omega;
        global_position=[0;0;0];
        x1; y1; % local position
        Velocity;
        Ts = 0.01;
        x_rot_angle;
        Rot_matrix;
        g;     % gravity        
        theta_orbit = linspace(0,2*pi,100);
        local_orbit; global_orbit;
    end
    
    methods
        function obj = Vehicles(r, theta_init, x_rot_angle) 
            obj.r = r;      % vehicle's altitude (= radius)
            obj.theta = theta_init;    % vehicle's initial angle in the polar coordinate
            obj.x_rot_angle = x_rot_angle;  % rotation angle with respect to global X axis
            
            % rotation matrix from local coordiante to global coordinate
            obj.Rot_matrix = [1, 0, 0;...
                0,cos(x_rot_angle), -sin(x_rot_angle);...
                0, sin(x_rot_angle), cos(x_rot_angle)];
            obj.g = 9.81*6400^2/r;       % gravity at the vehicle's altitude
            obj.Velocity = sqrt(obj.g*obj.r);   % vehicle's velocity
            obj.omega = obj.Velocity/obj.r; % orbital angular velocity         
            
            % local position and acceleration (polar coordinate -> local cartesian coordinate)
            obj.x1 = obj.r*cos(obj.theta);
            obj.y1 = obj.r*sin(obj.theta);     
            
            % global position (global cartesian coordinate)
            obj.global_position = obj.Rot_matrix*[obj.x1; obj.y1; 0];
            
            % global orbit generation
            obj.local_orbit = [obj.r*cos(obj.theta_orbit); obj.r*sin(obj.theta_orbit); zeros(1,size(obj.theta_orbit,2))];
            obj.global_orbit = obj.Rot_matrix*obj.local_orbit;
        end
        
        function move(obj)
            obj.theta = obj.theta + obj.omega*obj.Ts;
            obj.theta = rem(obj.theta,2*pi);
            % local position and acceleration (polar coordinate -> local cartesian coordinate)
            obj.x1 = obj.r*cos(obj.theta);
            obj.y1 = obj.r*sin(obj.theta);
            
            % global position (global cartesian coordinate)
            obj.global_position = obj.Rot_matrix*[obj.x1; obj.y1; 0]; 
        end
        
        function global_position = getPosition(obj)   
            global_position = obj.global_position;
            global_position(1) = global_position(1) + randn*0.1;
            global_position(2) = global_position(2) + randn*0.1;
            global_position(3) = global_position(3) + randn*0.1;
        end
        
        function distance = getDistance(obj, car_position)
            distance = sqrt((obj.global_position(1)-car_position(1))^2+(obj.global_position(2)-car_position(2))^2+(obj.global_position(3)-car_position(3))^2);
            distance = distance + randn*0.1;
        end
        
    end
        
    
    
end