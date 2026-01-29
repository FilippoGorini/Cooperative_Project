classdef TaskAltitudeControl < Task   
    properties
        type = 0        % 0 (default) equality, 1 inequality
        h
        % h_star is the goal for the equivalent equality task
        h_star = 1.5;           % For inequality task must be > h_full_activation   
        h_full_activation = 1   % default min altitude to 1
        gain = 0.4
    end
    
    methods
        function obj = TaskAltitudeControl(name,inputType)
                obj.type = inputType;
                obj.task_name = name;
                if inputType == 0
                    obj.gain = 0.05; 
                end
        end

        function updateReference(obj, robot)
            obj.h = robot.altitude;
            % Get reference rate by multiplying gain by error
            obj.xdotbar = obj.gain * (obj.h_star - obj.h);
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end

        function updateJacobian(obj, robot)
            vRw = robot.vTw(1:3, 1:3);        
            v_k_w = vRw * [0; 0; 1];         
            J_a = v_k_w' * [zeros(3,7), eye(3), zeros(3,3)];  
            obj.J = J_a;                         
        end

        function updateActivation(obj, robot)

            % Update the task's inherent activation
            if obj.type == 0
                obj.A = 1;
            else
                obj.A = DecreasingBellShapedFunction(obj.h_full_activation, obj.h_star, 0, 1, obj.h);
            end   

        end

        function set_h_star(obj, value)
            % This method is only for equality tasks (type 0)
            if obj.type ~= 0
                error('TaskAltitudeControl:InvalidType', ...
                      'set_h_star() can only be called on equality tasks (type 0).');
            end
            obj.h_star = value;
        end

        function set_h_min(obj, value)
            % This method is only for inequality tasks (type 1)
            if obj.type ~= 1
                error('TaskAltitudeControl:InvalidType', ...
                      'set_h_min() can only be called on inequality tasks (type 1).');
            end
            obj.h_full_activation = value;
            obj.h_star = value + 0.5;
        end

    end
end