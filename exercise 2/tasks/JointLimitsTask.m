classdef JointLimitsTask < Task   
    % Task to control the joint limits of both arm together at same
    % priority level
    properties
        % Joint limits in radians
        q_min
        q_max
        
        % Safety buffer size near the joint limit
        buffer_zone 

        gain = 1.0; 
    end
    
    methods

        function obj = JointLimitsTask(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
            
            % Joint limits from datasheet (we could also take them from the
            % robot model instead if we pass it as an argument to the
            % constructor)
            limits_deg_min = [-166, -101, -166, -176, -166,  -1, -166, -166, -101, -166, -176, -166,  -1, -166];
            limits_deg_max = [ 166,  101,  166,   -4,  166, 215,  166, 166,  101,  166,   -4,  166, 215,  166];
            
            obj.q_min = deg2rad(limits_deg_min');
            obj.q_max = deg2rad(limits_deg_max');
            
            % 10 degrees buffer zone near the limit (converted in radians)
            obj.buffer_zone = deg2rad(10); 
        end
        
        function updateReference(obj, robot_system)

            q_l = robot_system.left_arm.q;
            q_r = robot_system.right_arm.q;
            q = [q_l; q_r];
            obj.xdotbar = zeros(14,1);
            
            % Compute reference velocities for each joint
            for i = 1:14
                % If we are near the min limit:
                if q(i) < (obj.q_min(i) + obj.buffer_zone)
                    q_star = obj.q_min(i) + obj.buffer_zone;
                    obj.xdotbar(i) = obj.gain * (q_star - q(i));
                    
                % If we are near the max limit:
                elseif q(i) > (obj.q_max(i) - obj.buffer_zone)
                    q_star = obj.q_max(i) - obj.buffer_zone;
                    obj.xdotbar(i) = obj.gain * (q_star - q(i)); % Sarà negativo
                end
            end
            
            % Saturate
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
               
        function updateJacobian(obj, robot_system)
            % For joint limits the jacobian is a simple identity (qdot
            % directly affects qdot as they're the same)         
            obj.J = eye(14);
        end

        function updateActivation(obj, robot_system)

            q_l = robot_system.left_arm.q;
            q_r = robot_system.right_arm.q;
            q = [q_l; q_r];
            activations = zeros(1,14);
            
            for i = 1:14
                alpha_low = 0;
                alpha_high = 0;
                
                % Lower limit activation
                % 1 if we are on the limit, 0 if we are safe (limit + buffer)
                if q(i) < (obj.q_min(i) + obj.buffer_zone)
                     alpha_low = DecreasingBellShapedFunction(obj.q_min(i), obj.q_min(i) + obj.buffer_zone, 0, 1, q(i));
                end
                
                % Upper limit activation
                % 0 if we're safe (limit - buffer), 1 if we are on the limit
                if q(i) > (obj.q_max(i) - obj.buffer_zone)
                     alpha_high = IncreasingBellShapedFunction(obj.q_max(i) - obj.buffer_zone, obj.q_max(i), 0, 1, q(i));
                end
                
                % Take max between the 2 as final activation
                activations(i) = max(alpha_low, alpha_high);
            end
            
            % Diagonal activation matrix
            obj.A = diag(activations);

        end

    end
    
end