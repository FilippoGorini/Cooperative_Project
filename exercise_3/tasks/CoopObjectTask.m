classdef CoopObjectTask < Task
    
    properties
        H_lr
        C
        xdot_hat   
        gain = 1.0; 
    end
    
    methods

        function obj = CoopObjectTask(robot_ID, taskID)
            obj.is_kin_constraint = true;
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot_system)
            
            % Compute feasible cooperative velocity vector (12x1)
            xdot_tilde = obj.H_lr * (eye(12) - pinv(obj.C) * obj.C) * [obj.xdot_hat; obj.xdot_hat];

            % Compute reference for arm (6x1)
            if obj.ID == 'L'
                obj.xdotbar = xdot_tilde(1:6);
            elseif obj.ID == 'R'
                obj.xdotbar = xdot_tilde(7:12);   
            end

            % Saturate
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        
        function updateJacobian(obj, robot_system)

            if obj.ID == 'L'
                robot = robot_system.left_arm;
            elseif obj.ID == 'R'
                robot = robot_system.right_arm;    
            end

            % Retrieve the current object jacobian
            wJo = robot.wJo;
            
            if isempty(wJo)
                obj.J = zeros(6, 7);  % For safety at beginning while wJo is not set yet
            else
                obj.J = wJo;
            end
        end
        
        function updateActivation(obj, robot_system)
            % Activation matrix is now 6x6 (only one arm)
            obj.A = eye(6);
        end

    end

end