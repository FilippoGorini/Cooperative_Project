classdef ObjectTask < Task
    % In this version of the task we control 1 arm's object frame
    
    properties
        gain = 1.0; 
    end
    
    methods

        function obj = ObjectTask(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot_system)

            if obj.ID == 'L'
                robot = robot_system.left_arm;
            elseif obj.ID == 'R'
                robot = robot_system.right_arm;    
            end

            % Retrieve the goal
            wTog = robot.wTog;
            
            % Compute errors for left and right arm
            wTo = robot.wTo;
            [ang_err, lin_err] = CartError(wTog, wTo);            
            
            % Cartesian reference for 1 arm (6x1)
            obj.xdotbar = obj.gain * [ang_err; lin_err];
            
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
            % Activation matrix is now 12x12
            obj.A = eye(6);
        end

    end

end