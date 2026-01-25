classdef ObjectTaskDual < Task
    % In this version of the task we control both arm's object frame in a
    % single task, therefore we give the same priority to both arms
    
    properties
        gain = 1.0; 
    end
    
    methods

        function obj = ObjectTaskDual(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot_system)
            % Retrieve the common goal (should be the same actually if we set them correctly)
            wTog_L = robot_system.left_arm.wTog;
            wTog_R = robot_system.right_arm.wTog;
            
            % Compute errors for left and right arm
            wTo_L = robot_system.left_arm.wTo;
            [ang_err_L, lin_err_L] = CartError(wTog_L, wTo_L);            
            wTo_R = robot_system.right_arm.wTo;
            [ang_err_R, lin_err_R] = CartError(wTog_R, wTo_R);
            
            % Cartesian reference for both arms together (12x1)
            obj.xdotbar = obj.gain * [ang_err_L; lin_err_L; ang_err_R; lin_err_R];
            
            % Saturate
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        
        function updateJacobian(obj, robot_system)
            % Retrieve the current object jacobians from the arms
            JL = robot_system.left_arm.wJo;
            JR = robot_system.right_arm.wJo;
            
            if isempty(JL) || isempty(JR)
                obj.J = zeros(12, 14);  % For safety at beginning while wJo is not set yet
            else
                % Complete jacobian for both arms
                obj.J = [JL,           zeros(6, 7); 
                         zeros(6, 7),  JR         ];
            end
        end
        
        function updateActivation(obj, robot_system)
            % Activation matrix is now 12x12
            obj.A = eye(12);
        end

    end

end