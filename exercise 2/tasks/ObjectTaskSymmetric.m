classdef ObjectTaskSymmetric < Task
    properties
        kp = 0.1; % Proportional gain for position control
    end
    
    methods
        function obj = ObjectTaskSymmetric(robot_ID, taskID)
            % robot_ID should be "BM" (Bimanual)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot_system)
            % 1. Retrieve the Object Goal (Stored in either arm)
            wTog = robot_system.left_arm.wTog;
            
            % 2. Retrieve Current Object Frames
            wTo_L = robot_system.left_arm.wTo;
            wTo_R = robot_system.right_arm.wTo;
            
            % 3. Compute Average Current Frame for robustness
            % (Ideally wTo_L == wTo_R due to constraint, but average smooths noise)
            current_pos = 0.5 * (wTo_L(1:3,4) + wTo_R(1:3,4));
            
            % For orientation, since they are very close, we can just pick one
            current_rot = wTo_L(1:3,1:3);
            
            wTo_current = [current_rot, current_pos; 0 0 0 1];
            
            % 4. Compute Error (Goal - Current)
            % Assuming CartError(Goal, Current) convention from your main.m
            [ang_error, lin_error] = CartError(wTog, wTo_current);
            
            % 5. Compute Reference Velocity
            obj.xdotbar = obj.kp * [ang_error; lin_error];
            
            % 6. Saturate for safety
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        
        function updateJacobian(obj, robot_system)
            % Retrieve Object Jacobians
            Jo_L = robot_system.left_arm.wJo;
            Jo_R = robot_system.right_arm.wJo;
            
            if isempty(Jo_L) || isempty(Jo_R)
                error('wJo is empty. Ensure grasp transform is set.');
            end
            
            % Symmetric Jacobian: Weighted average of both arms
            obj.J = [0.5 * Jo_L, 0.5 * Jo_R];
        end
        
        function updateActivation(obj, robot_system)
            % Always active during the Move Phase
            obj.A = eye(6);
        end
    end
end