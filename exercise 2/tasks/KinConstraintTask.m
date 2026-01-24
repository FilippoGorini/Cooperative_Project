classdef KinConstraintTask < Task   
    % Enforce kinematic constraint between the 2 arms
    properties
        
    end

    methods

        function obj = KinConstraintTask(robot_ID, taskID)
            obj.is_kin_constraint = true; % Notice this is a kinematic constraint
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end

        function updateReference(obj, robot_system)
            % Reference set to zero always for kinematic constraint
            obj.xdotbar = zeros(6,1);
        end

        % function updateReference(obj, robot_system)
        %      % This task forces the two arms to hold the object together.
        %      % Ideally, the relative velocity is 0.
        %      % However, we add a proportional correction to fix numerical drift
        %      % ensuring the two calculated object frames stay overlapping.
        % 
        %      wTo_L = robot_system.left_arm.wTo;
        %      wTo_R = robot_system.right_arm.wTo;
        % 
        %      % Calculate Error: Deviation of Right arm's object frame wrt Left arm's
        %      % We want (Pos_L - Pos_R) = 0
        % 
        %      [ang_err, lin_err] = CartError(wTo_L, wTo_R);
        % 
        %      % Our target velocity is to close this error
        %      % Note: CartError returns vector from T to G. 
        %      % Here we treat Left as Goal, Right as Tool (or vice versa).
        %      % J = [J_L -J_R], so equation is v_L - v_R = - Kp * (x_L - x_R) ?
        %      % Let's stick to standard error dynamics:
        %      % error = x_L - x_R. We want dot(error) = -Kp * error.
        %      % J_L*q_L - J_R*q_R = -Kp * error
        % 
        %      obj.xdotbar =  2 * [ang_err; lin_err];
        % 
        %      % Saturate for safety
        %      obj.xdotbar = Saturate(obj.xdotbar, 0.4);
        % end
        % 

        function updateJacobian(obj,robot_system)
            % The Jacobian for the object kinematic constraint is a 6 x 14 matrix:
            % [J_o_Left, -J_o_Right]             
            JL = robot_system.left_arm.wJo;
            JR = robot_system.right_arm.wJo;
            obj.J = [JL, -JR];
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end

    end
    
end