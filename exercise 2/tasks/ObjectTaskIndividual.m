classdef ObjectTaskIndividual < Task
    % This version of the task controls a single arm object frame, either
    % the left or right one (we can have both L and R ones in the main, but
    % they will have different priorities)

    properties
        gain = 1.0; 
    end

    methods
        % The robot_ID selects which arm's object frame we control
        function obj = ObjectTaskIndividual(robot_ID, taskID)
            if ~strcmp(robot_ID, 'L') && ~strcmp(robot_ID, 'R')
                error('ObjectTaskIndividual: ID must be "L" or "R".');
            end
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end

        function updateReference(obj, robot_system)
            % Select correct arm
            if obj.ID == 'L'
                arm = robot_system.left_arm;
            elseif obj.ID == 'R'
                arm = robot_system.right_arm;
            end

            % Get object goal and current object frame position according
            % to the selected arm
            wTog = arm.wTog;
            wTo_current = arm.wTo;

            % Compute error
            [ang_error, lin_error] = CartError(wTog, wTo_current);

            % Reference
            obj.xdotbar = obj.gain * [ang_error; lin_error];

            % Saturate
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end

        function updateJacobian(obj, robot_system)

            if obj.ID == 'L'
                % We control the object using the left arm's jacobian.
                Jo = robot_system.left_arm.wJo;

                if isempty(Jo)
                    obj.J = zeros(6, 14);   % For safety at beginning while wJo is not set yet
                else
                    obj.J = [Jo, zeros(6, 7)];
                end

            elseif obj.ID == 'R'
                % We control the object using the right arm's jacobian.
                Jo = robot_system.right_arm.wJo;

                if isempty(Jo)
                    obj.J = zeros(6, 14);
                else
                    obj.J = [zeros(6, 7), Jo];
                end
            end
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end

    end
    
end