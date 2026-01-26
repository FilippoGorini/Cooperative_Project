classdef MinEffectorAltitudeTask < Task
    % Task to keep EE frame at a minimum altitude from a predefined
    % obstacle. The obstacle height can be changed in runtime and the
    % transitions will happen smoothly over a period of 3 seconds

    % NB: MAYBE WE SHOULD CHANGE THE CONTROL POINT FROM THE EE TO THE
    % TOOL!!!!!!! (BOTH REFERENCE AND JACOBIAN METHOD WOULD NEED UPDATE)

    properties
        h_min = 0.15;               % Minimum allowed altitude from obstacle
        buffer = 0.05;              % Buffer zone size
        gain = 1.0;

        h_star;                     % Target altitude when task activates
        h_current;                  % Current altitude relative to obstacle

        h_obstacle = 0.55;          % Current obstacle height
        h_obstacle_start = 0.55;    % Height at transition start
        h_obstacle_target = 0.55;   % Target height after transition

        transition_duration = 3.0;  % obstacle height transition time
        time_since_change = 0;    
    end

    methods

        function obj = MinEffectorAltitudeTask(robot_ID, taskID)
            obj.ID        = robot_ID;
            obj.task_name = taskID;

            % Desired safe altitude above obstacle (minimum altitude plus buffer)
            obj.h_star = obj.h_min + obj.buffer;
        end

        % Method to set a new obstacle height, to which we will transition smoothly
        function setObstacleHeight(obj, new_height)
            obj.h_obstacle_start  = obj.h_obstacle;
            obj.h_obstacle_target = new_height;
            obj.time_since_change = 0;
        end

        function updateReference(obj, robot_system)

            % Logic for obstacle height smoothing
            if obj.time_since_change < obj.transition_duration
                obj.time_since_change = min( ...
                    obj.time_since_change + robot_system.dt, ...
                    obj.transition_duration);

                t = obj.time_since_change;

                if obj.h_obstacle_start > obj.h_obstacle_target
                    % Decreasing transition
                    obj.h_obstacle = DecreasingBellShapedFunction( ...
                        0, ...
                        obj.transition_duration, ...
                        obj.h_obstacle_target, ...      % ymin (final)
                        obj.h_obstacle_start,  ...      % ymax (initial)
                        t);
                else
                    % Increasing transition
                    obj.h_obstacle = IncreasingBellShapedFunction( ...
                        0, ...
                        obj.transition_duration, ...
                        obj.h_obstacle_start, ...
                        obj.h_obstacle_target, ...
                        t);
                end
            else
                obj.h_obstacle = obj.h_obstacle_target;
            end

            if obj.ID == 'L'
                robot = robot_system.left_arm;
            elseif obj.ID == 'R'
                robot = robot_system.right_arm;    
            end

            % Compute current relative altitude
            obj.h_current = robot.wTe(3,4) - obj.h_obstacle;

            % Reference
            obj.xdotbar = obj.gain * (obj.h_star - obj.h_current);

            % Saturate
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end

        function updateJacobian(obj, robot_system)

            if obj.ID == 'L'
                robot = robot_system.left_arm;
            elseif obj.ID == 'R'
                robot = robot_system.right_arm;    
            end

            % Extract Z linear velocity Jacobian
            Jz = robot.wJe(6, :);

            % Build full Jacobian (1x14)
            if obj.ID == 'L'
                obj.J = [Jz, zeros(1, 7)];
            elseif obj.ID == 'R'
                obj.J = [zeros(1, 7), Jz];
            end
        end

        function updateActivation(obj, robot_system)
            % Activation depends only on relative altitude
            obj.A = DecreasingBellShapedFunction(obj.h_min, ...
                                                    obj.h_min + obj.buffer, ...
                                                    0, ...
                                                    1, ...
                                                    obj.h_current);
        end

    end
    
end
