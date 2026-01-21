classdef MinEffectorAltitudeTask < Task
    properties
        % --- Task parameters ---
        h_min   = 0.15;   % Minimum allowed altitude from obstacle
        buffer  = 0.05;   % Buffer zone size
        gain    = 1.0;

        % --- References ---
        h_star;          % Target altitude when task activates
        h_current;       % Current altitude relative to obstacle

        % --- Obstacle (table) height ---
        h_obstacle         = 0.55;   % Current obstacle height
        h_obstacle_start   = 0.55;   % Height at transition start
        h_obstacle_target  = 0.55;   % Target height after transition

        % --- Transition handling ---
        transition_duration = 3.0;   % [s]
        time_since_change   = 0;     % Timer
    end

    methods
        function obj = MinEffectorAltitudeTask(robot_ID, taskID)
            obj.ID        = robot_ID;
            obj.task_name = taskID;

            % Desired safe altitude above obstacle
            obj.h_star = obj.h_min + obj.buffer;
        end

        % -------------------------------------------------------------
        % Change obstacle height with smooth transition
        % -------------------------------------------------------------
        function setObstacleHeight(obj, new_height)
            obj.h_obstacle_start  = obj.h_obstacle;
            obj.h_obstacle_target = new_height;
            obj.time_since_change = 0;
        end

        % -------------------------------------------------------------
        % Reference update
        % -------------------------------------------------------------
        function updateReference(obj, robot_system)

            % ---- Update obstacle height smoothly ----
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
                        obj.h_obstacle_target, ... % ymin (final)
                        obj.h_obstacle_start,  ... % ymax (initial)
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

            % ---- Select correct arm ----
            if strcmp(obj.ID, 'L')
                robot = robot_system.left_arm;
            elseif strcmp(obj.ID, 'R')
                robot = robot_system.right_arm;
            else
                error('Unknown robot ID');
            end

            % ---- Compute current relative altitude ----
            obj.h_current = robot.wTe(3,4) - obj.h_obstacle;

            % ---- Velocity reference (1D task) ----
            obj.xdotbar = obj.gain * (obj.h_star - obj.h_current);

            % ---- Safety saturation ----
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end

        % -------------------------------------------------------------
        % Jacobian update
        % -------------------------------------------------------------
        function updateJacobian(obj, robot_system)

            % Select correct arm
            if strcmp(obj.ID, 'L')
                robot = robot_system.left_arm;
            elseif strcmp(obj.ID, 'R')
                robot = robot_system.right_arm;
            else
                error('Unknown robot ID');
            end

            % Extract Z linear velocity Jacobian
            % wJe = [ang; lin] → linear Z is row 6
            Jz = robot.wJe(6, :);

            % Build full Jacobian (1x14)
            if strcmp(obj.ID, 'L')
                obj.J = [Jz, zeros(1, 7)];
            else
                obj.J = [zeros(1, 7), Jz];
            end
        end

        % -------------------------------------------------------------
        % Activation update
        % -------------------------------------------------------------
        function updateActivation(obj, robot_system)
            % Activation depends only on relative altitude
            obj.A = DecreasingBellShapedFunction( ...
                obj.h_min, ...
                obj.h_min + obj.buffer, ...
                0, ...
                1, ...
                obj.h_current);
        end
    end
end
