classdef ActionManager < handle
    properties
        dt = 0.01
        num_dofs = 13
        task_set = {}                   % Contains all the possible tasks
        actions Action = Action.empty   % Array of action objects
        actionMap                       % Map of action names
        currentAction = 1               % index of currently active action
        previousAction = 1              % index of the previously active action
        timeInCurrentAction = 0         % Time elapsed since setCurrentAction was called
        transitionDuration = 5          % 5 second transition time between actions
    end

    methods

        % Constructor for the ActionManager, specify dt and transition
        % duration for correct transitions between actions
        function obj = ActionManager(dt, num_dofs, transitionDuration)
            
            if nargin >= 1 && ~isempty(dt)
                if ~isnumeric(dt) || ~isscalar(dt) || dt <= 0
                    error('dt must be a positive numeric scalar.');
                end
                obj.dt = dt;
            end
            
            if nargin >= 2 && ~isempty(num_dofs)
                if ~isnumeric(num_dofs) || ~isscalar(num_dofs) || num_dofs <= 0
                    error('num_dofs must be a positive numeric scalar.');
                end
                obj.num_dofs = num_dofs;
            end

            if nargin >= 3 && ~isempty(transitionDuration)
                if ~isnumeric(transitionDuration) || ~isscalar(transitionDuration) || transitionDuration <= 0
                    error('transitionDuration must be a positive numeric scalar.');
                end
                obj.transitionDuration = transitionDuration;
            end

            obj.actionMap = containers.Map('KeyType','char','ValueType','double');

        end

        % This method adds the global task set to the ActionManager
        % The priorities of the tasks depend on the order in this set
        function addTaskSet(obj, taskSet)
            obj.task_set = taskSet;
        end

        % This method now accepts an Action object and updates the
        % ActionManager actions and actionMap fields
        function addAction(obj, action)

            % Check if argument is an Action object
            if ~isa(action, 'Action')
                error('addAction requires an Action object.');
            end
            
            % Check that there aren't already actions with the same name
            name = char(action.name);
            if isKey(obj.actionMap, name)
                error('An action named "%s" already exists.', name);
            end

            % Append action and update map
            obj.actions(end+1) = action;
            obj.actionMap(name) = numel(obj.actions);
        end

        % This method iterates on all the tasks in order of priority and
        % performs the ICAT step
        function qdot = computeICAT(obj, robot)

            % Get current action
            current_tasks  = obj.actions(obj.currentAction).tasks;
            previous_tasks = obj.actions(obj.previousAction).tasks;

            % Perform ICAT (task-priority inverse kinematics)
            ydotbar = zeros(obj.num_dofs,1);
            Qp = eye(obj.num_dofs);

            for i = 1:length(obj.task_set)  % Iterate on ALL of the possible tasks
                
                % Extract task
                task = obj.task_set{i};

                % Check if task is contained in current and/or previous action
                inCurrent = false;
                for k = 1:length(current_tasks)
                    if task == current_tasks{k}, inCurrent = true; break; end
                end
                inPrevious = false;
                for k = 1:length(previous_tasks)
                    if task == previous_tasks{k}, inPrevious = true; break; end
                end          

                % Determine task transition activation based on task status
                trans_act = 0;
                if ~inPrevious && inCurrent
                    if task.is_kin_constraint
                        trans_act = 1;
                    else
                        trans_act = IncreasingBellShapedFunction(0, ...
                                                                obj.transitionDuration, ...
                                                                0, ...
                                                                1, ...
                                                                obj.timeInCurrentAction);
                    end
                elseif inPrevious && ~inCurrent
                    if task.is_kin_constraint
                        trans_act = 0;
                    else
                        trans_act = DecreasingBellShapedFunction(0, ...
                                                                obj.transitionDuration, ...
                                                                0, ...
                                                                1, ...
                                                                obj.timeInCurrentAction);
                    end
                elseif inPrevious && inCurrent
                    trans_act = 1;
                end

                % Update task
                task.updateReference(robot);
                task.updateJacobian(robot);
                task.updateActivation(robot);
                
                % If the transition activation is 0 we can just skip the ICAT
                % computation all together
                if trans_act == 0
                    continue; 
                end
                
                % Perform ICAT Step
                [Qp, ydotbar] = iCAT_task(trans_act*task.A, task.J, ...
                                           Qp, ydotbar, task.xdotbar, ...
                                           1e-4, 0.01, 10);
            end

            % Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(obj.num_dofs), eye(obj.num_dofs), Qp, ydotbar, zeros(obj.num_dofs,1), 1e-4, 0.01, 10);

            qdot = ydotbar;

            % Increment time elapsed since last action switch
            obj.timeInCurrentAction = obj.timeInCurrentAction + obj.dt;  % better use robot.dt

        end

        % This method triggers the switch to a new action 
        function setCurrentAction(obj, actionName)
            % Switch to a different action
            if ~(ischar(actionName) || isstring(actionName))
                error('setCurrentAction expects an action name (char or string).');
            end
            name = char(actionName);

            if ~isKey(obj.actionMap, name)
                error('No action named "%s" exists. Use addAction(Action) first.', name);
            end

            idx = obj.actionMap(name);

            % Update previous/current indices
            obj.previousAction = obj.currentAction;
            obj.currentAction = idx;
            obj.timeInCurrentAction = 0;
        end

        function name = getCurrentActionName(obj)
            name = obj.actions{obj.currentAction}.name;
        end
    end
end