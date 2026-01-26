classdef ActionManager < handle
    properties
        dt = 0.01
        num_dofs = 13
        unified_task_list = {}          % Contains all the possible tasks
        actions Action = Action.empty   % Array of action objects
        actionMap                       % Map of action names
        currentAction_idx = 1           % index of currently active action
        previousAction_idx = 1          % index of the previously active action
        timeInCurrentAction = 0         % Time elapsed since setCurrentAction was called
        transitionDuration = 5          % Transition time between actions
    end
    
    methods
        function obj = ActionManager(dt, num_dofs, transitionDuration)
            if nargin >= 1 && ~isempty(dt), obj.dt = dt; end
            if nargin >= 2 && ~isempty(num_dofs), obj.num_dofs = num_dofs; end
            if nargin >= 3 && ~isempty(transitionDuration), obj.transitionDuration = transitionDuration; end
            obj.actionMap = containers.Map('KeyType','char','ValueType','double');
        end

        function addUnifiedList(obj, list)
            obj.unified_task_list = list;
        end

        function addAction(obj, action)
            if ~isa(action, 'Action'), error('Requires an Action object.'); end
            name = char(action.name);
            if isKey(obj.actionMap, name), error('Action "%s" exists.', name); end
            
            obj.actions(end+1) = action;
            obj.actionMap(name) = numel(obj.actions);
        end

        function [v_nu, qdot] = computeICAT(obj, robot)
            % Get current and previous action tasks
            current_tasks  = obj.actions(obj.currentAction_idx).tasks;
            previous_tasks = obj.actions(obj.previousAction_idx).tasks;
            
            ydotbar = zeros(obj.num_dofs, 1);
            Qp = eye(obj.num_dofs);
            
            for i = 1:length(obj.unified_task_list)
                task = obj.unified_task_list{i};
                
                % Check task status in transitions
                inCurrent = false;
                for k = 1:length(current_tasks)
                    if task == current_tasks{k}, inCurrent = true; break; end
                end
                
                inPrevious = false;
                for k = 1:length(previous_tasks)
                    if task == previous_tasks{k}, inPrevious = true; break; end
                end
                
                % Determine transition activation
                trans_act = 0;
                if ~inPrevious && inCurrent
                    if isprop(task, 'is_kin_constraint') && task.is_kin_constraint
                        trans_act = 1; 
                    else
                        trans_act = IncreasingBellShapedFunction(0, obj.transitionDuration, 0, 1, obj.timeInCurrentAction);
                    end
                elseif inPrevious && ~inCurrent
                    if isprop(task, 'is_kin_constraint') && task.is_kin_constraint
                        trans_act = 0;
                    else
                        trans_act = DecreasingBellShapedFunction(0, obj.transitionDuration, 0, 1, obj.timeInCurrentAction);
                    end
                elseif inPrevious && inCurrent
                    trans_act = 1;
                end

                % Update and Compute
                task.updateReference(robot);
                task.updateJacobian(robot);
                task.updateActivation(robot);
                
                if trans_act == 0, continue; end
                
                [Qp, ydotbar] = iCAT_task(trans_act * task.A, task.J, Qp, ydotbar, task.xdotbar, 1e-4, 0.01, 10);
            end
            
            % Residual damping
            [~, ydotbar] = iCAT_task(eye(obj.num_dofs), eye(obj.num_dofs), Qp, ydotbar, zeros(obj.num_dofs,1), 1e-4, 0.01, 10);
            
            % UVMS Split: Arm (1:7), Vehicle (8:13)
            % Adjust indices if your UvmsModel uses a different order
            qdot = ydotbar(1:7);
            v_nu = ydotbar(8:13); 
            
            obj.timeInCurrentAction = obj.timeInCurrentAction + obj.dt;
        end

        function setCurrentAction(obj, actionName)
            name = char(actionName);
            if ~isKey(obj.actionMap, name), error('Action "%s" not found.', name); end
            obj.previousAction_idx = obj.currentAction_idx;
            obj.currentAction_idx = obj.actionMap(name);
            obj.timeInCurrentAction = 0;
        end

        function name = getCurrentActionName(obj)
            name = obj.actions(obj.currentAction_idx).name;
        end
    end
end