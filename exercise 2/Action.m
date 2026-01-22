classdef Action < handle
% Simple action class to store name of action and cell array containing the
% tasks belonging to the action

    properties
        name string
        tasks cell    % cell array of task objects
    end
    
    methods
        function obj = Action(name, tasks)
            if nargin > 0
                if ~(ischar(name) || isstring(name))
                    error('Action name must be a char/string.');
                end
                obj.name = string(name);

                if ~iscell(tasks)
                    error('Tasks must be a cell array.');
                end
                obj.tasks = tasks;
            end
        end
    end
end

