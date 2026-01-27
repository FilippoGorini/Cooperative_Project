classdef TaskStop < Task   
    properties
        gain = 0.3
        is_kin_constraint = 1;
    end
    methods
        function obj = TaskStop(name)
            obj.task_name = name;
            
        end

        function updateReference(obj, robot)
            obj.xdotbar = zeros(6,1);
        end
        
        function updateJacobian(obj, robot)
            obj.J = [zeros(6, 7), eye(6)];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end
