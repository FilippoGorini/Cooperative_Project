classdef ZeroJointVelTask < Task
    % Simple task to set 0 velcoity to all joints of both arms
    
    properties

    end
    
    methods

        function obj = ZeroJointVelTask(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot_system)
            % Reference velocity is just 0 for each joint
            obj.xdotbar = zeros(14,1);
        end
        
        function updateJacobian(obj, robot_system)
            obj.J = eye(14);
        end
        
        function updateActivation(obj, robot_system)
            obj.A = eye(14);
        end

    end

end