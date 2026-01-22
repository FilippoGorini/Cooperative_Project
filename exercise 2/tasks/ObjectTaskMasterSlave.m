classdef ObjectTaskMasterSlave < Task
    properties
        kp = 0.1; 
    end
    
    methods
        function obj = ObjectTaskMasterSlave(robot_ID, taskID)
            % robot_ID should be "BM"
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot_system)
            % 1. Retrieve the Goal
            wTog = robot_system.left_arm.wTog;
            
            % 2. Retrieve Current Frame (ONLY from Master/Left arm)
            wTo_current = robot_system.left_arm.wTo;
            
            % 3. Compute Error (Goal - Current)
            [ang_error, lin_error] = CartError(wTog, wTo_current);
            
            % 4. Compute Reference Velocity
            obj.xdotbar = obj.kp * [ang_error; lin_error];
            
            % 5. Saturate
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        
        function updateJacobian(obj, robot_system)
            % Retrieve Left Object Jacobian
            Jo_L = robot_system.left_arm.wJo;
            
            if isempty(Jo_L)
                error('wJo is empty.');
            end
            
            % Master/Slave Jacobian
            % Left Arm: Full control (Jo_L)
            % Right Arm: No contribution (Zeros)
            obj.J = [Jo_L, zeros(6, 7)];
        end
        
        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end