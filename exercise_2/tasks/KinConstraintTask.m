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