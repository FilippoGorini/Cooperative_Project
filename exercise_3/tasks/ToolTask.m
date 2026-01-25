classdef ToolTask < Task   
    %Tool position control for a single arm
    properties
        gain = 1.0;
    end

    methods

        function obj=ToolTask(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;
        end

        function updateReference(obj, robot_system)
            if(obj.ID=='L')
                robot_system=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot_system=robot_system.right_arm;    
            end
         [v_ang, v_lin] = CartError(robot_system.wTg , robot_system.wTt);
         robot_system.dist_to_goal=v_lin;
         robot_system.rot_to_goal=v_ang;
         obj.xdotbar = obj.gain * [v_ang; v_lin];
         % limit the requested velocities...
         obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
         obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
        end

        function updateJacobian(obj,robot)
            if(obj.ID=='L')
                robot=robot.left_arm;
            elseif(obj.ID=='R')
                robot=robot.right_arm;    
            end
            tool_jacobian=robot.wJt;
            obj.J=tool_jacobian;
            %Optional save the end effector velocity for plotting
            robot.xdot = tool_jacobian * robot.qdot;
        end

        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end