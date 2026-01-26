classdef TaskVehiclePos < Task   
    properties
        gain = 0.4
    end


    methods
        function updateReference(obj, robot)
            % In this task our control variable x is the cartesian distance
            % only (no orientation), so it is a 3x1 vector, therefore xdot
            % is 3x1 as well.
            [~, w_lin_err] = CartError(robot.wTgv , robot.wTv);
            % lambda = 0.2 gain
            obj.xdotbar = - obj.gain * w_lin_err;  % we should also consider the velocity of the goal if it was moving (feedforward term)
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7), -eye(3), zeros(3,3)];
        end
        
        function updateActivation(obj, robot)
            % Update the task's inherent activation
            obj.A = eye(3);
        end
    end
end