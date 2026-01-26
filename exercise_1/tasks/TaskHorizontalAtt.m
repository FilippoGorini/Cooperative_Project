classdef TaskHorizontalAtt < Task   
    properties
        n
        theta
        % theta_star is the goal for the equivalent equality task
        theta_star = 0.1
        theta_full_activation = 0.2
        gain = 0.2
    end


    methods
        function updateReference(obj, robot)
            % In this task our control variable x is the misalignment
            % vector norm (theta) between k_v and k_w so it is a scalar
            v_k_v = [0 0 1]';
            v_k_w = robot.vTw(1:3,3);
            rho = ReducedVersorLemma(v_k_v, v_k_w); % rot vector which makes k_w go to k_v
            % Get axis angle representation equivalent (n*theta = rho)
            obj.theta = norm(rho);
            if obj.theta > 1e-6
                obj.n = rho / obj.theta;
            else
                obj.n = [0, 0, 0]';
            end
            % Get reference rate by multiplying gain (0.2) by error between
            % theta_star and theta
            obj.xdotbar = - obj.gain * (obj.theta_star - obj.theta);
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = obj.n' * [zeros(3,10), eye(3)];
        end
        
        function updateActivation(obj, robot)

            % update the task's inherent activation
            obj.A = IncreasingBellShapedFunction(obj.theta_star, obj.theta_full_activation, 0, 1, obj.theta);
            
        end
    end
end