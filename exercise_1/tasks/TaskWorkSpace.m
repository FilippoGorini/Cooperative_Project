classdef TaskWorkSpace < Task   
    properties
        distance_star = 1.0;            
        distance_full_activation = 1.5; 
        dist_xy;                        
        n_d;  % Direction versors vehicle-nodule
        gain = 0.3;
    end
    
    methods
        function obj = TaskWorkSpace(name)
            obj.task_name = name;
        end

        function updateReference(obj, robot)
            [~, w_lin] = CartError(robot.wTg , robot.wTt);

            w_lin_xy = w_lin(1:2);
            obj.dist_xy = norm(w_lin_xy);

            if obj.dist_xy > 1e-4
                obj.n_d = [w_lin_xy / obj.dist_xy; 0]; 
            else
                obj.n_d = [0; 0; 0];
            end
 
            obj.xdotbar = obj.gain * (obj.distance_star - obj.dist_xy);
            
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
        end
        
        function updateJacobian(obj, robot)
            J_linear_veh = -obj.n_d';
            obj.J = [zeros(1,7), J_linear_veh, zeros(1,3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = IncreasingBellShapedFunction(obj.distance_star, obj.distance_full_activation, 0, 1, obj.dist_xy);
        end
    end
end