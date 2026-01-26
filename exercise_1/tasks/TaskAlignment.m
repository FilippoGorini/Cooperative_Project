classdef TaskAlignment < Task   
    properties
        theta
        d_vec       % Salviamo il vettore per la skew matrix
        dist        % Salviamo la norma per il denominatore
        n
        yaw_matrix = [zeros(3,2), [0; 0; 1]] 
        gain = 0.3
    end
    methods
        function updateReference(obj, robot)
            i_v = [1; 0; 0];
            
            % 1. Calcolo errore lineare e proiezione
            [~, w_lin] = CartError(robot.wTg , robot.wTv);
            w_lin(3) = 0; 
            obj.d_vec = wTv(1:3,1:3)' * w_lin;  % project w_lin in veichle frame
            obj.dist = norm(w_lin);
            
            % 2. Direzione asse x veicolo
            w_iv = robot.wTv(1:3,1:3) * i_v;
            
            % 3. Normalizzazione direzione target
            if obj.dist > 1e-4
                n_d = obj.d_vec / obj.dist; 
            else
                n_d = w_iv; % Errore nullo se siamo sopra
            end
        
            % 4. Calcolo angolo con atan2 (robusto per il segno)
            sin_theta = w_iv(1)*n_d(2) - w_iv(2)*n_d(1); 
            cos_theta = w_iv(1)*n_d(1) + w_iv(2)*n_d(2); 
            obj.theta = atan2(sin_theta, cos_theta);
            
            % 5. Asse di rotazione (sempre Z per lo yaw)
            obj.n = [0; 0; 1];
            
            % 6. Velocità di riferimento
            obj.xdotbar = obj.gain * obj.theta;
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        
        function updateJacobian(obj, robot)
            % Protezione contro divisione per zero
            if obj.dist < 1e-3
                obj.J = zeros(1, 13);
                return;
            end
            
            % Formula corretta: n' * [ 0_arm | J_linear_veh | J_angular_veh ]
            % Nota: usiamo obj.d_vec (vettore) dentro skew
            J_linear_veh = -(1/(obj.dist^2)) * skew(obj.d_vec);
            J_angular_veh = -obj.yaw_matrix;
            
            obj.J = obj.n' * [zeros(3,7), J_linear_veh, J_angular_veh];
        end
        
        function updateActivation(obj, robot)
            obj.A = 1;
        end
    end
end