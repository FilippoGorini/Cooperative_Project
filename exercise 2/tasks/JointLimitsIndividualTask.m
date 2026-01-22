classdef JointLimitsIndividualTask < Task   
    % Task per gestire i limiti di giunto del robot (Inequality Task)
    properties
        % Limiti di giunto in radianti
        q_min
        q_max
        
        % Zona di sicurezza (buffer) prima del limite effettivo
        buffer_zone 
        
        % Guadagno per la velocità di rientro
        kp = 1.0; 
    end
    
    methods
        function obj = JointLimitsIndividualTask(robot_ID, taskID)
            % Costruttore: Inizializza ID e limiti del Franka Panda
            obj.ID = robot_ID;
            obj.task_name = taskID;
            
            % Conversione da gradi a radianti
            deg2rad = pi/180;
            
            % Limiti dal Datasheet Franka Emika Panda
            % A1, A2, A3, A4, A5, A6, A7
            limits_deg_min = [-166, -101, -166, -176, -166,  -1, -166];
            limits_deg_max = [ 166,  101,  166,   -4,  166, 215,  166];
            
            obj.q_min = limits_deg_min' * deg2rad;
            obj.q_max = limits_deg_max' * deg2rad;
            
            % Definiamo un buffer di sicurezza (es. 10 gradi)
            obj.buffer_zone = 10 * deg2rad; 
        end
        
        function updateReference(obj, robot_system)
            % Seleziona il braccio corretto
            if(obj.ID == 'L')
                robot = robot_system.left_arm;
            elseif(obj.ID == 'R')
                robot = robot_system.right_arm;    
            end
            
            q = robot.q;
            obj.xdotbar = zeros(7,1); % Inizializza velocità riferimento a zero
            
            % Calcola la velocità repulsiva per ogni giunto
            for i = 1:7
                % CASO 1: Vicino al limite MINIMO
                if q(i) < (obj.q_min(i) + obj.buffer_zone)
                    % Obiettivo: tornare verso il buffer
                    q_target = obj.q_min(i) + obj.buffer_zone;
                    obj.xdotbar(i) = obj.kp * (q_target - q(i));
                    
                % CASO 2: Vicino al limite MASSIMO
                elseif q(i) > (obj.q_max(i) - obj.buffer_zone)
                    % Obiettivo: tornare indietro verso il buffer
                    q_target = obj.q_max(i) - obj.buffer_zone;
                    obj.xdotbar(i) = obj.kp * (q_target - q(i)); % Sarà negativo
                end
            end
            
            % Saturazione di sicurezza per evitare scatti violenti
            obj.xdotbar = Saturate(obj.xdotbar, 0.5);
        end
               
        function updateJacobian(obj, robot_system)
            % Il Jacobiano per i joint limits è semplicemente una matrice Identità
            % (vogliamo controllare direttamente qdot).
            
            % Costruiamo il Jacobiano esteso per il sistema (14 DOF)
            if obj.ID == 'L'
                % Parte sinistra Identity, parte destra Zeri
                obj.J = [eye(7), zeros(7,7)];
            elseif obj.ID == 'R'
                % Parte sinistra Zeri, parte destra Identity
                obj.J = [zeros(7,7), eye(7)];
            end
        end

        function updateActivation(obj, robot_system)
            % Seleziona il braccio
            if(obj.ID == 'L')
                robot = robot_system.left_arm;
            elseif(obj.ID == 'R')
                robot = robot_system.right_arm;    
            end
            
            q = robot.q;
            activations = zeros(1,7);
            
            for i = 1:7
                alpha_low = 0;
                alpha_high = 0;
                
                % Attivazione Limite Inferiore (Decreasing)
                % 1 se siamo sul limite, 0 se siamo al sicuro (limite + buffer)
                if q(i) < (obj.q_min(i) + obj.buffer_zone)
                     alpha_low = DecreasingBellShapedFunction(obj.q_min(i), obj.q_min(i) + obj.buffer_zone, 0, 1, q(i));
                end
                
                % Attivazione Limite Superiore (Increasing)
                % 0 se siamo al sicuro (limite - buffer), 1 se siamo sul limite
                if q(i) > (obj.q_max(i) - obj.buffer_zone)
                     alpha_high = IncreasingBellShapedFunction(obj.q_max(i) - obj.buffer_zone, obj.q_max(i), 0, 1, q(i));
                end
                
                % L'attivazione finale è la massima tra le due (non possono avvenire insieme)
                activations(i) = max(alpha_low, alpha_high);
            end
            
            % La matrice di attivazione è diagonale (ogni giunto è indipendente)
            obj.A = diag(activations);
        end
    end
end