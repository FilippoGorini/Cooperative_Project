classdef SimulationLogger < handle
    properties
        t            % time vector
        ql            % joint positions
        qdotl        % joint velocities
        qr            % joint positions
        qdotr        % joint velocities
        a            % task activations (diagonal only)
        xdotbar_task % reference velocities for tasks (cell array)
        robot        % robot model
        
        tasks_ref     % reference to the global list
        n_tasks       % number of the total task to look at

        a_task       % Cell array for task activation (diagonal of A) of task

        %action_set     % set of actions (Now contains ActionManager)
        %n              % number of actions
    end

    methods
        function obj = SimulationLogger(maxLoops, robotModel, global_tasks)
            obj.robot = robotModel;
            obj.tasks_ref = global_tasks; 
            obj.n_tasks = length(global_tasks);
            
            % Pre-allocazione memoria
            obj.t = zeros(1, maxLoops);
            obj.ql = zeros(7, maxLoops);
            obj.qdotl = zeros(7, maxLoops);
            obj.qr = zeros(7, maxLoops);
            obj.qdotr = zeros(7, maxLoops);
            
            % Inizializzazione contenitori dati task
            obj.xdotbar_task = cell(obj.n_tasks, maxLoops);
            obj.a_task = cell(obj.n_tasks, maxLoops);
        end

        function update(obj, t, loop)
            % 1. Salva lo stato del robot
            obj.t(loop) = t;
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;
            
            % 2. Salva i dati di ogni task presente nella global_list
            for i = 1:obj.n_tasks
                % Velocità di riferimento
                obj.xdotbar_task{i, loop} = obj.tasks_ref{i}.xdotbar;
                % Attivazione (estraiamo la diagonale della matrice A)
                obj.a_task{i, loop} = diag(obj.tasks_ref{i}.A);
            end
        end

        function plotAll(obj, task_indices)
            % Se non vengono passati indici, plotta tutte le task
            if nargin < 2 || isempty(task_indices)
                task_indices = 1:obj.n_tasks;
            end
            
            % --- FIGURA 1: Stato Braccio Sinistro ---
            figure(1);
            subplot(2,1,1);
            plot(obj.t, obj.ql, 'LineWidth', 2);
            title('Left Arm - Joint Positions');
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
            grid on;
            subplot(2,1,2);
            plot(obj.t, obj.qdotl, 'LineWidth', 2);
            title('Left Arm - Joint Velocities');
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');
            grid on;

            % --- FIGURA 2: Stato Braccio Destro ---
            figure(2);
            subplot(2,1,1);
            plot(obj.t, obj.qr, 'LineWidth', 2);
            title('Right Arm - Joint Positions');
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
            grid on;
            subplot(2,1,2);
            plot(obj.t, obj.qdotr, 'LineWidth', 2);
            title('Right Arm - Joint Velocities');
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');
            grid on;

            % --- FIGURA 3: Velocità di Riferimento Task (xdotbar) ---
            figure(3);
            sgtitle('Task Reference Velocities (xdotbar)');
            nt = length(task_indices);
            for i = 1:nt
                idx = task_indices(i);
                subplot(ceil(nt/2), 2, i);
                
                % Recupero dati validi (esclude step non ancora simulati)
                valid = ~cellfun(@isempty, obj.xdotbar_task(idx, :));
                if any(valid)
                    data = cell2mat(obj.xdotbar_task(idx, valid));
                    plot(obj.t(valid), data', 'LineWidth', 1.5);
                    grid on;
                    title(['Task ', num2str(idx), ': ', char(obj.tasks_ref{idx}.ID), ' - ', char(obj.tasks_ref{idx}.task_name)]);
                    if size(data, 1) == 6
                        legend('wx','wy','wz','vx','vy','vz');
                    end
                end
            end

            % --- FIGURA 4: Attivazioni Task (A) ---
            figure(4);
            sgtitle('Task Activations (A)');
            for i = 1:nt
                idx = task_indices(i);
                subplot(ceil(nt/2), 2, i);
                
                valid = ~cellfun(@isempty, obj.a_task(idx, :));
                if any(valid)
                    data = cell2mat(obj.a_task(idx, valid));
                    plot(obj.t(valid), data', 'LineWidth', 1.5);
                    grid on;
                    ylim([-0.1 1.1]); % Le attivazioni sono tipicamente tra 0 e 1
                    title(['Activation Task ', num2str(idx), ': ', char(obj.tasks_ref{idx}.ID)]);
                end
            end
        end
    end
end