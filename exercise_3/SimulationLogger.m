classdef SimulationLogger < handle
    properties
        t            % time vector
        
        % Robot State
        ql           % joint positions (Left)
        qdotl        % joint velocities (Left)
        qr           % joint positions (Right)
        qdotr        % joint velocities (Right)
        
        % Task Data
        xdotbar_task % reference velocities (cell array)
        a_task       % task activations (cell array)
        tasks_ref    % list of tasks for THIS logger
        n_tasks      % number of tasks
        
        robot        % reference to coop_system
        data_len     % valid data length
    end
    
    methods
        function obj = SimulationLogger(maxLoops, robotModel, tasks_list)
            obj.robot = robotModel;
            obj.tasks_ref = tasks_list; 
            obj.n_tasks = length(tasks_list);
            obj.data_len = 0;
            
            % Memory Pre-allocation
            obj.t = zeros(1, maxLoops);
            obj.ql = zeros(7, maxLoops);
            obj.qdotl = zeros(7, maxLoops);
            obj.qr = zeros(7, maxLoops);
            obj.qdotr = zeros(7, maxLoops);
            
            obj.xdotbar_task = cell(obj.n_tasks, maxLoops);
            obj.a_task = cell(obj.n_tasks, maxLoops);
        end
        
        function update(obj, t, loop)
            obj.t(loop) = t;
            
            % Log Robot State
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;
            
            % Log Task Data
            for i = 1:obj.n_tasks
                % Task Reference Velocities
                obj.xdotbar_task{i, loop} = obj.tasks_ref{i}.xdotbar;
                
                % Task Activations
                if isprop(obj.tasks_ref{i}, 'A') && ~isempty(obj.tasks_ref{i}.A)
                     obj.a_task{i, loop} = diag(obj.tasks_ref{i}.A);
                else
                     obj.a_task{i, loop} = [];
                end
            end
            
            obj.data_len = loop;
        end
        
        function plotAll(obj, task_indices)
            if nargin < 2 || isempty(task_indices)
                task_indices = 1:obj.n_tasks;
            end
            
            % Check if data exists
            if obj.data_len == 0
                warning('No data logged. Simulation might not have run.');
                return;
            end

            valid_range = 1:obj.data_len;
            time_vec = obj.t(valid_range);
            
            latex_set = @(h) set(h, 'Interpreter', 'latex', 'FontSize', 12);
            
            % --- DETERMINE AGENT ID ---
            % Safe conversion to char to avoid "Value must be a scalar" error
            if ~isempty(obj.tasks_ref) && isprop(obj.tasks_ref{1}, 'ID')
                raw_id = obj.tasks_ref{1}.ID;
                if isstring(raw_id) || ischar(raw_id)
                     agent_id = char(raw_id); % Force char vector
                else
                     agent_id = 'Unknown';
                end
            else
                agent_id = 'Unknown';
            end
            
            % --- FIGURE 1: Robot State ---
            figure('Name', ['Robot State - Agent ' agent_id]); clf;
            
            if strcmp(agent_id, 'L') || strcmp(agent_id, 'BM')
                subplot(2,1,1);
                plot(time_vec, obj.ql(:, valid_range), 'LineWidth', 2);
                t_h = title('Left Arm - Joint Positions ($q_L$)'); latex_set(t_h);
                grid on; xlim([0, time_vec(end)]);
                
                subplot(2,1,2);
                plot(time_vec, obj.qdotl(:, valid_range), 'LineWidth', 2);
                t_h = title('Left Arm - Joint Velocities ($\dot{q}_L$)'); latex_set(t_h);
                grid on; xlim([0, time_vec(end)]);
                
            elseif strcmp(agent_id, 'R')
                subplot(2,1,1);
                plot(time_vec, obj.qr(:, valid_range), 'LineWidth', 2);
                t_h = title('Right Arm - Joint Positions ($q_R$)'); latex_set(t_h);
                grid on; xlim([0, time_vec(end)]);
                
                subplot(2,1,2);
                plot(time_vec, obj.qdotr(:, valid_range), 'LineWidth', 2);
                t_h = title('Right Arm - Joint Velocities ($\dot{q}_R$)'); latex_set(t_h);
                grid on; xlim([0, time_vec(end)]);
            end

            % --- FIGURE 2: Task Reference Velocities ---
            figure('Name', ['Task References - Agent ' agent_id]); clf;
            sg_h = sgtitle(['Task Reference Velocities ($\dot{x}_{bar}$) - Agent ' agent_id]); 
            latex_set(sg_h);
            
            nt = length(task_indices);
            for i = 1:nt
                idx = task_indices(i);
                subplot(ceil(nt/2), 2, i);
                
                raw_data = obj.xdotbar_task(idx, valid_range);
                valid_cells = ~cellfun(@isempty, raw_data);
                
                if any(valid_cells)
                    data = cell2mat(raw_data(valid_cells));
                    plot(time_vec(valid_cells), data', 'LineWidth', 1.5);
                    grid on; xlim([0, time_vec(end)]);
                    
                    t_name = strrep(char(obj.tasks_ref{idx}.task_name), '_', '\_');
                    t_h = title(sprintf('Task %d: %s', idx, t_name)); latex_set(t_h);
                    
                    % Heuristic Legend
                    if size(data, 1) == 6
                        l_h = legend('$\omega_x$','$\omega_y$','$\omega_z$','$v_x$','$v_y$','$v_z$'); 
                        latex_set(l_h);
                    end
                end
            end
            
            % --- FIGURE 3: Activations ---
            figure('Name', ['Task Activations - Agent ' agent_id]); clf;
            sg_h = sgtitle(['Task Activations ($A$) - Agent ' agent_id]); 
            latex_set(sg_h);
            
            for i = 1:nt
                idx = task_indices(i);
                subplot(ceil(nt/2), 2, i);
                
                raw_data = obj.a_task(idx, valid_range);
                valid_cells = ~cellfun(@isempty, raw_data);
                
                if any(valid_cells)
                    data = cell2mat(raw_data(valid_cells));
                    plot(time_vec(valid_cells), data', 'LineWidth', 1.5);
                    grid on; xlim([0, time_vec(end)]); ylim([-0.1 1.1]);
                    
                    t_name = strrep(char(obj.tasks_ref{idx}.task_name), '_', '\_');
                    t_h = title(sprintf('Activation: %s', t_name)); latex_set(t_h);
                end
            end
        end
    end
end