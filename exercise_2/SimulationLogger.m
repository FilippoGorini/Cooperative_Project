classdef SimulationLogger < handle
    properties
        t            % time vector
        ql           % joint positions (Left)
        qdotl        % joint velocities (Left)
        qr           % joint positions (Right)
        qdotr        % joint velocities (Right)
        a            % task activations
        xdotbar_task % reference velocities
        robot        % robot model
        
        tasks_ref    % global task list
        n_tasks      % number of tasks
        a_task       % task activations data
        
        data_len     % valid data length
    end
    methods
        function obj = SimulationLogger(maxLoops, robotModel, global_tasks)
            obj.robot = robotModel;
            obj.tasks_ref = global_tasks; 
            obj.n_tasks = length(global_tasks);
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
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;
            
            for i = 1:obj.n_tasks
                obj.xdotbar_task{i, loop} = obj.tasks_ref{i}.xdotbar;
                obj.a_task{i, loop} = diag(obj.tasks_ref{i}.A);
            end
            obj.data_len = loop;
        end
        
        function plotAll(obj, task_indices)
            if nargin < 2 || isempty(task_indices)
                task_indices = 1:obj.n_tasks;
            end
            
            valid_range = 1:obj.data_len;
            time_vec = obj.t(valid_range);
            
            % Helper for consistent LaTeX formatting
            latex_set = @(h) set(h, 'Interpreter', 'latex', 'FontSize', 12);

            % --- FIGURE 1: Left Arm State ---
            figure(1); clf;
            subplot(2,1,1);
            plot(time_vec, obj.ql(:, valid_range), 'LineWidth', 2);
            t_h = title('Left Arm - Joint Positions ($q_L$)'); latex_set(t_h);
            l_h = legend('$q_1$','$q_2$','$q_3$','$q_4$','$q_5$','$q_6$','$q_7$'); latex_set(l_h);
            grid on; xlim([0, time_vec(end)]);
            
            subplot(2,1,2);
            plot(time_vec, obj.qdotl(:, valid_range), 'LineWidth', 2);
            t_h = title('Left Arm - Joint Velocities ($\dot{q}_L$)'); latex_set(t_h);
            l_h = legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','$\dot{q}_7$'); latex_set(l_h);
            grid on; xlim([0, time_vec(end)]);
            
            % --- FIGURE 2: Right Arm State ---
            figure(2); clf;
            subplot(2,1,1);
            plot(time_vec, obj.qr(:, valid_range), 'LineWidth', 2);
            t_h = title('Right Arm - Joint Positions ($q_R$)'); latex_set(t_h);
            l_h = legend('$q_1$','$q_2$','$q_3$','$q_4$','$q_5$','$q_6$','$q_7$'); latex_set(l_h);
            grid on; xlim([0, time_vec(end)]);
            
            subplot(2,1,2);
            plot(time_vec, obj.qdotr(:, valid_range), 'LineWidth', 2);
            t_h = title('Right Arm - Joint Velocities ($\dot{q}_R$)'); latex_set(t_h);
            l_h = legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','$\dot{q}_7$'); latex_set(l_h);
            grid on; xlim([0, time_vec(end)]);
            
            % --- FIGURE 3: Task Reference Velocities ---
            figure(3); clf;
            sg_h = sgtitle('Task Reference Velocities ($\dot{\bar{x}}$)'); latex_set(sg_h);
            nt = length(task_indices);
            
            for i = 1:nt
                idx = task_indices(i);
                subplot(ceil(nt/2), 2, i);
                
                raw_data = obj.xdotbar_task(idx, valid_range);
                if any(~cellfun(@isempty, raw_data))
                    data = cell2mat(raw_data(~cellfun(@isempty, raw_data)));
                    plot(time_vec(~cellfun(@isempty, raw_data)), data', 'LineWidth', 1.5);
                    grid on; xlim([0, time_vec(end)]);
                    
                    % Escape underscores in task names for LaTeX
                    t_name = strrep(char(obj.tasks_ref{idx}.task_name), '_', '\_');
                    t_id = char(obj.tasks_ref{idx}.ID);
                    t_h = title(sprintf('Task %d: %s (ID: %s)', idx, t_name, t_id)); latex_set(t_h);
                    
                    % --- UPDATED LEGEND LOGIC ---
                    if size(data, 1) == 6
                        % Standard 6-DoF Legend
                        l_h = legend('$\omega_x$','$\omega_y$','$\omega_z$','$v_x$','$v_y$','$v_z$'); 
                        latex_set(l_h);
                        set(l_h, 'NumColumns', 2); % 2 Columns for standard tasks
                        
                    elseif size(data, 1) == 12
                        % 12-DoF Object Task: Use compact notation and 4 columns
                        % L = Left, R = Right
                        l_h = legend('$\omega_{x,L}$','$\omega_{y,L}$','$\omega_{z,L}$','$v_{x,L}$','$v_{y,L}$','$v_{z,L}$', ...
                                     '$\omega_{x,R}$','$\omega_{y,R}$','$\omega_{z,R}$','$v_{x,R}$','$v_{y,R}$','$v_{z,R}$');
                        latex_set(l_h);
                        set(l_h, 'NumColumns', 2, 'FontSize', 10); % 4 Columns = 3 Rows (Much smaller)
                    end
                end
            end
            
            % --- FIGURE 4: Activations ---
            figure(4); clf;
            sg_h = sgtitle('Task Activations ($A$)'); latex_set(sg_h);
            for i = 1:nt
                idx = task_indices(i);
                subplot(ceil(nt/2), 2, i);
                
                raw_data = obj.a_task(idx, valid_range);
                if any(~cellfun(@isempty, raw_data))
                    data = cell2mat(raw_data(~cellfun(@isempty, raw_data)));
                    plot(time_vec(~cellfun(@isempty, raw_data)), data', 'LineWidth', 1.5);
                    grid on; xlim([0, time_vec(end)]); ylim([-0.1 1.1]);
                    
                    t_name = strrep(char(obj.tasks_ref{idx}.task_name), '_', '\_');
                    t_h = title(sprintf('Activation: %s', t_name)); latex_set(t_h);
                end
            end
        end
    end
end