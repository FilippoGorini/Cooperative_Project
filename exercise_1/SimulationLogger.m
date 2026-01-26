classdef SimulationLogger < handle
    properties
        t            % time vector
        q            % joint positions (Arm)
        qdot         % joint velocities (Arm)
        eta          % vehicle pose [x y z roll pitch yaw]
        v_nu         % vehicle velocity [v_linear v_angular]
        
        robot        % robot model
        tasks_ref    % global task list
        n_tasks      % number of tasks
        xdotbar_task % reference velocities (cell array for varying DoFs)
        a_task       % task activations data (cell array)
        
        data_len     % valid data length tracking
    end
    
    methods
        function obj = SimulationLogger(maxLoops, robotModel, global_tasks)
            obj.robot = robotModel;
            obj.tasks_ref = global_tasks; 
            obj.n_tasks = length(global_tasks);
            obj.data_len = 0;
            
            % Memory Pre-allocation
            obj.t = zeros(1, maxLoops);
            obj.q = zeros(7, maxLoops);
            obj.qdot = zeros(7, maxLoops);
            obj.eta = zeros(6, maxLoops);
            obj.v_nu = zeros(6, maxLoops);
            
            % Task data pre-allocation using cell arrays (matches bimanual logic)
            obj.xdotbar_task = cell(obj.n_tasks, maxLoops);
            obj.a_task = cell(obj.n_tasks, maxLoops);
        end
        
        function update(obj, t, loop)
            obj.t(loop) = t;
            % Arm State
            obj.q(:, loop) = obj.robot.q;
            obj.qdot(:, loop) = obj.robot.q_dot;
            % Vehicle State
            obj.eta(:, loop) = obj.robot.eta;
            obj.v_nu(:, loop) = obj.robot.v_nu;
            
            % Task-specific data
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
            
            % --- FIGURE 1: Arm State ---
            figure(1); clf;
            subplot(2,1,1);
            plot(time_vec, obj.q(:, valid_range), 'LineWidth', 2);
            t_h = title('Manipulator - Joint Positions ($q$)'); latex_set(t_h);
            l_h = legend('$q_1$','$q_2$','$q_3$','$q_4$','$q_5$','$q_6$','$q_7$'); latex_set(l_h);
            grid on; xlim([0, time_vec(end)]);
            
            subplot(2,1,2);
            plot(time_vec, obj.qdot(:, valid_range), 'LineWidth', 2);
            t_h = title('Manipulator - Joint Velocities ($\dot{q}$)'); latex_set(t_h);
            l_h = legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','$\dot{q}_7$'); latex_set(l_h);
            grid on; xlim([0, time_vec(end)]);
            
            % --- FIGURE 2: Vehicle State ---
            figure(2); clf;
            subplot(2,1,1);
            plot(time_vec, obj.eta(:, valid_range), 'LineWidth', 2);
            t_h = title('Vehicle Pose ($\eta$)'); latex_set(t_h);
            l_h = legend('$x$','$y$','$z$','$\phi$','$\theta$','$\psi$'); latex_set(l_h);
            grid on; xlim([0, time_vec(end)]);
            
            subplot(2,1,2);
            plot(time_vec, obj.v_nu(:, valid_range), 'LineWidth', 2);
            t_h = title('Vehicle Velocity ($\nu$)'); latex_set(t_h);
            l_h = legend('$u$','$v$','$w$','$p$','$q$','$r$'); latex_set(l_h);
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
                    
                    % Escape underscores in task names
                    t_name = strrep(char(obj.tasks_ref{idx}.task_name), '_', '\_');
                    t_h = title(sprintf('Task %d: %s', idx, t_name)); latex_set(t_h);
                    
                    % Adaptive Legend for UVMS tasks
                    if size(data, 1) == 6
                        l_h = legend('$v_x$','$v_y$','$v_z$','$\omega_x$','$\omega_y$','$\omega_z$'); 
                        latex_set(l_h); set(l_h, 'NumColumns', 2);
                    elseif size(data, 1) == 3
                        l_h = legend('$x$','$y$','$z$'); % or orientation components
                        latex_set(l_h);
                    elseif size(data, 1) == 1
                        l_h = legend('Value');
                        latex_set(l_h);
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