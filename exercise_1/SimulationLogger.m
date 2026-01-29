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
        xdotbar_task % reference velocities (cell array)
        a_task       % task activations data (cell array)
        
        % --- ACTION PROPERTIES ---
        action_idx_log % History of action indices
        action_names   % List of action names
        n_actions      % Total number of distinct actions
        
        data_len     % valid data length tracking
    end
    
    methods
        function obj = SimulationLogger(maxLoops, robotModel, global_tasks, action_names_list)
            obj.robot = robotModel;
            obj.tasks_ref = global_tasks; 
            obj.n_tasks = length(global_tasks);
            
            % Save Action Names
            if nargin < 4 || isempty(action_names_list)
                obj.action_names = {};
                obj.n_actions = 0;
            else
                obj.action_names = action_names_list;
                obj.n_actions = length(action_names_list);
            end
            
            obj.data_len = 0;
            
            % Memory Pre-allocation
            obj.t = zeros(1, maxLoops);
            obj.q = zeros(7, maxLoops);
            obj.qdot = zeros(7, maxLoops);
            obj.eta = zeros(6, maxLoops);
            obj.v_nu = zeros(6, maxLoops);
            
            obj.xdotbar_task = cell(obj.n_tasks, maxLoops);
            obj.a_task = cell(obj.n_tasks, maxLoops);
            
            % Pre-allocate Action Log
            obj.action_idx_log = ones(1, maxLoops); 
        end
        
        % --- UPDATE METHOD ---
        function update(obj, t, loop, current_action_idx)
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
            
            % Log the current action
            if nargin < 4 || isempty(current_action_idx)
                current_action_idx = 1; 
            end
            obj.action_idx_log(loop) = current_action_idx;
            
            obj.data_len = loop;
        end
        
        % --- PLOT ALL METHOD (MODIFICATO) ---
        function plotAll(obj, task_selectors)
            % Determine which tasks to plot
            if nargin < 2 || isempty(task_selectors)
                task_indices = 1:obj.n_tasks;
            else
                 if ischar(task_selectors) || isstring(task_selectors) || iscell(task_selectors)
                    if ischar(task_selectors) || isstring(task_selectors)
                        names_to_find = cellstr(task_selectors);
                    else
                        names_to_find = task_selectors;
                    end
                    task_indices = [];
                    for k = 1:length(names_to_find)
                        target_name = names_to_find{k};
                        found_idx = [];
                        for i = 1:obj.n_tasks
                            if strcmpi(obj.tasks_ref{i}.task_name, target_name)
                                found_idx = i;
                                break;
                            end
                        end
                        if ~isempty(found_idx)
                            task_indices(end+1) = found_idx;
                        end
                    end
                elseif isnumeric(task_selectors)
                    task_indices = task_selectors;
                end
            end
            
            valid_range = 1:obj.data_len;
            time_vec = obj.t(valid_range);
            t_max = time_vec(end);
            
            fig_pos = [100, 100, 800, 600];
            
            % --- FIGURE 101: Arm State ---
            figure(101); clf; set(gcf, 'Position', fig_pos, 'Color', 'w', 'Name', 'Arm State');
            
            ax1 = subplot(2,1,1);
            plot(time_vec, obj.q(:, valid_range), 'LineWidth', 2);
            t_h = title('\textbf{Manipulator - Joint Positions ($\mathbf{q}$)}'); 
            l_h = legend('$q_1$','$q_2$','$q_3$','$q_4$','$q_5$','$q_6$','$q_7$'); 
            grid on; xlim([0, t_max]);
            obj.expand_ylim(ax1); 
            obj.format_plot(t_h, l_h, ax1, 'Pos. [rad]');
            obj.add_action_bars(ax1, valid_range, t_max); 
            
            ax2 = subplot(2,1,2);
            plot(time_vec, obj.qdot(:, valid_range), 'LineWidth', 2);
            t_h = title('\textbf{Manipulator - Joint Velocities ($\dot{\mathbf{q}}$)}'); 
            l_h = legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','$\dot{q}_7$'); 
            grid on; xlim([0, t_max]);
            obj.expand_ylim(ax2);
            obj.format_plot(t_h, l_h, ax2, 'Vel. [rad/s]'); 
            obj.add_action_bars(ax2, valid_range, t_max);
            
            % --- FIGURE 102: Vehicle State ---
            figure(102); clf; set(gcf, 'Position', fig_pos, 'Color', 'w', 'Name', 'Vehicle State');
            
            ax1 = subplot(2,1,1);
            plot(time_vec, obj.eta(:, valid_range), 'LineWidth', 2);
            t_h = title('\textbf{Vehicle Pose ($\eta$)}'); 
            l_h = legend('$x$','$y$','$z$','$\phi$','$\theta$','$\psi$'); 
            grid on; xlim([0, t_max]);
            obj.expand_ylim(ax1); 
            obj.format_plot(t_h, l_h, ax1, 'Pose [m, rad]');
            obj.add_action_bars(ax1, valid_range, t_max); 
            
            ax2 = subplot(2,1,2);
            plot(time_vec, obj.v_nu(:, valid_range), 'LineWidth', 2);
            t_h = title('\textbf{Vehicle Velocity ($\nu$)}'); 
            l_h = legend('$\dot{x}$','$\dot{y}$','$\dot{z}$','$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$'); 
            grid on; xlim([0, t_max]);
            obj.expand_ylim(ax2);
            obj.format_plot(t_h, l_h, ax2, 'Vel. [m/s, rad/s]'); 
            obj.add_action_bars(ax2, valid_range, t_max);
            
            % --- INDIVIDUAL TASK FIGURES ---
            nt = length(task_indices);
            
            for i = 1:nt
                idx = task_indices(i);
                
                t_name_raw = char(obj.tasks_ref{idx}.task_name);
                % Escape underscores for LaTeX
                t_name_safe = strrep(t_name_raw, '_', '\_');
                t_name_lower = lower(t_name_raw);
                
                f_h = figure(200 + idx); clf;
                set(f_h, 'Name', t_name_raw, 'Position', fig_pos, 'Color', 'w');
                
                raw_ref = obj.xdotbar_task(idx, valid_range);
                raw_act = obj.a_task(idx, valid_range);
                
                if any(~cellfun(@isempty, raw_ref))
                    data_ref = cell2mat(raw_ref(~cellfun(@isempty, raw_ref)));
                    data_act = cell2mat(raw_act(~cellfun(@isempty, raw_act)));
                    t_plot = time_vec(~cellfun(@isempty, raw_ref));
                    dims = size(data_ref, 1);
                    
                    if dims > 7; c_order = turbo(dims); else; c_order = lines(dims); end
                    
                    % 1. DETERMINAZIONE DEL FRAME (Veicolo vs Mondo)
                    if contains(t_name_lower, 'horizontal') || ...
                       contains(t_name_lower, 'alignment') || ...
                       contains(t_name_lower, 'tool')
                        frame_str = '{}^v'; % Frame Veicolo
                    else
                        frame_str = '{}^w'; % Frame World (Default)
                    end
                    
                    % 2. DETERMINAZIONE SUFFISSO OPZIONALE (es. _t per tool)
                    suffix_str = '';
                    if contains(t_name_lower, 'tool')
                        suffix_str = '_t';
                    end
                    
                    % 3. SIMBOLO VARIABILE: Sempre 'x' come richiesto
                    var_sym = '\mathbf{x}';
                    var_sym_2 = var_sym;
                    if contains(t_name_lower, 'horizontal')
                        var_sym_2 = '\theta';
                    elseif contains(t_name_lower, 'workspace')
                        var_sym_2 = 'd_{xy}';
                    elseif contains(t_name_lower, 'altitude')
                        var_sym_2 = 'h';
                    end

                    
                    % --- SUBPLOT 1: REFERENCE ---
                    ax1 = subplot(2,1,1);
                    set(gca, 'ColorOrder', c_order);
                    plot(t_plot, data_ref', 'LineWidth', 1.5);
                    grid on; xlim([0, t_max]);
                    
                    % Generate adapted legends for UVMS based on Task Name
                    [leg_ref, y_lab_unit] = obj.generate_legends(dims, t_name_raw, 'Ref');
                    
                    % COSTRUZIONE TITOLO
                    % Esempio Tool: Reference ( dot{bar{x}} = {}^v dot{bar{x}}_t )
                    % Esempio Pos:  Reference ( dot{bar{x}} = {}^w dot{bar{x}} )
                    title_str = sprintf('\\textbf{%s - Reference ($ \\dot{\\bar{%s}} = %s\\dot{\\bar{%s}}%s$)}', ...
                                        t_name_safe, var_sym, frame_str, var_sym_2, suffix_str);
                    t_h = title(title_str);
                    
                    % Y Label SEMPRE x dot bar
                    y_lab_full = sprintf('$\\dot{\\bar{%s}}$ %s', var_sym, y_lab_unit);
                    
                    l_h = legend(leg_ref);
                    if dims > 4, set(l_h, 'NumColumns', 2); end
                    
                    obj.expand_ylim(ax1); 
                    obj.format_plot(t_h, l_h, ax1, y_lab_full); 
                    obj.add_action_bars(ax1, valid_range, t_max); 
                    
                    % --- SUBPLOT 2: ACTIVATION ---
                    ax2 = subplot(2,1,2);
                    set(gca, 'ColorOrder', c_order);
                    plot(t_plot, data_act', 'LineWidth', 1.5);
                    grid on; xlim([0, t_max]); ylim([-0.1 1.1]);
                    
                    t_h = title(sprintf('\\textbf{%s - Activation ($\\mathbf{A}$)}', t_name_safe));
                    [leg_act, ~] = obj.generate_legends(dims, t_name_raw, 'Act');
                    
                    l_h = legend(leg_act);
                    if dims > 4, set(l_h, 'NumColumns', 2); end
                    
                    obj.format_plot(t_h, l_h, ax2, 'Activation');
                    obj.add_action_bars(ax2, valid_range, t_max); 
                end
            end
        end
    end
    
    methods (Access = private)
        % --- HELPER: ADD SEPARATE ACTION STRIP AXES (CORRETTO PER LATEX) ---
        function add_action_bars(obj, h_ax_data, valid_range, t_max)
            if isempty(valid_range) || obj.data_len == 0, return; end
            act_hist = obj.action_idx_log(valid_range);
            time_hist = obj.t(valid_range);
            if isempty(time_hist), return; end
            
            changes = find(diff(act_hist) ~= 0);
            start_indices = [1, changes + 1];
            end_indices = [changes, length(act_hist)];
            
            colors = [0.88 0.95 1.00; 1.00 0.88 0.88; 0.90 1.00 0.90; 0.95 0.90 0.95; 1.00 0.95 0.80];
            
            pos_data = get(h_ax_data, 'Position'); 
            strip_frac = 0.08; h_strip = pos_data(4) * strip_frac;
            
            % Riduci l'asse dei dati e nascondi i suoi numeri X
            set(h_ax_data, 'Position', [pos_data(1), pos_data(2)+h_strip, pos_data(3), pos_data(4)*(1-strip_frac)]);
            set(h_ax_data, 'XTickLabel', []);
            
            % Crea l'asse della striscia
            h_ax_strip = axes('Position', [pos_data(1), pos_data(2), pos_data(3), h_strip]);
            hold(h_ax_strip, 'on');
            
            % Imposta TickLabelInterpreter LATEX sul nuovo asse
            set(h_ax_strip, ...
                'YLim', [0, 1], ...
                'XLim', [0, t_max], ...
                'YTick', [], ...
                'Box', 'on', ...
                'Layer', 'top', ...
                'TickLabelInterpreter', 'latex', ... 
                'FontSize', 12);
            
            xlabel(h_ax_strip, 'Time [s]', 'Interpreter', 'latex', 'FontSize', 12);
            linkaxes([h_ax_data, h_ax_strip], 'x');
            
            for k = 1:length(start_indices)
                idx_start = start_indices(k); idx_end = end_indices(k);
                act_id = act_hist(idx_start);
                col = [0.95, 0.95, 0.95];
                if act_id > 0 && act_id <= size(colors,1), col = colors(act_id, :); end
                
                t_s = time_hist(idx_start); t_e = time_hist(idx_end);
                patch(h_ax_strip, [t_s t_e t_e t_s], [0 0 1 1], col, 'FaceAlpha', 0.8, 'EdgeColor', 'none');
                
                if act_id > 0 && act_id <= length(obj.action_names)
                    text(h_ax_strip, (t_s+t_e)/2, 0.5, strrep(obj.action_names{act_id},'_','\_'), ...
                        'HorizontalAlignment','center','Interpreter','latex','FontSize',10, 'FontWeight', 'bold');
                end
            end
        end

        function expand_ylim(~, h_ax)
            lims = ylim(h_ax); margin = (lims(2)-lims(1))*0.05;
            if margin < 1e-6, margin = 1; end
            ylim(h_ax, [lims(1)-margin, lims(2)+margin]);
        end

        % --- LEGEND GENERATION LOGIC ---
        function [legends, y_label_unit] = generate_legends(~, dims, name, type)
            is_ref = strcmp(type, 'Ref');
            name_clean = lower(name); 
            
            y_label_unit = '[unit]';
            % Default fallback
            legends = arrayfun(@(x) sprintf('$%d$',x), 1:dims, 'UniformOutput', false);

            % --- 1. TaskVehiclePos (Linear 3D) ---
            if contains(name_clean, 'vehicle') && contains(name_clean, 'pos')
                y_label_unit = '[m/s]';
                if is_ref, legends = {'$v_x$', '$v_y$', '$v_z$'};
                else, legends = {'$\alpha_{v_{x}}$', '$\alpha_{v_{y}}$', '$\alpha_{v_{z}}$'}; end

            % --- 2. TaskVehicleOrient (Angular 3D) ---
            elseif contains(name_clean, 'vehicle') && contains(name_clean, 'orient')
                y_label_unit = '[rad/s]';
                if is_ref, legends = {'$\omega_x$', '$\omega_y$', '$\omega_z$'};
                else, legends = {'$\alpha_{\omega_{x}}$', '$\alpha_{\omega_{y}}$', '$\alpha_{\omega_{z}}$'}; end
                
            % --- 3. TaskAlignment (Angular 3D) ---
            elseif contains(name_clean, 'alignment')
                y_label_unit = '[rad/s]';
                if is_ref, legends = {'$\omega_x$', '$\omega_y$', '$\omega_z$'};
                else, legends = {'$\alpha_{\omega_{x}}$', '$\alpha_{\omega_{y}}$', '$\alpha_{\omega_{z}}$'}; end
                
            % --- 4. TaskHorizontalAtt (Scalar - Theta) ---
            elseif contains(name_clean, 'horizontal')
                y_label_unit = '[rad/s]';
                if is_ref
                    legends = {'$\dot{\bar{\theta}}$'};
                else
                    legends = {'$\alpha$'}; 
                end
                
            % --- 5. TaskWorkSpace (Scalar - Distanza) ---
            elseif contains(name_clean, 'workspace')
                y_label_unit = '[m/s]';
                if is_ref
                    legends = {'$\dot{\bar{d}}_{xy}$'};
                else
                    legends = {'$\alpha$'}; 
                end

            % --- 6. TaskTool (6D: Angular first, then Linear) ---
            elseif contains(name_clean, 'tool')
                y_label_unit = '[rad/s, m/s]';
                if is_ref
                    legends = {'$\omega_x$', '$\omega_y$', '$\omega_z$', '$v_x$', '$v_y$', '$v_z$'};
                else
                    legends = {'$\alpha_{\omega_{x}}$', '$\alpha_{\omega_{y}}$', '$\alpha_{\omega_{z}}$', '$\alpha_{v_{x}}$', '$\alpha_{v_{y}}$', '$\alpha_{v_{z}}$'}; 
                end

            % --- 7. TaskStop (6D: Linear first, then Angular) ---
            elseif contains(name_clean, 'stop')
                y_label_unit = '[m/s, rad/s]';
                if is_ref
                     legends = {'$v_x$','$v_y$','$v_z$','$\omega_x$','$\omega_y$','$\omega_z$'};
                else
                     legends = {'$\alpha_{v_{x}}$','$\alpha_{v_{y}}$','$\alpha_{v_{z}}$','$\alpha_{\omega_{x}}$','$\alpha_{\omega_{y}}$','$\alpha_{\omega_{z}}$'};
                end

            % --- GENERAL FALLBACKS ---
            elseif dims == 1
                y_label_unit = '[m/s]';
                if is_ref, legends = {'$\dot{\bar{h}}$'}; else, legends = {'$\alpha$'}; end
            elseif dims == 3
                y_label_unit = '[m/s]';
                if is_ref, legends = {'$\dot{x}$','$\dot{y}$','$\dot{z}$'};
                else, legends = {'$\alpha_x$','$\alpha_y$','$\alpha_z$'}; end
            elseif dims == 7
                y_label_unit = '[rad/s]';
                if is_ref, legends = arrayfun(@(x) sprintf('$\\dot{q}_{%d}$',x), 1:7, 'UniformOutput', false);
                else, legends = arrayfun(@(x) sprintf('$\\alpha_{%d}$',x), 1:7, 'UniformOutput', false); end
            end
        end

        function format_plot(~, h_title, h_legend, h_ax, y_unit)
            set(h_title, 'Interpreter', 'latex', 'FontSize', 16);
            if ~isempty(h_legend), set(h_legend, 'Interpreter', 'latex', 'FontSize', 12); end
            set(h_ax, 'TickLabelInterpreter', 'latex', 'FontSize', 12);
            ylabel(h_ax, y_unit, 'Interpreter', 'latex', 'FontSize', 14);
        end
    end
end