classdef SimulationLogger < handle
    properties
        t            % time vector
        ql           % joint positions (Left)
        qdotl        % joint velocities (Left)
        qr           % joint positions (Right)
        qdotr        % joint velocities (Right)
        xdotbar_task % reference velocities
        robot        % robot model
        
        tasks_ref    % global task list
        n_tasks      % number of tasks
        a_task       % task activations data
        
        % --- ACTION PROPERTIES ---
        action_idx_log % History of action indices
        action_names   % List of action names
        n_actions      % Total number of distinct actions
        
        % --- TOOL DATA FOR ANALYSIS ---
        tool_pos_L
        tool_pos_R
        
        data_len     % valid data length
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
            obj.ql = zeros(7, maxLoops);
            obj.qdotl = zeros(7, maxLoops);
            obj.qr = zeros(7, maxLoops);
            obj.qdotr = zeros(7, maxLoops);
            
            obj.tool_pos_L = zeros(3, maxLoops);
            obj.tool_pos_R = zeros(3, maxLoops);
            
            obj.xdotbar_task = cell(obj.n_tasks, maxLoops);
            obj.a_task = cell(obj.n_tasks, maxLoops);
            
            % Pre-allocate Action Log
            obj.action_idx_log = ones(1, maxLoops); 
        end
        
        % --- UPDATE METHOD ---
        function update(obj, t, loop, current_action_idx, wTt_L, wTt_R)
            obj.t(loop) = t;
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;
            
            % Store Tool Positions if provided
            if nargin > 4
                obj.tool_pos_L(:, loop) = wTt_L;
                obj.tool_pos_R(:, loop) = wTt_R;
            end
            
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
        
        % --- PLOT TOOL DISTANCE ---
        function plotToolDistance(obj)
            valid_range = 1:obj.data_len;
            time_vec = obj.t(valid_range);
            t_max = time_vec(end);
            
            % FIXED: Same size as other subplots (visually)
            fig_pos = [100, 100, 800, 400]; 
            
            figure(301); clf; set(gcf, 'Position', fig_pos, 'Color', 'w', 'Name', 'Tool Distance');
            
            % Compute Euclidean Distance
            diff_vec = obj.tool_pos_L(:, valid_range) - obj.tool_pos_R(:, valid_range);
            dist_norm = sqrt(sum(diff_vec.^2, 1));
            
            ax1 = subplot(1,1,1);
            
            % Blue line [0 0.4470 0.7410]
            plot(time_vec, dist_norm, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]); 
            
            grid on; xlim([0, t_max]);
            
            t_h = title('\textbf{Distance between Left and Right Tool Frames}');
            xlabel(''); 
            
            obj.expand_ylim(ax1);
            obj.format_plot(t_h, [], ax1, 'Distance [m]');
            
            obj.add_action_bars(ax1, valid_range, t_max);
        end

        % --- PLOT ALL METHOD ---
        function plotAll(obj, task_selectors)
            % 1. Determine which tasks to plot
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
                        else
                            warning('Logger:TaskNotFound', 'Task "%s" not found. Skipping.', target_name);
                        end
                    end
                elseif isnumeric(task_selectors)
                    task_indices = task_selectors;
                else
                    error('Invalid selector. Use task names (strings) or indices.');
                end
            end
            
            if isempty(task_indices)
                warning('No valid tasks selected to plot.');
                return;
            end

            valid_range = 1:obj.data_len;
            time_vec = obj.t(valid_range);
            t_max = time_vec(end);
            
            fig_pos = [100, 100, 800, 600]; 
            
            % --- FIGURE 101: Left Arm State ---
            figure(101); clf; set(gcf, 'Position', fig_pos, 'Color', 'w', 'Name', 'Left Arm State');
            
            ax1 = subplot(2,1,1);
            plot(time_vec, obj.ql(:, valid_range), 'LineWidth', 2);
            t_h = title('\textbf{Left Arm - Joint Positions ($\mathbf{q}_L$)}'); 
            l_h = legend('$q_1$','$q_2$','$q_3$','$q_4$','$q_5$','$q_6$','$q_7$'); 
            grid on; xlim([0, t_max]);
            obj.expand_ylim(ax1); 
            obj.format_plot(t_h, l_h, ax1, 'Pos. [rad]');
            obj.add_action_bars(ax1, valid_range, t_max); 
            
            ax2 = subplot(2,1,2);
            plot(time_vec, obj.qdotl(:, valid_range), 'LineWidth', 2);
            t_h = title('\textbf{Left Arm - Joint Velocities ($\dot{\mathbf{q}}_L$)}'); 
            l_h = legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','$\dot{q}_7$'); 
            grid on; xlim([0, t_max]);
            obj.expand_ylim(ax2);
            obj.format_plot(t_h, l_h, ax2, 'Vel. [rad/s]'); 
            obj.add_action_bars(ax2, valid_range, t_max);
            
            % --- FIGURE 102: Right Arm State ---
            figure(102); clf; set(gcf, 'Position', fig_pos, 'Color', 'w', 'Name', 'Right Arm State');
            
            ax1 = subplot(2,1,1);
            plot(time_vec, obj.qr(:, valid_range), 'LineWidth', 2);
            t_h = title('\textbf{Right Arm - Joint Positions ($\mathbf{q}_R$)}'); 
            l_h = legend('$q_1$','$q_2$','$q_3$','$q_4$','$q_5$','$q_6$','$q_7$'); 
            grid on; xlim([0, t_max]);
            obj.expand_ylim(ax1);
            obj.format_plot(t_h, l_h, ax1, 'Pos. [rad]'); 
            obj.add_action_bars(ax1, valid_range, t_max);
            
            ax2 = subplot(2,1,2);
            plot(time_vec, obj.qdotr(:, valid_range), 'LineWidth', 2);
            t_h = title('\textbf{Right Arm - Joint Velocities ($\dot{\mathbf{q}}_R$)}'); 
            l_h = legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','$\dot{q}_7$'); 
            grid on; xlim([0, t_max]);
            obj.expand_ylim(ax2);
            obj.format_plot(t_h, l_h, ax2, 'Vel. [rad/s]');
            obj.add_action_bars(ax2, valid_range, t_max);
            
            % --- INDIVIDUAL TASK FIGURES ---
            nt = length(task_indices);
            
            for i = 1:nt
                idx = task_indices(i);
                
                t_name_raw = char(obj.tasks_ref{idx}.task_name);
                t_name = strrep(t_name_raw, '_', '\_'); 
                
                f_h = figure(200 + idx); clf;
                set(f_h, 'Name', t_name_raw);
                set(f_h, 'Position', fig_pos, 'Color', 'w'); 
                
                raw_ref = obj.xdotbar_task(idx, valid_range);
                raw_act = obj.a_task(idx, valid_range);
                
                if any(~cellfun(@isempty, raw_ref))
                    data_ref = cell2mat(raw_ref(~cellfun(@isempty, raw_ref)));
                    data_act = cell2mat(raw_act(~cellfun(@isempty, raw_act)));
                    t_plot = time_vec(~cellfun(@isempty, raw_ref));
                    dims = size(data_ref, 1);
                    
                    if dims > 7
                        c_order = turbo(dims);
                    else
                        c_order = lines(dims);
                    end
                    
                    % --- SUBPLOT 1: REFERENCE ---
                    ax1 = subplot(2,1,1);
                    set(gca, 'ColorOrder', c_order);
                    plot(t_plot, data_ref', 'LineWidth', 1.5);
                    grid on; xlim([0, t_max]);
                    
                    [leg_ref, y_lab_unit] = obj.generate_legends(dims, t_name_raw, 'Ref');
                    
                    % --- DYNAMIC TITLE LOGIC ---
                    if contains(t_name_raw, 'Tool', 'IgnoreCase', true)
                        % Tool Task: x_bar = w_x_bar_t
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}} = {}^w\\dot{\\bar{\\mathbf{x}}}_t$)}', t_name);
                    elseif contains(t_name_raw, 'Object', 'IgnoreCase', true)
                        % Object Task: x_bar = w_x_bar_o
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}} = {}^w\\dot{\\bar{\\mathbf{x}}}_o$)}', t_name);
                    elseif contains(t_name_raw, 'Joint', 'IgnoreCase', true) || contains(t_name_raw, 'Velocities', 'IgnoreCase', true)
                        % Joint/Limits: x_bar = q_bar
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}} = \\dot{\\bar{\\mathbf{q}}}$)}', t_name);
                    elseif contains(t_name_raw, 'Alt', 'IgnoreCase', true)
                        % Min Alt: x_bar = h_dot
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}} = \\dot{\\bar{h}}$)}', t_name);
                    else
                        % Default
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}}$)}', t_name);
                    end
                    
                    t_h = title(title_str);
                    
                    % Y-Label is ALWAYS just x_dot_bar [units]
                    y_lab_full = sprintf('$\\dot{\\bar{\\mathbf{x}}}$ %s', y_lab_unit);
                    
                    l_h = legend(leg_ref);
                    if dims > 4, set(l_h, 'NumColumns', 2); end
                    if dims > 7, set(l_h, 'NumColumns', 4); end 
                    
                    obj.expand_ylim(ax1); 
                    obj.format_plot(t_h, l_h, ax1, y_lab_full); 
                    obj.add_action_bars(ax1, valid_range, t_max); 
                    
                    % --- SUBPLOT 2: ACTIVATION ---
                    ax2 = subplot(2,1,2);
                    set(gca, 'ColorOrder', c_order);
                    plot(t_plot, data_act', 'LineWidth', 1.5);
                    grid on; xlim([0, t_max]); ylim([-0.1 1.1]);
                    
                    t_h = title(sprintf('\\textbf{%s - Activation ($\\mathbf{A}$)}', t_name));
                    
                    [leg_act, ~] = obj.generate_legends(dims, t_name_raw, 'Act');
                    
                    l_h = legend(leg_act);
                    if dims > 4, set(l_h, 'NumColumns', 2); end
                    if dims > 7, set(l_h, 'NumColumns', 4); end
                    
                    obj.format_plot(t_h, l_h, ax2, 'Activation');
                    obj.add_action_bars(ax2, valid_range, t_max); 
                end
            end
        end
    end
    
    methods (Access = private)
        % --- HELPER: ADD SEPARATE ACTION STRIP AXES ---
        function add_action_bars(obj, h_ax_data, valid_range, t_max)
            % Add an action strip axis under the provided data axis and draw colored
            % patches + labels representing the action history.
            %
            % Inputs:
            %   h_ax_data   - handle to the main data axes (top)
            %   valid_range - indices into obj.t / obj.action_idx_log that are valid
            %   t_max       - scalar maximum time for xlim
        
            % Safety checks
            if isempty(valid_range) || obj.data_len == 0
                return;
            end
        
            act_hist = obj.action_idx_log(valid_range);
            time_hist = obj.t(valid_range);
        
            if isempty(time_hist)
                return;
            end
        
            % Find contiguous segments of equal actions
            changes = find(diff(act_hist) ~= 0);
            start_indices = [1, changes + 1];
            end_indices = [changes, length(act_hist)];
        
            % Color map for actions (expand if you have more actions)
            colors = [
                0.88, 0.95, 1.00;   % soft blue
                1.00, 0.88, 0.88;   % soft red
                0.90, 1.00, 0.90;   % soft green
                0.95, 0.90, 0.95;   % lavender
            ];
        
            % --- Resize top data axes to make room for the strip ---
            pos_data = get(h_ax_data, 'Position');   % [left bottom width height]
        
            strip_frac = 0.08;                        % fraction of original height used for strip
            h_strip = pos_data(4) * strip_frac;
            h_data_new = pos_data(4) * (1 - strip_frac);
        
            set(h_ax_data, 'Position', [pos_data(1), pos_data(2) + h_strip, pos_data(3), h_data_new]);
        
            % --- Hide x ticks/labels on main data axes so they don't peek through ---
            % Keep grid and other visuals intact but remove x-axis ticks and numbers.
            set(h_ax_data, 'XTick', [], 'XTickLabel', []);
        
            % --- Create strip axes directly below the data axes ---
            h_ax_strip = axes('Position', [pos_data(1), pos_data(2), pos_data(3), h_strip]);
            hold(h_ax_strip, 'on');
        
            % Configure strip axis appearance
            set(h_ax_strip, ...
                'YLim', [0, 1], ...
                'XLim', [0, t_max], ...
                'YTick', [], ...
                'Box', 'on', ...
                'Layer', 'top', ...
                'XGrid', 'off', ...
                'YGrid', 'off', ...
                'TickLabelInterpreter', 'latex', ...
                'FontSize', 12);
        
            xlabel(h_ax_strip, 'Time [s]', 'Interpreter', 'latex', 'FontSize', 14);
        
            % Ensure strip axis is on top visually and data axis below
            uistack(h_ax_strip, 'top');
            uistack(h_ax_data,  'bottom');
        
            % Link x-axes so zoom/pan stays synced
            linkaxes([h_ax_data, h_ax_strip], 'x');
        
            % Preallocate handle arrays
            text_handles = gobjects(0,1);
            patch_handles = gobjects(length(start_indices),1);
        
            % Loop over segments and draw patches + text labels (if available)
            for k = 1:length(start_indices)
                idx_start = start_indices(k);
                idx_end = end_indices(k);
                act_id = act_hist(idx_start);
        
                % Choose color (fallback if act_id outside palette)
                if act_id > 0 && act_id <= size(colors,1)
                    col = colors(act_id, :);
                else
                    col = [0.95, 0.95, 0.95]; % neutral grey
                end
        
                % Start/end times (use actual segment end index)
                t_start = time_hist(idx_start);
                % Protect indexing just in case
                if idx_end <= numel(time_hist)
                    t_end = time_hist(idx_end);
                else
                    t_end = t_max;
                end
        
                % Draw semi-transparent patch for the segment
                patch_handles(k) = patch(h_ax_strip, [t_start t_end t_end t_start], [0 0 1 1], ...
                    col, 'FaceAlpha', 0.75, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        
                % If there is a matching action name, draw it centered on the patch
                if act_id > 0 && act_id <= length(obj.action_names)
                    act_name = obj.action_names{act_id};
                    act_name = strrep(act_name, '_', '\_'); % escape underscores for latex
        
                    th = text(h_ax_strip, (t_start + t_end)/2, 0.5, act_name, ...
                        'Units', 'data', ...
                        'VerticalAlignment', 'middle', ...
                        'HorizontalAlignment', 'center', ...
                        'FontSize', 10, ...
                        'FontWeight', 'bold', ...
                        'Interpreter', 'latex', ...
                        'Color', [0 0 0]);
        
                    text_handles(end+1,1) = th; %#ok<AGROW>
                end
            end
        
            % Send all patch graphics to the bottom of the strip axes so text and ticks sit above them
            for p = 1:numel(patch_handles)
                if isgraphics(patch_handles(p))
                    uistack(patch_handles(p), 'bottom');
                end
            end
        
            % Bring text handles to top
            if ~isempty(text_handles)
                for t = 1:numel(text_handles)
                    if isgraphics(text_handles(t))
                        uistack(text_handles(t), 'top');
                    end
                end
            end
        
            drawnow; % Ensure the renderer updates stacking and labels immediately
        end

        
        function expand_ylim(~, h_ax)
            current_lims = ylim(h_ax);
            y_min = current_lims(1);
            y_max = current_lims(2);
            range = y_max - y_min;
            if range < 1e-6, range = 1.0; end
            margin = range * 0.05; 
            ylim(h_ax, [y_min - margin, y_max + margin]);
        end

        function [legends, y_label_unit] = generate_legends(~, dims, name, type)
            is_ref = strcmp(type, 'Ref');
            is_alt = contains(lower(name), 'alt') && dims == 1;
            is_joint = dims == 7 || dims == 14; 
            
            suffix = '';
            if contains(name, 'Left', 'IgnoreCase', true), suffix = ',L'; % CHANGED: Added comma
            elseif contains(name, 'Right', 'IgnoreCase', true), suffix = ',R'; end % CHANGED: Added comma
            
            legends = {};
            y_label_unit = '[unit]';
            
            if is_alt
                y_label_unit = '[m/s]';
                % CHANGED: v_z to h_dot
                if is_ref, legends = {'$\dot{\bar{h}}$'}; else, legends = {'$\alpha$'}; end
                
            elseif dims == 6
                y_label_unit = '[rad/s, m/s]';
                if is_ref
                    % CHANGED: v_x -> x_dot
                    legends = {'$\omega_x$','$\omega_y$','$\omega_z$','$\dot{x}$','$\dot{y}$','$\dot{z}$'};
                else
                    legends = {'$\alpha_{\omega_x}$','$\alpha_{\omega_y}$','$\alpha_{\omega_z}$','$\alpha_{\dot{x}}$','$\alpha_{\dot{y}}$','$\alpha_{\dot{z}}$'};
                end
                
            elseif is_joint && dims == 7
                y_label_unit = '[rad/s]';
                if isempty(suffix)
                    if is_ref, legends = arrayfun(@(x) sprintf('$\\dot{q}_{%d}$',x), 1:7, 'UniformOutput', false);
                    else,      legends = arrayfun(@(x) sprintf('$\\alpha_{%d}$',x), 1:7, 'UniformOutput', false); end
                else
                    % Suffix now has comma included
                    if is_ref, legends = arrayfun(@(x) sprintf('$\\dot{q}_{%d%s}$',x,suffix), 1:7, 'UniformOutput', false);
                    else,      legends = arrayfun(@(x) sprintf('$\\alpha_{%d%s}$',x,suffix), 1:7, 'UniformOutput', false); end
                end
                
            elseif is_joint && dims == 14
                y_label_unit = '[rad/s]';
                if is_ref
                    % CHANGED: Added commas
                    legends = [arrayfun(@(x) sprintf('$\\dot{q}_{%d,L}$',x), 1:7, 'UniformOutput', false), ...
                               arrayfun(@(x) sprintf('$\\dot{q}_{%d,R}$',x), 1:7, 'UniformOutput', false)];
                else
                    legends = [arrayfun(@(x) sprintf('$\\alpha_{%d,L}$',x), 1:7, 'UniformOutput', false), ...
                               arrayfun(@(x) sprintf('$\\alpha_{%d,R}$',x), 1:7, 'UniformOutput', false)];
                end
                
            elseif dims == 12
                y_label_unit = '[rad/s, m/s]';
                if is_ref
                    % CHANGED: v -> x_dot
                    legends = {'$\omega_{x,L}$','$\omega_{y,L}$','$\omega_{z,L}$','$\dot{x}_L$','$\dot{y}_L$','$\dot{z}_L$', ...
                               '$\omega_{x,R}$','$\omega_{y,R}$','$\omega_{z,R}$','$\dot{x}_R$','$\dot{y}_R$','$\dot{z}_R$'};
                else
                    legends = {'$\alpha_{\omega x,L}$','$\alpha_{\omega y,L}$','$\alpha_{\omega z,L}$','$\alpha_{vx,L}$','$\alpha_{vy,L}$','$\alpha_{vz,L}$', ...
                               '$\alpha_{\omega x,R}$','$\alpha_{\omega y,R}$','$\alpha_{\omega z,R}$','$\alpha_{vx,R}$','$\alpha_{vy,R}$','$\alpha_{vz,R}$'};
                end
                
            else % Fallback
                y_label_unit = '[unit]';
                if is_ref, legends = arrayfun(@(x) sprintf('$\\dot{x}_{%d}$',x), 1:dims, 'UniformOutput', false);
                else,      legends = arrayfun(@(x) sprintf('$\\alpha_{%d}$',x), 1:dims, 'UniformOutput', false); end
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