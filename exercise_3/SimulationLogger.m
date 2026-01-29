classdef SimulationLogger < handle
    properties
        t               % time vector
        ql              % joint positions (Left)
        qdotl           % joint velocities (Left)
        qr              % joint positions (Right)
        qdotr           % joint velocities (Right)
        xdotbar_task    % reference velocities
        robot           % robot model
        
        tasks_ref       % global task list
        n_tasks         % number of tasks
        a_task          % task activations data
        
        action_idx_log  % History of action indices
        action_names    % List of action names
        n_actions       % Total number of distinct actions
        tool_pos_L
        tool_pos_R
        
        data_len        % valid data length
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
            
            % Allocate stuff
            obj.t = zeros(1, maxLoops);
            obj.ql = zeros(7, maxLoops);
            obj.qdotl = zeros(7, maxLoops);
            obj.qr = zeros(7, maxLoops);
            obj.qdotr = zeros(7, maxLoops);
            
            obj.tool_pos_L = zeros(3, maxLoops);
            obj.tool_pos_R = zeros(3, maxLoops);
           
            obj.xdotbar_task = cell(obj.n_tasks, maxLoops);
            obj.a_task = cell(obj.n_tasks, maxLoops);
            obj.action_idx_log = ones(1, maxLoops); 
        end
        

        function update(obj, t, loop, current_action_idx, wTt_L, wTt_R)
        
            % Number of input arguments (includes obj)
            numArgs = nargin;
        
            % Basic state logger
            obj.t(loop) = t;
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;
        
            % Store Tool Positions if provided (arg#5 = wTt_L, arg#6 = wTt_R)
            if numArgs >= 5 && ~isempty(wTt_L) && numel(wTt_L) >= 3
                obj.tool_pos_L(:, loop) = wTt_L(:);
            else
                obj.tool_pos_L(:, loop) = [nan; nan; nan];
            end
            if numArgs >= 6 && ~isempty(wTt_R) && numel(wTt_R) >= 3
                obj.tool_pos_R(:, loop) = wTt_R(:);
            else
                obj.tool_pos_R(:, loop) = [nan; nan; nan];
            end
        
            % Tasks references / activations (ALWAYS update from task objects)
            for i = 1:obj.n_tasks
                obj.xdotbar_task{i, loop} = obj.tasks_ref{i}.xdotbar;
                % Use diag to store vector of activations if A is diagonal matrix
                obj.a_task{i, loop} = diag(obj.tasks_ref{i}.A);
            end
        
            if numArgs < 4 || isempty(current_action_idx)
                current_action_idx = 1;
            end
            obj.action_idx_log(loop) = current_action_idx;
        
            % Update data length
            obj.data_len = loop;
        end
      

        % PLOT TOOL DISTANCE
        function plotToolDistance(obj)
            valid_range = 1:obj.data_len;
            if isempty(valid_range), return; end

            time_vec = obj.t(valid_range);
            t_max = time_vec(end);
            
            fig_pos = [100, 100, 800, 400]; 
            
            figure(301); clf; set(gcf, 'Position', fig_pos, 'Color', 'w', 'Name', 'Tool Distance');
            
            % Compute Euclidean Distance
            diff_vec = obj.tool_pos_L(:, valid_range) - obj.tool_pos_R(:, valid_range);
            dist_norm = sqrt(sum(diff_vec.^2, 1));
            
            ax1 = subplot(1,1,1);
            
            plot(time_vec, dist_norm, 'LineWidth', 2); 
            
            grid on; xlim([0, t_max]);
            
            t_h = title('\textbf{Distance between Left and Right Tool Frames}');
            xlabel('');             % main x label is added by action strip
            
            obj.expand_ylim(ax1);
            obj.format_plot(t_h, [], ax1, 'Distance [m]');
            
            obj.add_action_bars(ax1, valid_range, t_max);
        end
        

        function plotAll(obj, task_selectors)
            % Which tasks to plot
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
            
            % Individual tasks
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
                    
                    % reference on top and activation on bottom
                    ax1 = subplot(2,1,1);
                    set(gca, 'ColorOrder', c_order);
                    plot(t_plot, data_ref', 'LineWidth', 1.5);
                    grid on; xlim([0, t_max]);
                    
                    [leg_ref, y_lab_unit] = obj.generate_legends(dims, t_name_raw, 'Ref');
                    
                    % Set title depending on which task we are plotting
                    if contains(t_name_raw, 'Tool', 'IgnoreCase', true)
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}} = {}^w\\dot{\\bar{\\mathbf{x}}}_t$)}', t_name);

                    elseif contains(t_name_raw, 'Cooperative', 'IgnoreCase', true) && contains(t_name_raw, 'Object', 'IgnoreCase', true)
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}} = {}^w\\dot{\\tilde{\\mathbf{x}}}_o$)}', t_name);

                    elseif contains(t_name_raw, 'Object', 'IgnoreCase', true)
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}} = {}^w\\dot{\\bar{\\mathbf{x}}}_o$)}', t_name);
                    
                    elseif contains(t_name_raw, 'Joint', 'IgnoreCase', true) || contains(t_name_raw, 'Velocities', 'IgnoreCase', true)
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}} = \\dot{\\bar{\\mathbf{q}}}$)}', t_name);
                    
                    elseif contains(t_name_raw, 'Alt', 'IgnoreCase', true)
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}} = \\dot{\\bar{h}}$)}', t_name);
                    
                    else        % default
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}}$)}', t_name);
                    end
                    
                    t_h = title(title_str);
                    
                    y_lab_full = sprintf('$\\dot{\\bar{\\mathbf{x}}}$ %s', y_lab_unit);
                    
                    l_h = legend(leg_ref);
                    if dims > 4, set(l_h, 'NumColumns', 2); end
                    if dims > 7, set(l_h, 'NumColumns', 4); end 
                    
                    obj.expand_ylim(ax1); 
                    obj.format_plot(t_h, l_h, ax1, y_lab_full); 
                    obj.add_action_bars(ax1, valid_range, t_max); 
                    
                    % Activation subplot
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



        % helper function to add blocks with action names
        function add_action_bars(obj, h_ax_data, valid_range, t_max)
        
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
        
            % Color map for actions
            action_colors = [
                0.88, 0.95, 1.00; 
                1.00, 0.88, 0.88;  
                0.90, 1.00, 0.90;  
                0.95, 0.90, 0.95;  
            ];
        
            % Resize top data axes to make room for the strip 
            pos_data = get(h_ax_data, 'Position');      % [left bottom width height]
        
            strip_frac = 0.08;                          % fraction of original height used for strip
            h_strip = pos_data(4) * strip_frac;
            h_data_new = pos_data(4) * (1 - strip_frac);
        
            set(h_ax_data, 'Position', [pos_data(1), pos_data(2) + h_strip, pos_data(3), h_data_new]);
        
            % Hide x ticks/labels on main data axes so they don't peek through
            set(h_ax_data, 'XTick', [], 'XTickLabel', []);
        
            % Create strip axes 
            h_ax_strip = axes('Position', [pos_data(1), pos_data(2), pos_data(3), h_strip]);
            hold(h_ax_strip, 'on');
        
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
            text_handles = gobjects(0,1);
            patch_handles = gobjects(length(start_indices),1);
        
            % Loop over segments and draw patches and text label
            for k = 1:length(start_indices)
                idx_start = start_indices(k);
                idx_end = end_indices(k);
                act_id = act_hist(idx_start);
        
                % Choose color (fallback if act_id outside palette)
                if act_id > 0 && act_id <= size(action_colors,1)
                    col = action_colors(act_id, :);
                else
                    col = [0.95, 0.95, 0.95]; % neutral grey
                end
        
                % Start/end times
                t_start = time_hist(idx_start);
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
                    act_name = strrep(act_name, '_', '\_');
        
                    th = text(h_ax_strip, (t_start + t_end)/2, 0.5, act_name, ...
                        'Units', 'data', ...
                        'VerticalAlignment', 'middle', ...
                        'HorizontalAlignment', 'center', ...
                        'FontSize', 10, ...
                        'FontWeight', 'bold', ...
                        'Interpreter', 'latex', ...
                        'Color', [0 0 0]);
        
                    text_handles(end+1,1) = th; 
                end
            end
        
            % move all patches to the bottom
            for p = 1:numel(patch_handles)
                if isgraphics(patch_handles(p))
                    uistack(patch_handles(p), 'bottom');
                end
            end
        
            % move text handles to top
            if ~isempty(text_handles)
                for t = 1:numel(text_handles)
                    if isgraphics(text_handles(t))
                        uistack(text_handles(t), 'top');
                    end
                end
            end
        
            drawnow;
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
            
            % Determine suffix based on task name
            suffix = '';
            if contains(name, 'Left', 'IgnoreCase', true), suffix = 'L';
            elseif contains(name, 'Right', 'IgnoreCase', true), suffix = 'R'; end 
            
            legends = {};
            y_label_unit = '[unit]';
            
            % Altitude 1D
            if is_alt
                y_label_unit = '[m/s]';
                if is_ref, legends = {'$\dot{\bar{h}}$'}; else, legends = {'$\alpha$'}; end
                
            % Cartesian 6D
            elseif dims == 6
                y_label_unit = '[rad/s, m/s]';
                if is_ref
                    legends = { ...
                        sprintf('$\\omega_{x,%s}$', suffix), sprintf('$\\omega_{y,%s}$', suffix), sprintf('$\\omega_{z,%s}$', suffix), ...
                        sprintf('$\\dot{x}_{%s}$', suffix), sprintf('$\\dot{y}_{%s}$', suffix), sprintf('$\\dot{z}_{%s}$', suffix)};
                else
                    legends = { ...
                        sprintf('$\\alpha_{\\omega_x,%s}$', suffix), sprintf('$\\alpha_{\\omega_y,%s}$', suffix), sprintf('$\\alpha_{\\omega_z,%s}$', suffix), ...
                        sprintf('$\\alpha_{\\dot{x},%s}$', suffix), sprintf('$\\alpha_{\\dot{y},%s}$', suffix), sprintf('$\\alpha_{\\dot{z},%s}$', suffix)};
                end
                
            % Joints 7D
            elseif dims == 7
                y_label_unit = '[rad/s]';
                if isempty(suffix)
                    % No L/R suffix found
                    if is_ref, legends = arrayfun(@(x) sprintf('$\\dot{q}_{%d}$',x), 1:7, 'UniformOutput', false);
                    else,      legends = arrayfun(@(x) sprintf('$\\alpha_{%d}$',x), 1:7, 'UniformOutput', false); end
                else
                    % Suffix exists add comma
                    if is_ref
                        legends = arrayfun(@(x) sprintf('$\\dot{q}_{%d,%s}$',x,suffix), 1:7, 'UniformOutput', false);
                    else
                        legends = arrayfun(@(x) sprintf('$\\alpha_{%d,%s}$',x,suffix), 1:7, 'UniformOutput', false); 
                    end
                end
                

            else 
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