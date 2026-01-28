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
        
        action_idx_log % History of action indices
        action_names   % List of action names
        n_actions      % Total number of distinct actions

        tool_pos_L
        tool_pos_R

        object_twist_actual_L   % 6xN actual object twist computed from left wJo * qdot
        object_twist_actual_R   % 6xN actual object twist computed from right wJo * qdot

        object_xdotbar_L        % 6xN desired object twist for left arm (first half of task xdotbar)
        object_xdotbar_R        % 6xN desired object twist for right arm (second half)
        
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
            
            % Allocate stuff
            obj.t = zeros(1, maxLoops);
            obj.ql = zeros(7, maxLoops);
            obj.qdotl = zeros(7, maxLoops);
            obj.qr = zeros(7, maxLoops);
            obj.qdotr = zeros(7, maxLoops);
            
            obj.tool_pos_L = zeros(3, maxLoops);
            obj.tool_pos_R = zeros(3, maxLoops);

            obj.object_twist_actual_L = nan(6, maxLoops);
            obj.object_twist_actual_R = nan(6, maxLoops);
            
            obj.object_xdotbar_L = nan(6, maxLoops);
            obj.object_xdotbar_R = nan(6, maxLoops);
           
            obj.xdotbar_task = cell(obj.n_tasks, maxLoops);
            obj.a_task = cell(obj.n_tasks, maxLoops);

            obj.action_idx_log = ones(1, maxLoops); 

        end
        

        function update(obj, t, loop, current_action_idx, wTt_L, wTt_R, ...
                        obj_vel_actual_left, obj_vel_actual_right, xdotbar_L, xdotbar_R)
        
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
                obj.a_task{i, loop} = diag(obj.tasks_ref{i}.A);
            end
        
            if numArgs < 4 || isempty(current_action_idx)
                current_action_idx = 1;
            end
            obj.action_idx_log(loop) = current_action_idx;
        
            % Store the object twists and references if provided
            % obj_vel_actual_left  is arg#7
            if numArgs >= 7 && ~isempty(obj_vel_actual_left) && numel(obj_vel_actual_left) >= 6
                obj.object_twist_actual_L(:, loop) = obj_vel_actual_left(:);
            else
                obj.object_twist_actual_L(:, loop) = nan(6,1);
            end
        
            % obj_vel_actual_right is arg#8
            if numArgs >= 8 && ~isempty(obj_vel_actual_right) && numel(obj_vel_actual_right) >= 6
                obj.object_twist_actual_R(:, loop) = obj_vel_actual_right(:);
            else
                obj.object_twist_actual_R(:, loop) = nan(6,1);
            end
        
            % xdotbar_L is arg#9 (expect 6x1)
            if numArgs >= 9 && ~isempty(xdotbar_L) && numel(xdotbar_L) >= 6
                obj.object_xdotbar_L(:, loop) = xdotbar_L(:);
            else
                obj.object_xdotbar_L(:, loop) = nan(6,1);
            end
        
            % xdotbar_R is arg#10 (expect 6x1)
            if numArgs >= 10 && ~isempty(xdotbar_R) && numel(xdotbar_R) >= 6
                obj.object_xdotbar_R(:, loop) = xdotbar_R(:);
            else
                obj.object_xdotbar_R(:, loop) = nan(6,1);
            end
        
            % Update data length
            obj.data_len = loop;
        end

      
        % PLOT TOOL DISTANCE
        function plotToolDistance(obj)
            valid_range = 1:obj.data_len;
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
            xlabel('');             % main x label is added by action strip, we dont add it here
            
            obj.expand_ylim(ax1);

            obj.format_plot(t_h, [], ax1, 'Distance [m]');
            
            obj.add_action_bars(ax1, valid_range, t_max);

        end


        function plotObjectActualReal(obj)
            % Plot norms of object reference velocities (L/R) and the actual object
            % velocities computed via each arm's object Jacobian

            valid_range = 1:obj.data_len;
            if isempty(valid_range)
                warning('No data to plot.');
                return;
            end
            time_vec = obj.t(valid_range);
            t_max = time_vec(end);
        
            % extract (6 x N) logged arrays for valid range
            refL = obj.object_xdotbar_L(:, valid_range);
            refR = obj.object_xdotbar_R(:, valid_range);
            actL = obj.object_twist_actual_L(:, valid_range);
            actR = obj.object_twist_actual_R(:, valid_range);
        
            norm_refL = vecnorm(refL, 2, 1);
            norm_refR = vecnorm(refR, 2, 1);
            norm_actL = vecnorm(actL, 2, 1);
            norm_actR = vecnorm(actR, 2, 1);
        
            fig_pos = [100, 100, 800, 400];

            figure(305); clf; set(gcf, 'Position', fig_pos, 'Color', 'w', 'Name', 'Object Velocity Norms');
        
            ax1 = subplot(1,1,1); hold(ax1,'on');

            col_refL = [0.9290 0.6940 0.1250];   
            col_refR = [0.4940 0.1840 0.5560];   
            col_actL = [0.4660 0.6740 0.1880];
            col_actR = [0.3010 0.7450 0.9330]; 
        
            % Plot: dashed refs, solid actuals
            p1 = plot(time_vec, norm_refL, '--', 'LineWidth', 1.8, ...
                      'Color', col_refL, 'DisplayName', '$\|{}^w\dot{\bar{\mathbf{x}}}_{o,L}\|$');
            
            p2 = plot(time_vec, norm_refR, '--', 'LineWidth', 1.8, ...
                      'Color', col_refR, 'DisplayName', '$\|{}^w\dot{\bar{\mathbf{x}}}_{o,R}\|$');
            
            p3 = plot(time_vec, norm_actL, '-',  'LineWidth', 1.8, ...
                      'Color', col_actL, 'DisplayName', '$\|{}^wJ_{o,L}\:\dot{\mathbf{q}}_L\|$');
            
            p4 = plot(time_vec, norm_actR, '-',  'LineWidth', 1.4, ...
                      'Color', col_actR, 'DisplayName', '$\|{}^wJ_{o,R}\:\dot{\mathbf{q}}_R\|$');
        
            grid on; xlim([0, t_max]);
            xlabel('');                 % main x label is added by action strip, we dont add it here
            t_h = title('\textbf{Desired \& Actual Object Cartesian Velocities Norm}', 'Interpreter', 'latex');
        
            l_h = legend([p1 p2 p3 p4], 'Location', 'best');
            obj.expand_ylim(ax1);
          
            set(gca, 'Box', 'on')

            obj.format_plot(t_h, l_h, ax1, '$\| {}^w \dot{\mathbf{x}}_o \|$ [m/s, rad/s]');
        
            % Add action strip at bottom
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
            
            % --- Left Arm State ---
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
            
            % --- Right Arm State ---
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
            
            % --- Individual Tasks ---
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
                    
                    % Plot reference on top and activation on bottom
                    ax1 = subplot(2,1,1);
                    set(gca, 'ColorOrder', c_order);
                    plot(t_plot, data_ref', 'LineWidth', 1.5);
                    grid on; xlim([0, t_max]);
                    
                    [leg_ref, y_lab_unit] = obj.generate_legends(dims, t_name_raw, 'Ref');
                    
                    % Set title depending on which task we are plotting
                    if contains(t_name_raw, 'Tool', 'IgnoreCase', true)
                        title_str = sprintf('\\textbf{%s - Reference ($\\dot{\\bar{\\mathbf{x}}} = {}^w\\dot{\\bar{\\mathbf{x}}}_t$)}', t_name);
                    
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
            is_joint = dims == 7 || dims == 14; 
            
            suffix = '';
            if contains(name, 'Left', 'IgnoreCase', true), suffix = ',L';
            elseif contains(name, 'Right', 'IgnoreCase', true), suffix = ',R'; end 
            
            legends = {};
            y_label_unit = '[unit]';
            
            if is_alt
                y_label_unit = '[m/s]';
                if is_ref, legends = {'$\dot{\bar{h}}$'}; else, legends = {'$\alpha$'}; end
                
            elseif dims == 6
                y_label_unit = '[rad/s, m/s]';
                if is_ref
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
                    if is_ref, legends = arrayfun(@(x) sprintf('$\\dot{q}_{%d%s}$',x,suffix), 1:7, 'UniformOutput', false);
                    else,      legends = arrayfun(@(x) sprintf('$\\alpha_{%d%s}$',x,suffix), 1:7, 'UniformOutput', false); end
                end
                
            elseif is_joint && dims == 14
                y_label_unit = '[rad/s]';
                if is_ref
                    legends = [arrayfun(@(x) sprintf('$\\dot{q}_{%d,L}$',x), 1:7, 'UniformOutput', false), ...
                               arrayfun(@(x) sprintf('$\\dot{q}_{%d,R}$',x), 1:7, 'UniformOutput', false)];
                else
                    legends = [arrayfun(@(x) sprintf('$\\alpha_{%d,L}$',x), 1:7, 'UniformOutput', false), ...
                               arrayfun(@(x) sprintf('$\\alpha_{%d,R}$',x), 1:7, 'UniformOutput', false)];
                end
                
            elseif dims == 12
                y_label_unit = '[rad/s, m/s]';
                if is_ref
                    legends = {'$\omega_{x,L}$','$\omega_{y,L}$','$\omega_{z,L}$','$\dot{x}_L$','$\dot{y}_L$','$\dot{z}_L$', ...
                               '$\omega_{x,R}$','$\omega_{y,R}$','$\omega_{z,R}$','$\dot{x}_R$','$\dot{y}_R$','$\dot{z}_R$'};
                else
                    legends = {'$\alpha_{\omega_{x},L}$','$\alpha_{\omega_{y},L}$','$\alpha_{\omega_{z},L}$','$\alpha_{\dot{x},L}$','$\alpha_{\dot{y},L}$','$\alpha_{\dot{z},L}$', ...
                               '$\alpha_{\omega_{x},R}$','$\alpha_{\omega_{y},R}$','$\alpha_{\omega_{z},R}$','$\alpha_{\dot{x},R}$','$\alpha_{\dot{y},R}$','$\alpha_{\dot{z},R}$'};
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