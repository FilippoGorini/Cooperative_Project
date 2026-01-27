function main()
    % Add path
    addpath('./simulation_scripts');
    addpath('./tools')
    addpath('./icat')
    addpath('./tasks')
    close all; 
    clear; 
    clc;  


    % Simulation Setup
    real_robot = false;

    % Simulation Parameters
    dt = 0.001;     % lowering the dt seems to decrease the drifting error between the 2 grippers in the bimanual manipulation phase
    end_time = 15;

    % Stuff for singularity debug
    % We store the minimum non zero sigma saw by the RegPseudoInverse (if it is lower than 0.01 regularization is applied)
    n_steps = ceil(end_time/dt) + 1;
    internal_sigma_left = zeros(1, n_steps);     
    internal_sigma_right = zeros(1, n_steps);
    loop_idx = 0;
    t_coop_start = 0;
    
    % Print on terminal parameters
    print_frequency = 4;
    print_interval = 1 / print_frequency;
    last_print_time = 0;

    % Action manager parameters
    action_transition_duration = 1.0;       % 1 second 
    n_dofs = 7;                             % 7 dof for each arm

    % Cartesian error thresholds for tool and obj
    ang_error_threshold = 0.01;
    lin_error_threshold = 0.003;
    obj_ang_error_threshold = 0.01;
    obj_lin_error_threshold = 0.01;
    % Table edge threshold: we assumed that the object lies on a table,
    % which the robot has to avoid while moving the object to the object
    % goal position. After the robot clears the table edge, the minimum
    % altitude is computed wrt the floor instead of the table
    table_edge_threshold = 0.25;   
    table_height = 0.55;
    
    % Initialize Franka Emika Panda Model
    model = load("panda.mat");

    % Initiliaze panda_arm() Class, specifying the base offset w.r.t World Frame
    left_arm = PandaArm(model,eye(4));
    % TO DO: TRANSFORMATION MATRIX FROM WORLD FRAME TO RIGHT ARM BASE FRAME
    wTb2 = [-1  0   0   1.06;
            0   -1  0   -0.01;
            0   0   1   0;
            0   0   0   1];
    right_arm = PandaArm(model, wTb2);
    
    % Initialize Cooperative Simulator Class
    coop_system = coop_sim(dt, left_arm, right_arm, end_time);
    
    % Define Object Shape and origin Frame
    obj_length = 0.06;
    w_obj_pos = [0.5 0 0.59]';
    w_obj_ori = rotation(0, 0, 0);
    wTo_start = [w_obj_ori, w_obj_pos; 0 0 0 1];
    
    % Compute tool goal frame offset wrt to object frame
    offset = (obj_length/2) - 0.01;     % offset with a margin to not take the obj exactly at the end
    linear_offset = [offset 0 0]';
    % Set goal frames for left and right arm, based on object frame
    % The goal orientation of the tool frames is obtained by rotating the tool frames 20 deg (pi/9) around their y-axis
    left_arm.setGoal(w_obj_pos, w_obj_ori, -linear_offset, rotation(pi, -pi/9, 0));
    right_arm.setGoal(w_obj_pos, w_obj_ori, +linear_offset, rotation(pi, -pi/9, pi));
    
    % Define Object goal frame (Cooperative Motion)
    wTog = [rotation(0, pi/6, 0) [0.6, 0.4, 0.48]'; 0 0 0 1];
    left_arm.setObjGoal(wTog);
    right_arm.setObjGoal(wTog);
    
    % -------------
    % --- TASKS ---
    % -------------

    % TOOL TASKS
    left_tool_task = ToolTask("L", "Left Tool");
    right_tool_task = ToolTask("R", "Right Tool");
    
    % JOINT LIMITS TASKS
    left_joint_limits_task = JointLimitsIndividualTask("L", "Left Joint Limits");
    right_joint_limits_task = JointLimitsIndividualTask("R", "Right Joint Limits");

    % MINIMUM END EFFECTOR ALTITUDE TASKS
    left_min_alt_task = MinEffectorAltitudeTask("L", "Left Min. Alt.");
    left_min_alt_task.setObstacleHeight(table_height);
    right_min_alt_task = MinEffectorAltitudeTask("R", "Right Min. Alt.");
    right_min_alt_task.setObstacleHeight(table_height);

    % OBJECT TASKS (non cooperative)
    left_object_task = ObjectTask("L", "Left Object"); 
    right_object_task = ObjectTask("R", "Right Object");

    % ZERO VELOCITY TASKS
    left_zero_joint_vel_task = ZeroJointVelTask("L", "Left Zero Velocities");
    right_zero_joint_vel_task = ZeroJointVelTask("R", "Right Zero Velocities");

    % COOPERATIVE OBJECT TASK (CONSTRAINT)
    left_object_task_coop = CoopObjectTask("L", "Left Cooperative Object");
    right_object_task_coop = CoopObjectTask("R", "Right Cooperative Object");


    % ---------------
    % --- ACTIONS ---
    % ---------------

    % NB: Compared to the original implementation, the order in which we add
    % tasks to the action is not relevant for priorities, which are decided
    % in the unified list instead
    left_move_to_action = Action("LeftMoveTo", {left_joint_limits_task, ...
                                                        left_min_alt_task, ...
                                                        left_tool_task});

    right_move_to_action = Action("RightMoveTo", {right_joint_limits_task, ...
                                                        right_min_alt_task, ...
                                                        right_tool_task});

    left_move_obj_action = Action("LeftMoveObj", {left_joint_limits_task, ...
                                                        left_min_alt_task, ...
                                                        left_object_task});

    right_move_obj_action = Action("RightMoveObj", {right_joint_limits_task, ...
                                                        right_min_alt_task, ...
                                                        right_object_task});

    left_zero_vel_action = Action("LeftZeroVel", {left_min_alt_task, ...
                                                        left_zero_joint_vel_task});

    right_zero_vel_action = Action("RightZeroVel", {right_min_alt_task, ...
                                                        right_zero_joint_vel_task});
    
    % COOPERATIVE ACTIONS
    left_move_obj_action_coop = Action("LeftMoveObjCoop", {left_object_task_coop, ...
                                                            left_joint_limits_task, ...
                                                            left_min_alt_task});

    right_move_obj_action_coop = Action("RightMoveObjCoop", {right_object_task_coop, ...
                                                            right_joint_limits_task, ...
                                                            right_min_alt_task});

    % NON-COOPERATIVE UNIFIED LISTS
    left_unified_list = {left_joint_limits_task, ...
                            left_min_alt_task, ...
                            left_tool_task, ...
                            left_object_task, ...
                            left_zero_joint_vel_task};

    right_unified_list = {right_joint_limits_task, ...
                            right_min_alt_task, ...
                            right_tool_task, ...
                            right_object_task, ...
                            right_zero_joint_vel_task};

    % COOPERATIVE UNIFIED LISTS (we now put the cooperative constraint task at top priority
    left_unified_list_coop = {left_object_task_coop, ...
                                left_joint_limits_task, ...
                                left_min_alt_task};

    right_unified_list_coop = {right_object_task_coop, ...
                                right_joint_limits_task, ...
                                right_min_alt_task};

    % TO DO: Create two action manager objects to manage the tasks of a single
    % manipulator (one for the non-cooperative and one for the cooperative steps
    % of the algorithm)

    % NON-COOPERATIVE ACTION MANAGERS
    left_action_manager = ActionManager(dt, n_dofs, action_transition_duration);
    left_action_manager.addUnifiedList(left_unified_list);
    left_action_manager.addAction(left_move_to_action);
    left_action_manager.addAction(left_move_obj_action);
    left_action_manager.addAction(left_zero_vel_action);
    left_action_manager.setCurrentAction("LeftMoveTo");   % Explicitly start with the MoveTo action

    right_action_manager = ActionManager(dt, n_dofs, action_transition_duration);
    right_action_manager.addUnifiedList(right_unified_list);
    right_action_manager.addAction(right_move_to_action);
    right_action_manager.addAction(right_move_obj_action);
    right_action_manager.addAction(right_zero_vel_action);
    right_action_manager.setCurrentAction("RightMoveTo");   % Explicitly start with the MoveTo action
    
    % COOPERATIVE ACTION MANAGERS
    left_action_manager_coop = ActionManager(dt, n_dofs, action_transition_duration);
    left_action_manager_coop.addUnifiedList(left_unified_list_coop);
    left_action_manager_coop.addAction(left_move_obj_action_coop);
    left_action_manager_coop.setCurrentAction("LeftMoveObjCoop");  
    % We can set the move obj action already as that's the only action for which we need the cooperative TPIK 

    right_action_manager_coop = ActionManager(dt, n_dofs, action_transition_duration);
    right_action_manager_coop.addUnifiedList(right_unified_list_coop);
    right_action_manager_coop.addAction(right_move_obj_action_coop);
    right_action_manager_coop.setCurrentAction("RightMoveObjCoop");  
    
    % Initiliaze robot interface
    robot_udp = UDP_interface(real_robot);
    
    % Initialize logger
    logger_left = SimulationLogger(ceil(end_time/dt)+1, coop_system, left_unified_list);
    logger_right = SimulationLogger(ceil(end_time/dt)+1, coop_system, right_unified_list);
    
    % Flags 
    table_edge_passed = false;

    % Set initial state
    mission_phase = "MoveTo";

    % Main simulation Loop
    for t = 0:dt:end_time

        % Receive UDP packets - DO NOT EDIT
        [ql, qr] = robot_udp.udp_receive(t);
        if real_robot == true       % Only in real setup, assign current robot configuration as initial configuratio
            coop_system.left_arm.q = ql;
            coop_system.right_arm.q = qr;
        end

        % Update Full kinematics of the bimanual system
        coop_system.update_full_kinematics();
        
        % Needed for singularity debug logic
        loop_idx = loop_idx + 1;        

        switch mission_phase

            case "MoveTo"
                % Compute tool errors
                left_ang_error = norm(left_arm.rot_to_goal);
                left_lin_error = norm(left_arm.dist_to_goal);
                right_ang_error = norm(right_arm.rot_to_goal);
                right_lin_error = norm(right_arm.dist_to_goal);

                % Feedback on terminal
                if (t - last_print_time > print_interval)
                    fprintf('ACTION: MoveTo [%.2f s]\n', t);
                    fprintf('  Left Arm Tool: Lin Err = %.4f m | Ang Err = %.4f rad\n', left_lin_error, left_ang_error);
                    fprintf('  Right Arm Tool: Lin Err = %.4f m | Ang Err = %.4f rad\n', right_lin_error, right_ang_error);
                    fprintf('\n');
                    last_print_time = t;
                end

                % Check thresholds: if we got to the goal switch to next action
                if (left_ang_error < ang_error_threshold) && (left_lin_error < lin_error_threshold) && ...
                    (right_ang_error < ang_error_threshold) && (right_lin_error < lin_error_threshold)
               
                    fprintf('TOOL GOALS REACHED [%.2f s]\n', t);
                    fprintf('  Switch to action: MoveObj\n');
                    fprintf('\n');
                    
                    % Compute current obj position relative to current arm pose
                    tTo_L = invT(left_arm.wTt) * wTo_start;
                    tTo_R = invT(right_arm.wTt) * wTo_start;
                    
                    % Set the obj/tool transforms 
                    left_arm.setObjToolTransform(tTo_L);
                    right_arm.setObjToolTransform(tTo_R);
                
                    % Force update of Jacobians/Transforms
                    left_arm.updateTransform();
                    right_arm.updateTransform();
                    left_arm.updateJacobian();
                    right_arm.updateJacobian();
                    
                    % Switch to next action
                    left_action_manager.setCurrentAction("LeftMoveObj");
                    right_action_manager.setCurrentAction("RightMoveObj");
                    
                    % Trigger state transition
                    mission_phase = "MoveObj";

                    % Just for help with singularity debug
                    t_coop_start = t;
                end

            case "MoveObj"
                % Current y position of the left arm, we check just one for
                % simplicity as they're kinematically constrained to each other
                % We take the absolute value assuming table is centered
                current_obj_y_abs = abs(left_arm.wTo(2,4));
            
                if current_obj_y_abs > table_edge_threshold && ~table_edge_passed
                    % If we cleared the table edge we can now set the
                    % ground as the new obstacle to keep a minimum altitude from
                    left_min_alt_task.setObstacleHeight(0);
                    right_min_alt_task.setObstacleHeight(0);
                    
                    table_edge_passed = true;
                    
                    fprintf('TABLE EDGE CLEARED [%.2f s]\n', t);
                    fprintf('  Obstacle height set to 0 (Ground).\n');
                    fprintf('\n');
                end
                
                % Compute object error for both arms
                [left_obj_err_ori, left_obj_err_lin] = CartError(left_arm.wTog, left_arm.wTo);
                left_obj_ang_err = norm(left_obj_err_ori);
                left_obj_lin_err = norm(left_obj_err_lin);

                [right_obj_err_ori, right_obj_err_lin_R] = CartError(right_arm.wTog, right_arm.wTo);
                right_obj_ang_err = norm(right_obj_err_ori);
                right_obj_lin_err = norm(right_obj_err_lin_R);

                % Feedback on terminal
                if (t - last_print_time > print_interval)
                    fprintf('ACTION: MoveObj [%.2f s]\n', t);
                    fprintf('  Obj (Left Est) : Lin Err = %.4f m | Ang Err = %.4f rad\n', left_obj_lin_err, left_obj_ang_err);
                    fprintf('  Obj (Right Est): Lin Err = %.4f m | Ang Err = %.4f rad\n', right_obj_lin_err, right_obj_ang_err);
                    fprintf('\n'); 
                    last_print_time = t;
                end

                % Check thresholds (we check both for safety even if, if the constraint works perfectly, one should probably be enough)
                if (left_obj_ang_err < obj_ang_error_threshold) && (left_obj_lin_err < obj_lin_error_threshold) && ...
                   (right_obj_ang_err < obj_ang_error_threshold) && (right_obj_lin_err < obj_lin_error_threshold)
                   
                    fprintf('OBJECT GOAL REACHED [%.2f s]\n', t);
                    fprintf('  Switch to action: Stop\n');
                    fprintf('\n');
                    
                    % Switch to next action
                    left_action_manager.setCurrentAction("LeftZeroVel");
                    right_action_manager.setCurrentAction("RightZeroVel");

                    % Trigger state transition
                    mission_phase = "Stop";
                end

            case "Stop"
                % Feedback on terminal
                if (t - last_print_time > print_interval)
                    fprintf('ACTION: Stop [%.2f s]\n', t);
                    last_print_time = t;
                end

        end

        % Non cooperative TPIK: 
        qdot_l = left_action_manager.computeICAT(coop_system);
        qdot_r = right_action_manager.computeICAT(coop_system);

        % Cooperative TPIK (only for cooperative object motion phase)
        if mission_phase == "MoveObj"

            % Compute non-cooperative OBJECT velocities
            xdot_l = left_arm.wJo * qdot_l;
            xdot_r = right_arm.wJo * qdot_r;

            % Compute admissible object frame velocity space of each arm
            % when standalone acting
            H_l = left_arm.wJo * pinv(left_arm.wJo);
            H_r = right_arm.wJo * pinv(right_arm.wJo);
            
            % If the robots were actually independent, now is the step
            % where they would exchange xdot_l, xdot_r, H_l and H_r, but we
            % can skip it as we already have them in the same program

            % Compute weighting coefficients
            mu_0 = 0.005;       % Small number > 0
            mu_l = mu_0 + norm(left_object_task.xdotbar - xdot_l);
            mu_r = mu_0 + norm(right_object_task.xdotbar - xdot_r);
            
            % NB: in a real system where the robots are independent, xdot_hat
            % is computed (in the exact same way) by both robots, here we
            % compute it just once for simplicity.
            xdot_hat = 1 / (mu_l + mu_r) * (mu_l * xdot_l + mu_r * xdot_r);
            
            % Compute cartesian constraint
            C = [H_l, -H_r];
            % Compute combined unconstrained motion space
            H_lr = [H_l, zeros(6, 6); zeros(6, 6), H_r];

            % Update CoopObjectTask
            left_object_task_coop.xdot_hat = xdot_hat;
            left_object_task_coop.C = C;
            left_object_task_coop.H_lr = H_lr;

            right_object_task_coop.xdot_hat = xdot_hat;
            right_object_task_coop.C = C;
            right_object_task_coop.H_lr = H_lr;
            
            % Compute cooperative TPIK with new cooperative velocity constraint
            qdot_l = left_action_manager_coop.computeICAT(coop_system);
            qdot_r = right_action_manager_coop.computeICAT(coop_system);

            % DEBUG: we log the minimum non zero singular value seen by the
            % RegPseudoInverse to know if it ever gets under the threshold
            % (0.01) which activates regularization
            internal_sigma_left(loop_idx) = left_action_manager_coop.debug_min_sigma;  
            internal_sigma_right(loop_idx) = right_action_manager_coop.debug_min_sigma;
            
        end

        coop_system.sim(qdot_l, qdot_r);
        
        % Send updated state to Pybullet
        robot_udp.send(t, coop_system)
    
        % Logging
        logger_left.update(coop_system.time, coop_system.loopCounter)
        logger_right.update(coop_system.time, coop_system.loopCounter)
        coop_system.time;

        % Optional real-time slowdown
        SlowdownToRealtime(dt);
    end

    %9. Display joint position, velocity and end effector velocities, Display for a given action, a number
    %of tasks
    % action=1;
    % tasks=[1];
    logger_left.plotAll();
    logger_right.plotAll();


    % DEBUG: Singularity plot
    figure('Name', 'Internal Regularization Check', 'Color', 'w');
    
    time_vector = 0:dt:end_time;
    len = min([length(time_vector), length(internal_sigma_left)]);
    
    % Find the index corresponding to the start of the cooperative phase
    start_idx = find(time_vector >= t_coop_start, 1, 'first');
    if isempty(start_idx), start_idx = 1; end % Safety check
    
    % Slice the vectors
    t_plot = time_vector(start_idx:len);
    sigma_l_plot = internal_sigma_left(start_idx:len);
    sigma_r_plot = internal_sigma_right(start_idx:len);
    
    % Plot the sliced data
    plot(t_plot, sigma_l_plot, 'b', 'LineWidth', 1.5); hold on;
    plot(t_plot, sigma_r_plot, 'r', 'LineWidth', 1.5);
    
    % Draw the Threshold Line
    yline(0.01, '--k', '$\lambda_{thresh} = 0.01$', ...
        'LineWidth', 2, 'LabelHorizontalAlignment', 'left', ...
        'Interpreter', 'latex', 'FontSize', 12);
    
    % Formatting with LaTeX
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12);
    ylabel('Squared Singular Value $\sigma^2_i(J_{proj})$', 'Interpreter', 'latex', 'FontSize', 12);
    
    legend({'Left Arm ($\sigma^2_{min}$)', 'Right Arm ($\sigma^2_{min}$)'}, ...
           'Interpreter', 'latex', 'FontSize', 12, 'Location', 'northeast');
           
    title('\textbf{Singularity Check}', 'Interpreter', 'latex', 'FontSize', 14);
    
    grid on;
    xlim([t_coop_start, end_time]); % Force x-axis to start at transition
    ylim([0, 0.05]);


end
