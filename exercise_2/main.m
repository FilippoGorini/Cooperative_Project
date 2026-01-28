function [logger] = bimanual_main()
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
    
    % Print on terminal parameters
    print_frequency = 4;
    print_interval = 1 / print_frequency;
    last_print_time = 0;

    % Action manager parameters
    action_transition_duration = 1.0;     % 1 second 
    n_dofs = 14;

    % Cartesian error thresholds for tool and obj
    ang_error_threshold = 0.01;
    lin_error_threshold = 0.001;
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

    % Initiliaze pandaArm() Class, specifying the base offset w.r.t World Frame
    left_arm = PandaArm(model, eye(4));
    % TO DO: TRANSFORMATION MATRIX FROM WORLD FRAME TO RIGHT ARM BASE FRAME
    wTb2 = [-1  0   0   1.06;
            0   -1  0   -0.01;
            0   0   1   0;
            0   0   0   1];
    right_arm = PandaArm(model, wTb2);
 
    % Initialize Bimanual Simulator Class
    bm_sim = bimanual_sim(dt, left_arm, right_arm, end_time);
    
    % Define Object Shape and origin Frame
    obj_length = 0.10;
    w_obj_pos = [0.5 0 0.59]';
    w_obj_ori = rotation(0, 0, 0);
    wTo_start = [w_obj_ori, w_obj_pos; 0 0 0 1];

    % Compute tool goal frame offset wrt to object frame
    offset = (obj_length/2) - 0.01;     % offset with a margin to not take the obj exactly at the end
    linear_offset = [offset 0 0]';
    % Set goal frames for left and right arm, based on object frame
    left_arm.setGoal(w_obj_pos, w_obj_ori, -linear_offset, rotation(pi, -pi/6, 0));    
    right_arm.setGoal(w_obj_pos, w_obj_ori, +linear_offset, rotation(pi, -pi/6, pi));   % With this pose we avoid making the right EE turn and collide with the left one

    % Define Object goal frame for bimanual motion of the object
    wTog = [rotation(0, 0, 0) [0.65, -0.35, 0.28]'; 0 0 0 1];
    left_arm.setObjGoal(wTog)
    right_arm.setObjGoal(wTog)

    % -------------
    % --- TASKS ---
    % -------------

    % Tool tasks
    left_tool_task = ToolTask("L", "Left Tool");
    right_tool_task = ToolTask("R", "Right Tool");

    % Joint limits task
    joint_limits_task = JointLimitsTask("BM", "Joint Limits");
 
    % Task minimum altitude
    left_min_alt_task = MinEffectorAltitudeTask("L", "Left Min. Alt.");
    left_min_alt_task.setObstacleHeight(table_height);
    right_min_alt_task = MinEffectorAltitudeTask("R", "Right Min. Alt.");
    right_min_alt_task.setObstacleHeight(table_height);

    % Task object kinematic constraint (unique for both arms)
    kin_constraint_task = KinConstraintTask("BM", "Kinematic Constraint");

    % Task object
    % left_object_task = ObjectTaskIndividual("L", "Object"); 
    % right_object_task = ObjectTaskIndividual("R", "Object");
    object_task = ObjectTaskDual("BM", "Object");

    % Stop joints task
    zero_joint_vel_task = ZeroJointVelTask("BM", "Zero Velocities");

    % ---------------
    % --- ACTIONS ---
    % ---------------

    % NB: Compared to the original implementation, the order in which we add
    % tasks to the action is not relevant for priorities, which are decided
    % in the unified list instead
    action_move_to = Action("MoveTo", {joint_limits_task, ...
                                            left_min_alt_task, right_min_alt_task, ...
                                            left_tool_task, right_tool_task});
    action_move_obj = Action("MoveObj", {kin_constraint_task, ...
                                            joint_limits_task, ...
                                            left_min_alt_task, right_min_alt_task, ...
                                            object_task});
    action_stop = Action("Stop", {left_min_alt_task, right_min_alt_task, ...
                                            zero_joint_vel_task});

    % Order defines priority { HIGHEST , ... , lowest}
    unified_list = {kin_constraint_task, ...
                    joint_limits_task, ...
                    left_min_alt_task, right_min_alt_task, ...
                    left_tool_task, right_tool_task, ...
                    object_task, ...
                    zero_joint_vel_task};

    % Load Action Manager Class and load actions
    actionManager = ActionManager(dt, n_dofs, action_transition_duration);
    actionManager.addUnifiedList(unified_list);
    actionManager.addAction(action_move_to);
    actionManager.addAction(action_move_obj);
    actionManager.addAction(action_stop);
    actionManager.setCurrentAction("MoveTo");   % Explicitly start with the MoveTo action

    % Initiliaze robot interface
    robot_udp = UDP_interface(real_robot);
    
    % Get action names list for simulation logger/plots
    action_names_list = cell(1, length(actionManager.actions));
    for k = 1:length(actionManager.actions)
        action_names_list{k} = actionManager.actions(k).name;
    end

    % Initialize logger
    logger = SimulationLogger(ceil(end_time/dt)+1, bm_sim, unified_list, action_names_list);

    % Flags 
    table_edge_passed = false;

    for t = 0:dt:end_time
        
        % Receive UDP packets - DO NOT EDIT
        [ql, qr] = robot_udp.udp_receive(t);
        if real_robot == true 
            bm_sim.left_arm.q = ql;
            bm_sim.right_arm.q = qr;
        end
        
        % Update Full kinematics of the bimanual system
        bm_sim.update_full_kinematics();

        % Handle different logic for each mission phase
        switch actionManager.getCurrentActionName()
            
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
                    actionManager.setCurrentAction("MoveObj");
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

                [right_obj_err_ori, right_obj_err_lin] = CartError(right_arm.wTog, right_arm.wTo);
                right_obj_ang_err = norm(right_obj_err_ori);
                right_obj_lin_err = norm(right_obj_err_lin);

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
                    
                    % Switch to stop action
                    actionManager.setCurrentAction("Stop");
                end

            case "Stop"
                
                % Feedback on terminal
                if (t - last_print_time > print_interval)
                    fprintf('ACTION: Stop [%.2f s]\n', t);
                    last_print_time = t;
                end

        end
        
        % Compute control commands for current action
        [q_dot] = actionManager.computeICAT(bm_sim);

        % Actual object velocities: if the constraint hold this of course should be equal
        obj_vel_actual_left  = bm_sim.left_arm.wJo  * q_dot(1:7);
        obj_vel_actual_right = bm_sim.right_arm.wJo * q_dot(8:14);
        
        % Step the simulator (integrate velocities)
        bm_sim.sim(q_dot);
        
        % Send updated state to Pybullet
        robot_udp.send(t, bm_sim)

        % Logging
        logger.update(bm_sim.time, bm_sim.loopCounter, actionManager.currentAction_idx, ...
                        bm_sim.left_arm.wTt(1:3, 4), bm_sim.right_arm.wTt(1:3, 4), ...      % we also log the tool positions
                        obj_vel_actual_left, obj_vel_actual_right, ...                      % and the actual object velocities  
                        object_task.xdotbar(1:6), object_task.xdotbar(7:12));               % and each arm's desired ref velocity              
        
        % Optional real-time slowdown
        SlowdownToRealtime(dt);
    end
    
end



%% LAUNCH THIS SECTION TO RUN THE SIMULATION ONCE
logger = bimanual_main();

%% PLOT ARM STATES AND TASKS

% Example on how to plot just a specific subset of tasks:
% logger.plotAll({"Left Tool", "Right Tool", "Left Min. Alt.", "Right Min. Alt."});

% Plot all instead:
logger.plotAll();

%% PLOT LINEAR DISTANCE BETWEEN TOOLS
logger.plotToolDistance();

%% PLOT DESIRED VS ACTUAL OBJECT VELOCITIES
logger.plotObjectActualReal();
