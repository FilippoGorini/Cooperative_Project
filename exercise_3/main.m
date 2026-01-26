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
    
    % Print on terminal parameters
    print_frequency = 4;
    print_interval = 1 / print_frequency;
    last_print_time = 0;

    % Action manager parameters
    action_transition_duration = 1.0;       % 1 second 
    n_dofs = 7;                             % 7 dof for each arm

    % Cartesian error thresholds for tool and obj
    ang_error_threshold = 0.01;
    lin_error_threshold = 0.001;
    obj_ang_error_threshold = 0.01;
    obj_lin_error_threshold = 0.01;
    % Table edge threshold: we assumed that the object lies on a table,
    % which the robot has to avoid while moving the object to the object
    % goal position. After the robot clears the table edge, the minimum
    % altitude is computed wrt the floor instead of the table
    table_edge_threshold = 0.58; 
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
    wTog = [rotation(0,0,0) [0.6, 0.4, 0.48]'; 0 0 0 1];
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

    % % KINEMATIC CONSTRAINT?????????? BOH
    % left_kinematic_constraint_task
    % right_kinematic_constraint_task


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

    % COOPERATIVE UNIFIED LISTS
    % left_unified_list = {left_joint_limits_task, ...
    %                        left_min_alt_task, ...
    %                        left_tool_task, ...
    %                        left_object_task, ...
    %                        left_zero_joint_vel_task};
    % right_unified_list = {right_joint_limits_task, ...
    %                        right_min_alt_task, ...
    %                        right_tool_task, ...
    %                        right_object_task, ...
    %                        right_zero_joint_vel_task};

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
    % left_action_manager_COOP = ActionManager(dt, n_dofs, action_transition_duration);
    % left_action_manager.addUnifiedList(left_unified_list_COOP);
    % left_action_manager.addAction(left_move_to_action);
    % left_action_manager.addAction(left_move_obj_action);
    % left_action_manager.addAction(left_zero_vel_action);
    % left_action_manager.setCurrentAction("LeftMoveTo");   % Explicitly start with the MoveTo action
    % 
    % right_action_manager_COOP = ActionManager(dt, n_dofs, action_transition_duration);
    % right_action_manager.addUnifiedList(right_unified_list_COOP);
    % right_action_manager.addAction(right_move_to_action);
    % right_action_manager.addAction(right_move_obj_action);
    % right_action_manager.addAction(right_zero_vel_action);
    % right_action_manager.setCurrentAction("RightMoveTo");   % Explicitly start with the MoveTo action
    
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

        % TO DO: compute the TPIK for each manipulator with your action
        % manager
        [ql_dot] = left_action_manager.computeICAT(coop_system);
        [qr_dot] = right_action_manager.computeICAT(coop_system);

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
                end

            case "MoveObj"
                % Current x position of the left arm, we check just one for
                % simplicity as they're kinematically constrained to each other
                current_obj_x = left_arm.wTo(1,4); 
            
                if current_obj_x > table_edge_threshold && ~table_edge_passed
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

        % 4. TO DO: COOPERATION hierarchy
        % SAVE THE NON COOPERATIVE VELOCITIES COMPUTED
    
        % 5. TO DO: compute the TPIK for each manipulator with your action
        % manager (with the constrained action to track the coop velocity)
    
        % 6. get the two variables for integration
        coop_system.sim(ql_dot, qr_dot);
        
        % 6. Send updated state to Pybullet
        robot_udp.send(t, coop_system)
    
        % 7. Loggging
        logger_left.update(coop_system.time, coop_system.loopCounter)
        logger_right.update(coop_system.time, coop_system.loopCounter)
        coop_system.time;
        % 8. Optional real-time slowdown
        SlowdownToRealtime(dt);
    end

    %9. Display joint position, velocity and end effector velocities, Display for a given action, a number
    %of tasks
    % action=1;
    % tasks=[1];
    logger_left.plotAll();
    logger_right.plotAll();

end
