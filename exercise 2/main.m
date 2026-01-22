function main()
    %Add path
    addpath('./simulation_scripts');
    addpath('./tools')
    addpath('./icat')
    addpath('./tasks')
    close all; 
    clear; 
    clc; 

    %Simulation Parameters
    dt = 0.005;
    end_time = 40;
    is_reaching_phase = true;
    is_moving_phase = false;
    is_stopped = false;
    done = 0;

    action_transition_duration = 1;     % 1 second 
    n_dofs = 14;

    % Thresholds
    ang_error_threshold = 0.02;
    lin_error_threshold = 0.001;
    obj_ang_error_threshold = 0.02;
    obj_lin_error_threshold = 0.001;

    % Initialize Franka Emika Panda Model
    model = load("panda.mat");

    %Simulation Setup
    real_robot = false;

    %Initiliaze panda_arm() Class, specifying the base offset w.r.t World Frame
    arm1 = panda_arm(model, eye(4));
    %TO DO: TRANSFORMATION MATRIX FROM WORLD FRAME TO RIGHT ARM BASE FRAME
    wTb2 = [-1  0   0   1.06;
            0   -1  0   -0.01;
            0   0   1   0;
            0   0   0   1];
    arm2 = panda_arm(model, wTb2);
 
    %Initialize Bimanual Simulator Class
    bm_sim = bimanual_sim(dt, arm1, arm2, end_time);
    
    %Define Object Shape and origin Frame

    obj_length = 0.10;

    w_obj_pos = [0.5 0 0.59]';

    w_obj_ori = rotation(0, 0, 0);

    %Set goal frames for left and right arm, based on object frame
    
    %TO DO: Set arm goal frame based on object frame.
    offset = (obj_length/2) - 0.01; % offset with a margin to not take the obj exactly at the end
    linear_offset = [offset 0 0]';
    
    arm1.setGoal(w_obj_pos, w_obj_ori, -linear_offset, rotation(pi, -pi/6, 0));
    
    arm2.setGoal(w_obj_pos, w_obj_ori, +linear_offset, rotation(pi, pi/6, 0));



    % Define Object goal frame (Cooperative Motion)
    wTog = [rotation(0, 0, 0) [0.65, -0.35, 0.28]'; 0 0 0 1];
    arm1.set_obj_goal(wTog)
    arm2.set_obj_goal(wTog)

    %Define Tasks, input values(Robot type(L,R,BM), Task Name)
    left_tool_task = ToolTask("L", "LT");
    right_tool_task = ToolTask("R", "RT");

    % % Task joint limits
    % left_joint_task = JointLimitsTask("L", "LJ");
    % right_joint_task = JointLimitsTask("R", "RJ");
    joint_limits_task = JointLimitsTask("LR", "JL");
 
    % Task minimum altitude
    left_min_ee_alt_task = MinEffectorAltitudeTask("L", "LMA");
    right_min_ee_alt_task = MinEffectorAltitudeTask("R", "RMA");

    % Task object kinematic constraint (unique for both arms)
    kin_constraint_task = KinConstraintTask();

    % Task object
    object_task = ObjectTaskMasterSlave("L", "LO"); 
    % object_task = ObjectTaskSymmetric("L", "LO");

    %Actions for each phase: go to phase, coop_motion phase, end_motion phase
    action_go_to = Action("ReachObject", {left_tool_task, right_tool_task, ...
                                          joint_limits_task, ...
                                          left_min_ee_alt_task, right_min_ee_alt_task});
    action_move_obj = Action("MoveObject", {kin_constraint_task, ...
                                            joint_limits_task, ...
                                            left_min_ee_alt_task, right_min_ee_alt_task, ...
                                            object_task});
    action_stop = Action("Stop", {joint_limits_task, ...
                                  left_min_ee_alt_task, right_min_ee_alt_task});

    % order define priority { HIGHEST , ... , lowest}
    % global_list = {left_tool_task, right_tool_task};
    global_list = {kin_constraint_task, ...
                   joint_limits_task, ...
                   left_min_ee_alt_task, right_min_ee_alt_task, ...
                   object_task, ...
                   left_tool_task, right_tool_task};

    %Load Action Manager Class and load actions
    actionManager = ActionManager(dt, n_dofs, action_transition_duration);
    actionManager.addTaskSet(global_list);
    actionManager.addAction(action_go_to);
    actionManager.addAction(action_move_obj);
    actionManager.addAction(action_stop);

    %Initiliaze robot interface
    robot_udp = UDP_interface(real_robot);

    %Initialize logger
    logger = SimulationLogger(ceil(end_time/dt)+1, bm_sim, global_list);

    %Main simulation Loop
    for t = 0:dt:end_time

        % 1. Receive UDP packets - DO NOT EDIT
        [ql, qr] = robot_udp.udp_receive(t);
        if real_robot == true %Only in real setup, assign current robot configuration as initial configuratio
            bm_sim.left_arm.q = ql;
            bm_sim.right_arm.q = qr;
        end

        % 2. Update Full kinematics of the bimanual system
        bm_sim.update_full_kinematics();


        if is_reaching_phase    
          
            left_ang_error = norm(arm1.rot_to_goal);
            left_lin_error = norm(arm1.dist_to_goal);
            right_ang_error = norm(arm2.rot_to_goal);
            right_lin_error = norm(arm2.dist_to_goal);
            
            % Check Thresholds
            if (left_ang_error < ang_error_threshold) && (left_lin_error < lin_error_threshold) && ...
               (right_ang_error < ang_error_threshold) && (right_lin_error < lin_error_threshold)
               
                disp(['Target Reached at t = ' num2str(t) '. Switching to MoveObject.']);
                
                % --- TRANSITION LOGIC (Run Once) ---
                
                % 1. Define Actual Object Frame (from simulation setup constants)
                wTo_obj = [w_obj_ori, w_obj_pos; 0 0 0 1];
                
                % 2. Compute Relative Grasp Transforms (tTo)
                % Using your invT function
                tTo_L = invT(arm1.wTt) * wTo_obj;
                tTo_R = invT(arm2.wTt) * wTo_obj;
                
                % 3. Set the grasp transforms in the robot objects
                arm1.set_grasp_transform(tTo_L);
                arm2.set_grasp_transform(tTo_R);
            
                arm1.update_transform();
                arm2.update_transform();

                arm1.update_jacobian();
                arm2.update_jacobian();
                
                % 4. Switch Action
                actionManager.setCurrentAction("MoveObject");
                
                % 5. Update flag to stop checking this condition
                is_reaching_phase = false; 
                is_moving_phase = true;

                % 6. We now "forget" the table and set the ground (z=0) as
                % the obstacle
                % left_min_ee_alt_task.setObstacleHeight(0);
                % right_min_ee_alt_task.setObstacleHeight(0);
            end
        end

        if is_moving_phase

            % Actual x position of the arm
            current_obj_x = arm1.wTo(1,4); 
            
            % x threshold, after this limit manipulators can lower towards the target pos
            table_edge_threshold = 0.58; 
        
            % If we go over the edge of the table we can start to go down
            % To achieve this we need to put 0 the obstacle height in minimum altitude task
            if current_obj_x > table_edge_threshold && not(done)
                left_min_ee_alt_task.setObstacleHeight(0);
                right_min_ee_alt_task.setObstacleHeight(0);
                done = 1;
                disp(['t = ' num2str(t)]);
                %disp("Table edge reached: obstacle height is now set to 0.")
            end


            % NB: for now we just check only the left arm's object frame
            % Compute vectors first
            [obj_err_ori, obj_err_lin] = CartError(arm1.wTog , arm1.wTo);

            % Compute norms separately
            obj_ang_error = norm(obj_err_ori);
            obj_lin_error = norm(obj_err_lin);

            % Check Thresholds
            if (obj_ang_error < obj_ang_error_threshold) && (obj_lin_error < obj_lin_error_threshold)

                disp(['Object moved to position at t = ' num2str(t) '. Switching to Stop.']);

                % Switch to stop action
                actionManager.setCurrentAction("Stop");

                is_moving_phase = false;
                is_stopped = true;
            end
        end
    
        if is_stopped 
            disp("MISSION COMPLETED");
        end
        

        % 3. Compute control commands for current action
        [q_dot] = actionManager.computeICAT(bm_sim);

        % 4. Step the simulator (integrate velocities)
        bm_sim.sim(q_dot);

        % 5. Send updated state to Pybullet
        robot_udp.send(t, bm_sim)

        % 6. Logging
        logger.update(bm_sim.time, bm_sim.loopCounter)
        bm_sim.time;

        % 7. Optional real-time slowdown
        SlowdownToRealtime(dt);
    end

    %Display joint position and velocity, Display for a given action, a number
    %of tasks
    action = 1;
    tasks = [1];
    %logger.plotAll(action, tasks);
    logger.plotAll();
end
