function main()
    %Add path
    addpath('./simulation_scripts');
    addpath('./tools')
    addpath('./icat')
    addpath('./tasks')
    clc;clear;close all; 


    % Simulation Parameters
    dt = 0.001;
    end_time = 20;

    % Thresholds
    ang_error_threshold = 0.02;
    lin_error_threshold = 0.001;
    obj_ang_error_threshold = 0.02;
    obj_lin_error_threshold = 0.001;
    
    % Initialize Franka Emika Panda Model
    model = load("panda.mat");
    
    % Simulation Setup
    real_robot = false;
    
    % Initiliaze panda_arm() Class, specifying the base offset w.r.t World Frame
    left_arm = PandaArm(model,eye(4));
    % Transform from world frame to arm2 base frame
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
    
    % Set goal frames for left and right arm, based on object frame
    %TO DO: Set arm goal frame based on object frame.
    offset = (obj_length/2) - 0.01; % offset with a margin to not take the obj exactly at the end
    linear_offset = [offset 0 0]';

    % The goal orientation of the tool frames is obtained by rotating the tool frames 20 deg (pi/9) around their y-axis
    left_arm.setGoal(w_obj_pos, w_obj_ori, -linear_offset, rotation(pi, -pi/9, 0));
    right_arm.setGoal(w_obj_pos, w_obj_ori, +linear_offset, rotation(pi, -pi/9, pi));
    
    % Define Object goal frame (Cooperative Motion)
    % wTog=[rotation(0,0,0) [0.6, -0.4, 0.48]'; 0 0 0 1];   
    wTog=[rotation(0,0,0) [0.6, 0.4, 0.48]'; 0 0 0 1];
    left_arm.setObjGoal(wTog);
    right_arm.setObjGoal(wTog);
    
    % Define Tasks, input values(Robot type(L,R,BM), Task Name)

    % TOOL TASKS
    left_tool_task=ToolTask("L","LT");
    right_tool_task=ToolTask("R","RT");
    
    % JOINT LIMITS TASKS
    left_joint_task = JointLimitsIndividualTask("L", "LJ");
    right_joint_task = JointLimitsIndividualTask("R", "RJ");

    % MINIMUM END EFFECTOR ALTITUDE TASKS
    left_min_alt_task = MinEffectorAltitudeTask("L", "LMA");
    right_min_alt_task = MinEffectorAltitudeTask("R", "RMA");

    % KINEMATIC CONSTRAINT?????????? BOH
    % OBJECT TASK??????????

    % TO DO: Define the actions for each manipulator (remember the specific one
    % for the cooperation)
    left_move_to_action = Action("LeftMoveTo", {left_tool_task, ...
                                          left_joint_task, ...
                                          left_min_alt_task});
    right_move_to_action = Action("RightMoveTo", {right_tool_task, ...
                                          right_joint_task, ...
                                          right_min_alt_task});
    % TODO: add actions and tasks for phase moving
    % left_rigid_move_action = Action("LeftMoveObj", {left_tool_task, ...
    %                                       left_joint_task, ...
    %                                       left_min_alt_task});
    

    % Priorities are defined by the order in this lists
    left_global_list = {left_joint_task, ...
                   left_min_alt_task, ...
                   left_tool_task};
    right_global_list = {right_joint_task, ...
                   right_min_alt_task, ...
                   right_tool_task};


    action_transition_duration = 2.0;
    % TO DO: Create two action manager objects to manage the tasks of a single
    % manipulator (one for the non-cooperative and one for the cooperative steps
    % of the algorithm)

    left_action_manager = ActionManager(dt, 7, action_transition_duration);
    left_action_manager.addUnifiedList(left_global_list);
    left_action_manager.addAction(left_move_to_action);

    right_action_manager = ActionManager(dt, 7, action_transition_duration);
    right_action_manager.addUnifiedList(right_global_list);
    right_action_manager.addAction(right_move_to_action);
    
    % Initiliaze robot interface
    robot_udp=UDP_interface(real_robot);
    
    % Initialize logger
    logger_left=SimulationLogger(ceil(end_time/dt)+1, coop_system, left_global_list);
    logger_right=SimulationLogger(ceil(end_time/dt)+1, coop_system, right_global_list);
    
    % Set initial state
    mission_phase = "MoveTo";

    % Main simulation Loop
    for t = 0:dt:end_time
        % 1. Receive UDP packets - DO NOT EDIT
        [ql,qr]=robot_udp.udp_receive(t);
        if real_robot==true %Only in real setup, assign current robot configuration as initial configuratio
            coop_system.left_arm.q=ql;
            coop_system.right_arm.q=qr;
        end

        % 2. Update Full kinematics of the bimanual system
        coop_system.update_full_kinematics();
        
        % HERE WE CHECK AND TRIGGER STATE TRANSITIONS
      
        
        % 3. TO DO: compute the TPIK for each manipulator with your action
        % manager
        [ql_dot] = left_action_manager.computeICAT(coop_system);
        [qr_dot] = right_action_manager.computeICAT(coop_system);

        % SWITCH TO HANDLE DIFFERENT LOGIC FOR DIFFERENT PHASES/ACTIONS OF
        % THE MISSION
        disp(mission_phase);

        switch mission_phase
            case "MoveTo"
                left_ang_error = norm(left_arm.rot_to_goal);
                left_lin_error = norm(left_arm.dist_to_goal);
                right_ang_error = norm(right_arm.rot_to_goal);
                right_lin_error = norm(right_arm.dist_to_goal);

                if (left_ang_error < ang_error_threshold) && (left_lin_error < lin_error_threshold) && ...
                    (right_ang_error < ang_error_threshold) && (right_lin_error < lin_error_threshold)
               
                    disp(['Target Reached at t = ' num2str(t) '. Switching to MoveObject.']);
                    
                    % 2. Compute Relative Grasp Transforms (tTo)
                    % Using your invT function
                    tTo_L = invT(left_arm.wTt) * wTo_start;
                    tTo_R = invT(right_arm.wTt) * wTo_start;
                    
                    % 3. Set the grasp transforms in the robot objects
                    left_arm.setObjToolTransform(tTo_L);
                    right_arm.setObjToolTransform(tTo_R);
                
                    left_arm.updateTransform();
                    right_arm.updateTransform();
    
                    left_arm.updateJacobian();
                    right_arm.updateJacobian();
                    
                    % 4. Switch Action
                    left_action_manager.setCurrentAction("LeftMoveObj");
                    right_action_manager.setCurrentAction("RightMoveObj");
                    
                    % Trigger state transition
                    mission_phase = "MoveObj";
                end
                % DO STUFF
            case "MoveObj"
                disp("We're in the move phase yuuuu")
                % DO STUFF
            case "Stop"
                % DO STUFF
        end

        % 4. TO DO: COOPERATION hierarchy
        % SAVE THE NON COOPERATIVE VELOCITIES COMPUTED
    
        % 5. TO DO: compute the TPIK for each manipulator with your action
        % manager (with the constrained action to track the coop velocity)
    
        % 6. get the two variables for integration
        coop_system.sim(ql_dot,qr_dot);
        
        % 6. Send updated state to Pybullet
        robot_udp.send(t,coop_system)
    
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
