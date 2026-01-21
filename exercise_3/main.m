function main()
    %Add path
    addpath('./simulation_scripts');
    addpath('./tools')
    addpath('./icat')
    addpath('./tasks')
    clc;clear;close all; 


    % Simulation Parameters
    dt = 0.005;
    end_time = 20;
    
    % Initialize Franka Emika Panda Model
    model = load("panda.mat");
    
    % Simulation Setup
    real_robot = false;
    
    % Initiliaze panda_arm() Class, specifying the base offset w.r.t World Frame
    arm1 = panda_arm(model,eye(4));
    % Transform from world frame to arm2 base frame
    wTb2 = [-1  0   0   1.06;
            0   -1  0   -0.01;
            0   0   1   0;
            0   0   0   1];
    arm2 = panda_arm(model, wTb2);
    
    % Initialize Cooperative Simulator Class
    coop_system = coop_sim(dt, arm1, arm2, end_time);
    
    % Define Object Shape and origin Frame
    obj_length = 0.06;
    w_obj_pos = [0.5 0 0.59]';
    w_obj_ori = rotation(0, 0, 0);
    
    % Set goal frames for left and right arm, based on object frame
    %TO DO: Set arm goal frame based on object frame.
    offset = (obj_length/2) - 0.01; % offset with a margin to not take the obj exactly at the end
    linear_offset = [offset 0 0]';

    arm1.setGoal(w_obj_pos, w_obj_ori, -linear_offset, rotation(pi, -pi/9, 0));
    arm2.setGoal(w_obj_pos, w_obj_ori, +linear_offset, rotation(pi, pi/9, 0));
    
    % Define Object goal frame (Cooperative Motion)
    % wTog=[rotation(0,0,0) [0.6, -0.4, 0.48]'; 0 0 0 1];   
    wTog=[rotation(0,0,0) [0.6, 0.4, 0.48]'; 0 0 0 1];
    arm1.set_obj_goal(wTog);
    arm2.set_obj_goal(wTog);
    
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
    
    left_global_list = {left_joint_task, ...
                   left_min_alt_task, ...
                   left_tool_task};
    right_global_list = {right_joint_task, ...
                   right_min_alt_task, ...
                   right_tool_task};

    % ARRIVATI FINO A QUI MANCANO ANCORA GLI ACTION MANAGER

    % TO DO: Create two action manager objects to manage the tasks of a single
    % manipulator (one for the non-cooperative and one for the cooperative steps
    % of the algorithm)
    
    % Initiliaze robot interface
    robot_udp=UDP_interface(real_robot);
    
    % Initialize logger
    logger_left=SimulationLogger(ceil(end_time/dt)+1,coop_system.left_arm);
    logger_right=SimulationLogger(ceil(end_time/dt)+1,coop_system.right_arm);
    %Main simulation Loop
    for t = 0:dt:end_time
        % 1. Receive UDP packets - DO NOT EDIT
        [ql,qr]=robot_udp.udp_receive(t);
        if real_robot==true %Only in real setup, assign current robot configuration as initial configuratio
            coop_system.left_arm.q=ql;
            coop_system.right_arm.q=qr;
        end
        % 2. Update Full kinematics of the bimanual system
        coop_system.update_full_kinematics();
        
        % 3. TO DO: compute the TPIK for each manipulator with your action
        % manager
    
        % 4. TO DO: COOPERATION hierarchy
        % SAVE THE NON COOPERATIVE VELOCITIES COMPUTED
    
        % 5. TO DO: compute the TPIK for each manipulator with your action
        % manager (with the constrained action to track the coop velocity)
    
        % 6. get the two variables for integration
        coop_system.sim(ql_dot,qr_dot);
        
        % 6. Send updated state to Pybullet
        robot_udp.send(t,coop_system)
    
        % 7. Loggging
        logger_left.update(coop_system.time,coop_system.loopCounter)
        logger_right.update(coop_system.time,coop_system.loopCounter)
        coop_system.time
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
