% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./tasks')
addpath('./robust_robot');
clc; clear; close all;

% --- Simulation parameters ---
dt             = 0.05;      
endTime        = 250;
trans_duration = 5.0;         % 5 s transition duration between actions
n_dofs         = 13;          % 6 (vehicle) + 7 (arm)

% --- Initialize robot model and simulator ---
robotModel = UvmsModel(); 
% Set initial conditions
robotModel.eta = [10.5 35.5 -36 -pi/3 pi/3 pi/2]';
sim = UvmsSim(dt, robotModel, endTime);

% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% --- TASKS ---
task_vehicle_pos      = TaskVehiclePos("Vehicle Position");
task_vehicle_orient   = TaskVehicleOrient("Vehicle Orientation");
task_horizontal_att   = TaskHorizontalAtt("Horizontal Attitude");
task_altitude_min     = TaskAltitudeControl("Minimum Altitude",1); 
task_altitude_min.set_h_min(1);
task_altitude_landing = TaskAltitudeControl("Landing Altitude",0);
task_altitude_landing.set_h_star(0);
task_alignment        = TaskAlignment("Alignment");
task_workspace        = TaskWorkSpace("Workspace");
task_tool             = TaskTool("Tool");
task_stop             = TaskStop("Stop");

% --- ACTIONS ---
% Priorities are decided by the unified_list order, not the order here.
action_safe_navigation = Action("SafeNavigation", ...
    {task_altitude_min, task_vehicle_pos, task_vehicle_orient, task_horizontal_att});

action_alignment = Action("Alignment", ...
    {task_horizontal_att, task_alignment, task_workspace});

action_landing = Action("Landing", ...
    {task_altitude_landing, task_horizontal_att, task_alignment, task_workspace}); % ho tolto task_vehicle_pos

action_manipulation = Action("Manipulation", ...
    {task_stop, task_tool});

% --- UNIFIED LIST (Highest to Lowest Priority) ---
unified_list = {task_stop, ...                % constraint
                task_altitude_min, ...        % safety
                task_horizontal_att, ...      % prerequisite
                task_alignment, ...           % prerequisite
                task_workspace, ...           % prerequisite
                task_vehicle_pos, ...         % action def
                task_vehicle_orient, ...      % action def
                task_altitude_landing, ...    % action def
                task_tool                     % action def
                };

% --- ActionManager Initialization ---
actionManager = ActionManager(dt, n_dofs, trans_duration);
actionManager.addUnifiedList(unified_list);
actionManager.addAction(action_safe_navigation);
actionManager.addAction(action_alignment);
actionManager.addAction(action_landing); 
actionManager.addAction(action_manipulation);
actionManager.setCurrentAction("SafeNavigation"); 

% --- Goals and Thresholds ---
w_arm_goal_position     = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation  = [0, pi, pi/2];
w_vehicle_goal_position = [10.5, 37.5, -38]';  
w_vehicle_goal_orientation = [0 ,-0.06, 0.5];

ang_error_threshold = 0.02;
lin_error_threshold = 0.05;

% Set goals in the robot model
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, unified_list);

% --- Main simulation loop ---
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);
    
    % --- Mission Phase Logic (Switch-Case) ---
    switch actionManager.getCurrentActionName()
        
        case "SafeNavigation"
            % Compute vehicle errors
            [vehicle_ang_err_vec, vehicle_lin_err_vec] = CartError(robotModel.wTgv, robotModel.wTv);
            vehicle_ang_error = norm(vehicle_ang_err_vec);
            vehicle_lin_error = norm(vehicle_lin_err_vec);
            
            % Debug prints every 1 second
            if mod(sim.loopCounter, round(1 / sim.dt)) == 0
                fprintf('ACTION: SafeNavigation [t = %.2f s]\n', sim.time);
                fprintf('  Vehicle: Lin Err = %.4f | Ang Err = %.4f | Alt = %.2f\n', ...
                    vehicle_lin_error, vehicle_ang_error, robotModel.altitude);
            end
            
            % Check transition criteria
            if (vehicle_ang_error < ang_error_threshold) && (vehicle_lin_error < lin_error_threshold)
                fprintf('VEHICLE GOAL REACHED [%.2f s]\n', sim.time);
                fprintf('  Switching to: Alignment\n\n');
                actionManager.setCurrentAction("Alignment");
            end
            
        case "Alignment"
            if mod(sim.loopCounter, round(1 / sim.dt)) == 0
                fprintf('ACTION: Aligning [t = %.2f s]\n', sim.time);
                fprintf('  Current Alignment Error: %.4f rad\n', task_alignment.theta);
            end
            
            if abs(task_alignment.theta) < 0.01
                 fprintf('ALIGNMENT COMPLETED [%.2f s]\n', sim.time);
                 fprintf('  Switching to: Landing\n\n');
                 actionManager.setCurrentAction("Landing");
            end

        case "Landing"
            if mod(sim.loopCounter, round(1 / sim.dt)) == 0
                fprintf('ACTION: Landing [t = %.2f s]\n', sim.time);
                fprintf('  Current Altitude: %.4f m\n', robotModel.altitude);
            end
            
            if robotModel.altitude < 0.05 && abs(task_alignment.theta) < 0.01
                 fprintf('LANDING COMPLETED [%.2f s]\n', sim.time);
                 fprintf('  Switching to: Manipulation\n\n');
                 actionManager.setCurrentAction("Manipulation");
            end
        case "Manipulation"
            if mod(sim.loopCounter, round(1 / sim.dt)) == 0
                fprintf('ACTION: Manipulation [t = %.2f s]\n', sim.time);
                fprintf('  Current Altitude: %.4f m\n', robotModel.altitude);
            end
    end

    % 2. Compute control commands for current action
    % Skip ICAT for the first second to allow Unity/Sensors to settle
    if sim.time > 1.0
        [v_nu, q_dot] = actionManager.computeICAT(robotModel);
    else
        v_nu = zeros(6,1);
        q_dot = zeros(7,1);
    end

    % 3. Step the simulator (integrate velocities)
    sim.step(v_nu, q_dot);

    % 4. Send updated state to Unity
    unity.send(robotModel);

    % 5. Logging
    logger.update(sim.time, sim.loopCounter);
end

% --- Post-Simulation ---
logger.plotAll();
delete(unity);