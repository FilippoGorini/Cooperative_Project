% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./tasks')
addpath('./robust_robot');
clc; clear; close all;

% --- Simulation parameters ---
dt             = 0.05;      
endTime        = 200;
trans_duration = 5.0;         % 5 s transition duration between actions
n_dofs         = 13;          % 6 (vehicle) + 7 (arm)

% --- Initialize robot model and simulator ---
robotModel = UvmsModel(); 
% Set initial conditions
%robotModel.eta = [48.5 11.5 -33 0 0 pi/2]';
robotModel.eta = [10.5 11.5 -33 0 0 pi/2]';
sim = UvmsSim(dt, robotModel, endTime);

% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% --- TASKS ---
task_vehicle_pos      = TaskVehiclePos();
task_vehicle_orient   = TaskVehicleOrient();
task_horizontal_att   = TaskHorizontalAtt();
task_altitude_min     = TaskAltitudeControl(1); 
task_altitude_min.set_h_min(1);
task_altitude_landing = TaskAltitudeControl(0);
task_altitude_landing.set_h_star(0);

% --- ACTIONS ---
% Priorities are decided by the unified_list order, not the order here.
action_safe_navigation = Action("SafeNavigation", ...
    {task_altitude_min, task_vehicle_pos, task_vehicle_orient, task_horizontal_att});

action_landing = Action("Landing", ...
    {task_altitude_landing, task_vehicle_pos, task_horizontal_att});

% --- UNIFIED LIST (Highest to Lowest Priority) ---
unified_list = {task_altitude_min, ...       
                task_altitude_landing, ...
                task_vehicle_pos, ...
                task_vehicle_orient, ...
                task_horizontal_att};

% --- ActionManager Initialization ---
actionManager = ActionManager(dt, n_dofs, trans_duration);
actionManager.addUnifiedList(unified_list);
actionManager.addAction(action_safe_navigation); 
actionManager.addAction(action_landing); 
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
                fprintf('  Switching to: Landing\n\n');
                actionManager.setCurrentAction("Landing");
            end
            
        case "Landing"
            if mod(sim.loopCounter, round(1 / sim.dt)) == 0
                fprintf('ACTION: Landing [t = %.2f s]\n', sim.time);
                fprintf('  Current Altitude: %.4f m\n', robotModel.altitude);
            end
            
            % Optional: Add logic to stop simulation if altitude ~ 0
            if robotModel.altitude < 0.05
                 % Handle landing completion if necessary
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