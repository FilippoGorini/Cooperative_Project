% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
clc; clear; close all;

% Simulation parameters
dt       = 0.05;            % Prima era 0.005
endTime  = 50;
% Initialize robot model and simulator
robotModel = UvmsModel(); 
% Set initial conditions
robotModel.eta = [10.5 35.5 -36 0 0 pi/2]';
% Create sim object 
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% Define tasks 
task_vehicle_pos = TaskVehiclePos();
task_horizontal_att = TaskHorizontalAtt();
task_altitude_control = TaskAltitudeControl();
task_set = {task_altitude_control, task_vehicle_pos, task_horizontal_att};         % The order in which I place the tasks sets the priority

% Define actions and add to ActionManager
actionManager = ActionManager();
actionManager.addAction(task_set); 

% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [12.2025 37.3748 -39.8860]';    % Unreachable goal (too close to seafloor)
% w_vehicle_goal_position = [10.5 37.5 -38]';     % Reachable goal
w_vehicle_goal_orientation = [0, 0, 0];

% Set goals in the robot model
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, task_set);

% Main simulation loop
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);

    % 2. Compute control commands for current action
    % In the first few loops the altitude is not yet received from
    % unity so this leads to an empty activation, so we skip the ICAT
    % for the first 0.5 seconds
    if step * dt > 0.5
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

    % 6. Optional debug prints
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
        [~, lin_pos_error] = CartError(robotModel.wTgv , robotModel.wTv);
        fprintf('lin_pos_error = %.4f m\n', lin_pos_error);
        [~, mis_error] = RotToAngleAxis(robotModel.wTv(1:3,1:3));
        fprintf('misalignment_error = %.4f rad\n', mis_error);
    end

    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
end

% Display plots
logger.plotAll();

% Clean up Unity interface
delete(unity);