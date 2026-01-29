%% --- SEZIONE 1: ESECUZIONE SIMULAZIONE ---
% Lancia questa sezione UNA sola volta.
% Assicurati che Unity sia in Play.

clc; clear; close all;

% Chiama la funzione di simulazione definita in fondo al file
try
    logger = uvms_simulation_loop();
    fprintf('Simulazione completata. Ora puoi eseguire la sezione Plot.\n');
catch ME
    % Gestione errori pulita per chiudere la connessione Unity
    fprintf(2, 'Errore durante la simulazione: %s\n', ME.message);
    delete(instrfindall); % Pulisce porte UDP appese
end


%% --- SEZIONE 2: PLOTTING ---
% Puoi lanciare questa sezione quante volte vuoi per aggiornare i grafici
% senza dover rifare i calcoli.

if exist('logger', 'var')
    % Esempio: Plotta tutto
    close all;
    logger.plotAll();
    
    % Esempio: Plotta solo specifici task (scommenta sotto)
    % logger.plotAll({'Vehicle Position', 'Tool'});
else
    fprintf(2, 'Errore: Oggetto "logger" non trovato. Esegui prima la Sezione 1.\n');
end


%% ---------------------------------------------------------
%  --- FUNZIONI LOCALI (Devono stare in fondo al file) ---
%  ---------------------------------------------------------

function [logger] = uvms_simulation_loop()
    % Add paths
    addpath('./simulation_scripts');
    addpath('./tools');
    addpath('./icat');
    addpath('./tasks')
    addpath('./robust_robot');

    % --- Simulation parameters ---
    dt             = 0.05;      
    endTime        = 150;
    trans_duration = 5.0;         
    n_dofs         = 13;          

    % --- Initialize robot model and simulator ---
    robotModel = UvmsModel(); 
    robotModel.eta = [10.5 35.5 -36 -pi/3 pi/3 pi/2]';
    sim = UvmsSim(dt, robotModel, endTime);

    % Initialize Unity interface
    fprintf('Connessione a Unity in corso...\n');
    unity = UnityInterface("127.0.0.1");
    fprintf('Connesso.\n');

    % --- TASKS ---
    task_vehicle_pos      = TaskVehiclePos("Vehicle Position");
    task_vehicle_orient   = TaskVehicleOrient("Vehicle Orientation");
    task_horizontal_att   = TaskHorizontalAtt("Horizontal Attitude");
    task_altitude_min     = TaskAltitudeControl("Minimum Altitude", 1); 
    task_altitude_min.set_h_min(1);
    task_altitude_landing = TaskAltitudeControl("Landing Altitude", 0);
    task_altitude_landing.set_h_star(0);
    task_alignment        = TaskAlignment("Alignment");
    task_workspace        = TaskWorkSpace("Workspace");
    task_tool             = TaskTool("Tool");
    task_stop             = TaskStop("Stop");

    % --- ACTIONS ---
    action_safe_navigation = Action("SafeNavigation", ...
        {task_altitude_min, task_horizontal_att, task_vehicle_pos, task_vehicle_orient});

    action_alignment = Action("Alignment", ...
        {task_horizontal_att, task_alignment, task_workspace});

    action_landing = Action("Landing", ...
        {task_horizontal_att, task_alignment, task_workspace, task_altitude_landing}); 

    action_manipulation = Action("Manipulation", ...
        {task_stop, task_tool});

    % --- UNIFIED LIST ---
    unified_list = {task_stop, ...                
                    task_altitude_min, ...        
                    task_horizontal_att, ...      
                    task_alignment, ...           
                    task_workspace, ...           
                    task_vehicle_pos, ...         
                    task_vehicle_orient, ...      
                    task_altitude_landing, ...    
                    task_tool                     
                    };

    % --- ActionManager Initialization ---
    actionManager = ActionManager(dt, n_dofs, trans_duration);
    actionManager.addUnifiedList(unified_list);
    actionManager.addAction(action_safe_navigation);
    actionManager.addAction(action_alignment);
    actionManager.addAction(action_landing); 
    actionManager.addAction(action_manipulation);
    actionManager.setCurrentAction("SafeNavigation"); 

    % --- Goals ---
    w_arm_goal_position     = [12.2025, 37.3748, -39.8860]';
    w_arm_goal_orientation  = [0, pi, pi/2];
    w_vehicle_goal_position = [10.5, 37.5, -38]';  
    w_vehicle_goal_orientation = [0 ,-0.06, 0.5];
    
    robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

    % --- Estrazione nomi azioni per Logger ---
    action_names_list = cell(1, length(actionManager.actions));
    for k = 1:length(actionManager.actions)
        action_names_list{k} = actionManager.actions(k).name;
    end

    % Initialize logger
    logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, unified_list, action_names_list);

    % --- Main simulation loop ---
    fprintf('Avvio ciclo di simulazione...\n');
    
    % Try-Catch interno per garantire la chiusura di Unity
    try
        for step = 1:sim.maxSteps
            % 1. Receive data
            robotModel.altitude = unity.receiveAltitude(robotModel);
            
            % 2. Logic
            current_action_name = actionManager.getCurrentActionName();
            
            switch current_action_name
                case "SafeNavigation"
                    [vehicle_ang_err_vec, vehicle_lin_err_vec] = CartError(robotModel.wTgv, robotModel.wTv);
                    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
                        fprintf('ACTION: SafeNavigation [t = %.2f s] ErrLin: %.3f\n', sim.time, norm(vehicle_lin_err_vec));
                    end
                    if (norm(vehicle_ang_err_vec) < 0.02) && (norm(vehicle_lin_err_vec) < 0.05)
                        actionManager.setCurrentAction("Alignment");
                    end
                    
                case "Alignment"
                    if abs(task_alignment.theta) < 0.01
                         actionManager.setCurrentAction("Landing");
                    end
    
                case "Landing"
                    if robotModel.altitude < 0.05 && abs(task_alignment.theta) < 0.01
                         actionManager.setCurrentAction("Manipulation");
                    end
            end
    
            % 3. Control
            if sim.time > 1.0
                [v_nu, q_dot] = actionManager.computeICAT(robotModel);
            else
                v_nu = zeros(6,1);
                q_dot = zeros(7,1);
            end
    
            % 4. Integration & Communication
            sim.step(v_nu, q_dot);
            unity.send(robotModel);
            
            % 5. Logging
            logger.update(sim.time, sim.loopCounter, actionManager.currentAction_idx);
        end
        
        delete(unity);
        
    catch ME
        delete(unity);
        rethrow(ME);
    end
end