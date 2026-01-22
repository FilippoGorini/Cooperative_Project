classdef panda_arm < handle
    % Franka Emika Panda Kinematic Model

    properties
        %% Rigid Body Tree
        robot_model
        %% --- State variables ---
        q
        qdot
        %% --- Geometry ---
        wTb
        %% --- Limits ---
        jlmin
        jlmax
        %% ---Transformations ---
        bTe
        wTe
        eTt
        wTt
        %% ---Goals---
        wTg
        wTog
        wTo
        robot_type
        tTo     % Not defined at the beginning, must be set with set_grasp_transform
        wJt
        wJo % Jacobian of object's frame: this is our new control point once the manipulators grasp the object
        % wJo is not defined in the beginning, until we have tTo available
        wJe % <--- AGGIUNTO: Jacobian End Effector
        %% Sensor variables
        alt
        dist_to_goal = 1000 % big default value at start to ensure we don't think to have reached the goal already until they're not calculated
        rot_to_goal = 1000
    end

    methods
        function obj = panda_arm(model,wTb)
            % Constructor
            obj.robot_model = model;
            obj.wTb = wTb;
            
            %Initialize Default State
            obj.q=[0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
            obj.qdot=[0 0 0 0 0 0 0]';
            
            %Get current transformation from world frame to end effector
            %Frame
            obj.bTe=getTransform(obj.robot_model.franka,[obj.q',0,0],'panda_link7');
            obj.wTe=obj.wTb*obj.bTe;

            %Default Limits
            obj.jlmin=[-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
            obj.jlmax=[2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

            % FIXED END EFFECTOR
            % frame e = frame f = frame 8
            psi = -44.9949 * (pi/180);% FIXED ANGLE BETWEEN EE AND TOOL 
            tool_length = 0.2124;% FIXED DISTANCE BETWEEN EE AND TOOL
            
            eRt = YPRToRot(psi, 0, 0);
            eOt = [0 0 tool_length]';
            % TO DO: Define trasnformation matrix from ee to tool, and
            % transformation from world frame to tool
            obj.eTt = [eRt,         eOt;
                       zeros(1,3),  1];
            obj.wTt = obj.wTe * obj.eTt;

            % Initialize tTo, wTo and wJo to avoid dimension problems at startup
            obj.tTo = zeros(4,4);
            obj.wTo = zeros(4,4);
            obj.wJo = zeros(6,7);

        end

        function setGoal(obj,obj_position,obj_orientation,arm_dist_offset,arm_rot_offset)
            % Set goal positions and orientations for arm 
            obj.wTo=[[obj_orientation obj_position]; 0 0 0 1];
            %obj.wTg=[[arm_rot_offset arm_dist_offset]; 0 0 0 1]; del %template
            
            oTg = [[arm_rot_offset arm_dist_offset]; 0 0 0 1];

            obj.wTg = obj.wTo * oTg;            

        end
        
        function set_grasp_transform(obj, tTo_val)
             obj.tTo = tTo_val;
             % obj.wTo = obj.wTt * obj.tTo;
        end

        function set_obj_goal(obj,wTog)
            % Set goal positions and orientations for the object
            obj.wTog = wTog;
        end

        function update_transform(obj)
            % Compute forward kinematics of the robot

            obj.bTe=getTransform(obj.robot_model.franka,[obj.q',0,0],'panda_link7');
            obj.wTe=obj.wTb*obj.bTe;
            % obj.wTt =obj.wTe;
            obj.wTt = obj.wTe * obj.eTt;
            if ~isempty(obj.tTo)
                obj.wTo = obj.wTt * obj.tTo;
            end
        end
        
        function update_jacobian(obj)
            % Compute Differential kinematics from the base frame to the
            % Tool Frame
            bJe = geometricJacobian(obj.robot_model.franka,[obj.q',0,0],'panda_link7');%DO NOT EDIT

            % Ruota lo Jacobiano nel World Frame (wJe)
            wRb = obj.wTb(1:3,1:3);
            Rot_block = [wRb zeros(3,3); zeros(3,3) wRb];
            % Salvataggio wJe 
            obj.wJe = Rot_block * bJe(:, 1:7);
            w_r_et = obj.wTe(1:3,1:3) * obj.eTt(1:3,4);
            % NB: we renamed Ste to Set to follow naming convention
            Set = [eye(3) zeros(3); -skew(w_r_et) eye(3)];
            obj.wJt = Set * obj.wJe;
            % obj.wJt = Set * [obj.wTb(1:3,1:3) zeros(3,3); zeros(3,3) obj.wTb(1:3,1:3)] * bJe(:, 1:7);
            
            % We update the jacobian of the object only once we set the
            % transform of the object wrt to the tool (tTo)
            if ~isempty(obj.tTo)
                % w_r_to is vector from Tool to Object in World Frame
                % wRt * tP_o
                w_r_to = obj.wTt(1:3,1:3) * obj.tTo(1:3,4);
                
                % Twist transformation matrix from Tool to Object
                % [ I   0 ]
                % [ -S(r) I ]
                Sto = [eye(3) zeros(3); -skew(w_r_to) eye(3)];
                
                obj.wJo = Sto * obj.wJt;
            end



        end
    end
end