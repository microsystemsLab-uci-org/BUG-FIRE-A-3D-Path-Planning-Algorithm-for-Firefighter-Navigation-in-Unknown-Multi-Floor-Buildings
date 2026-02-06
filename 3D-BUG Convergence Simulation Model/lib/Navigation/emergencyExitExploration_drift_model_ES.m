%% Caption
% EMERGENCYEXITEXPLORATION_DRIFT_MODEL_ES  Emergency-exit exploration with vision-cone perception and ZUPT-aided INS drift (E. Sangenis)
%
% A concrete subclass of the abstract Navigation class implementing an original
% reactive exploration strategy for emergency egress. The algorithm navigates on an
% occupancy-grid map using local sensing only (vision cone + obstacle contact) to
% discover and prioritize emergency exits and directional emergency signs.
%
% Core features implemented by Eudald Sangenis::
%   • Vision-cone perception model (radius/angle configurable) with ray-casting to
%     determine line-of-sight visibility of exits and signs.
%   • Online goal management: dynamic re-targeting to the closest visible exit, or
%     to unvisited emergency signs, with sign-following behavior (left/right/front/back).
%   • Reactive obstacle avoidance via a two-state automaton (goal-seeking + wall-following),
%     including m-line updates when the goal changes.
%   • Coupled drift simulation for the navigation estimate using a ZUPT-aided pedestrian
%     INS error-growth model driven by IMU parameters (ARW/RRW/VRW), producing:
%       - true trajectory (ground truth on the grid),
%       - estimated/drifted trajectory (INS-like),
%       - propagated yaw and position-error state over walked distance.
%   • Drift/covariance logging for post-run analysis (error vs. distance/time, incremental
%     and cumulative σ|| and σ⊥ growth).
%   • Support for drift-state inheritance between phases via constructor options
%     initialDriftState and initialDistance.
%
% Methods::
%   emergencyExitExploration_drift_model_ES  Constructor (supports initialDriftState / initialDistance)
%   query                 Explore toward goal while prioritizing exits/signs; returns true+drifted paths
%   next                  Main reactive state machine step with drift injection
%   applyDrift             ZUPT-aided INS drift propagation per step (IMU-parameter dependent)
%   plotVisionCone         Render vision cone and update visible points of interest
%   computeP44             Helper for covariance term used in drift model
%
% Example::
%         bug = emergencyExitExploration_drift_model_ES(map, 'initialDistance', 0);
%         start = [20,10,0];
%         goal  = [50,35,0];
%         [pp_est, pp_true, goalReached] = bug.query(start, goal, 'animate', ...
%                               'visionRadius', 30, 'visionAngle', 170);
%
% Reference (drift model)::
% - “Study on Estimation Errors in ZUPT-Aided Pedestrian Inertial Navigation Due to IMU Noises.”
%
% See also Navigation, Bug2_drift_model_ES.
%
% Copyright (C) 2026, Eudald Sangenis

%% Class
classdef emergencyExitExploration_drift_model_ES < Navigation
    properties(Access=protected)
        H                       % hit points
        j                       % number of hit points
        mline                   % line from starting position to goal
        step                    % state, in step 1 or step 2 of algorithm
        edge                    % edge list
        k                       % edge index
        visionConeHandle        % Handle for the vision cone plot
        exitCells               % Array to keep track of location of the emergency exits
        signCells               % Container of arrays to keep track of location and directions of the emergency signs
        spottedExitCells        % Array to keep track of spotted emergency exits
        spottedSigns            % Array to keep track of spotted signs
        spottedSignsPositions   % Cell array to store positions of spotted signs
        spottedSignsTypes       % Cell array to store types of spotted signs
        goalLineHandle          % Handle for the line between robot and current goal
        originalGoal            % Store the original goal
        visitedSigns            % Array to keep track of visited emergency signs
        observedOccGrid
        isWallFollowing = false
        visionRadius
        visionAngle

        cumulativeDistance      % total walked distance in meters (true)
        driftState              % struct: yaw_err_rad, pos_err_m, acc_step_m

        % Degub Drift Parameters:
        driftLog_time_s         % 1×N  equivalent time
        driftLog_dist_m         % 1×N  cumulative distance
        driftLog_posErr_m       % 2×N  [ex; ey] estimated pos error in meters
        driftLog_yawErr_rad     % 1×N  yaw error
        driftLog_sigpar_m       % 1×N  incremental parallel STD (model)
        driftLog_sigperp_m      % 1×N  incremental perpendicular STD (model)
        driftLog_sigparCum_m
        driftLog_sigperpCum_m
    end 
    
    methods
     
        function bug = emergencyExitExploration_drift_model_ES(varargin)
            initialDriftState = struct('yaw_err_rad', 0, 'pos_err_m', [0;0], 'acc_step_m', 0);
            initialDistance = 0;
            superArgs = varargin;

            % NEW: Extract initialDriftState
            idx = find(strcmpi(superArgs, 'initialDriftState'));
            if ~isempty(idx) && idx < length(superArgs)
                initialDriftState = superArgs{idx+1};
                superArgs([idx, idx+1]) = [];
            end
            
            % NEW: Extract initialDistance
            idx = find(strcmpi(superArgs, 'initialDistance'));
            if ~isempty(idx) && idx < length(superArgs)
                initialDistance = superArgs{idx+1};
                superArgs([idx, idx+1]) = [];
            end

            % Bug3_ES Construct a Bug3 navigation object 
            bug = bug@Navigation(superArgs{:});

            bug.H = zeros(1, 2);
            bug.j = 1;
            bug.step = 1;
            bug.visionConeHandle = []; % Initialize vision cone handle
            bug.exitCells = []; % Initialize stop cells
            bug.signCells.location = []; % Initialize stop cells
            bug.signCells.direction = []; % Initialize stop cells
            bug.spottedExitCells = []; % Initialize spotted emergency exit cells
            bug.spottedSigns = []; % Initialize spotted emergency sign cells
            bug.spottedSignsPositions = {};
            bug.spottedSignsTypes = {};
            bug.goalLineHandle = [];  % Initialize goal line handle
            bug.originalGoal = [];  % Initialize original goal
            bug.visitedSigns = [];
            bug.observedOccGrid = zeros(size(bug.occgridnav));  % Initialize with unknown space
            bug.visionRadius = 30; % Default value 30 m
            bug.visionAngle = 170; % Default value 170 deg cone vision

            % Initialize with cumulative drift state
            bug.cumulativeDistance = initialDistance;
            bug.driftState = initialDriftState;

            % Initialize Degub Drift Parameters
            bug.driftLog_time_s     = [];
            bug.driftLog_dist_m     = [];
            bug.driftLog_posErr_m   = [];
            bug.driftLog_yawErr_rad = [];
            bug.driftLog_sigpar_m   = [];
            bug.driftLog_sigperp_m  = [];
            bug.driftLog_sigparCum_m  = [];
            bug.driftLog_sigperpCum_m = [];
        end

%% Set Cells Functions 
%         function setExitCells(bug, cells) % Set stop cells 
%             % cells: Nx2 matrix, where each row is a [x, y] position
%             bug.exitCells = cells;
%         end
% 
%         function setSignCellsLocation(bug, cells)
%             % Set stop cells
%             % cells: Nx2 matrix, where each row is a [x, y] position
%             bug.signCells.location = cells;
%         end
%     
%         function setSignCellsDirection(bug, direction)
%             % Set stop cells
%             % cells: Nx2 matrix, where each row is a [x, y] position
%             bug.signCells.direction = direction;
%         end
%        
%         function state = getDriftState(bug)
%             % Get the current drift state
%             state = bug.driftState;
%         end
%         
%         function dist = getCumulativeDistance(bug)
%             % Get the cumulative distance traveled
%             dist = bug.cumulativeDistance;
%         end
        
        function [drifted_pos, cumulative_dist, drift_state, sigma_par_inc, sigma_perp_inc, sigpar2_curr, sigperp2_curr, t_curr] = applyDrift( ...
            bug, prev_true, next_true, cumulative_dist, drift_state)

            % APPLYDRIFT  ZUPT-aided INS drift model (distance–based incremental form)
            %
            % Implements the ZUPT-aided pedestrian INS error-growth model from:
            %   "Study on Estimation Errors in ZUPT-Aided Pedestrian Inertial
            %    Navigation Due to IMU Noises."
            %
            %  • uses RIGHT normal (east) for e_perp,
            %  • adds yaw-coupled lateral term,
            %  • adds deterministic right-drift bias per meter.
        
            % --------------------------------------------------------------
            % constants
            % --------------------------------------------------------------
            CELLS_PER_METER = 4;
            DEG_TO_RAD      = pi/180;
        
            % VN-200 / paper-like parameters (Industrial Grade IMU)
            ARW_deg_sqrt_hr   = 0.21;           % Angle Random Walk [deg/sqrt(hr)]
            RRW_deg_s_sqrt_hr = 0.048;          % Rate Random Walk [deg/s/sqrt(hr)]
            VRW_mg_sqrtHz     = 0.14;           % Velocity Random Walk [mg/√Hz]

            % ADIS 16490 (Tactical Grade IMU)
%             ARW_deg_sqrt_hr   = 0.09;           % Angle Random Walk [deg/sqrt(hr)]
%             RRW_deg_s_sqrt_hr = 8e-5;           % Rate Random Walk [deg/s/sqrt(hr)]
%             VRW_mg_sqrtHz     = 0.0136;         % Velocity Random Walk [mg/√Hz]

            fs       = 800;                     % IMU sampling rate [Hz]
            t_stride = 1.54;                    % stride period [s]
            t_stance = 1.54*0.6;                % stance duration [s]
            stride_length = 1.4;                % stride length [m]
            r  = 0.02;                          % [m/s]
            a_c = 0.84;
            g = 9.81;                           % [m/s^2]
        
            % convert to SI
            ARW = ARW_deg_sqrt_hr   * DEG_TO_RAD / sqrt(3600);  % [rad/√s]
            RRW = RRW_deg_s_sqrt_hr * DEG_TO_RAD / sqrt(3600);  % [rad/s/√s]
            VRW = VRW_mg_sqrtHz * g / 1000;                     % [(m/s)/√Hz]  / sqrt(fs)
        
            % --------------------------------------------------------------
            % 1) distance and equivalent time
            % --------------------------------------------------------------
            step_dist_m     = norm(next_true - prev_true) / CELLS_PER_METER;
            cumulative_dist = cumulative_dist + step_dist_m;
        
            if isempty(drift_state)
                drift_state = struct('yaw_err_rad',0,'pos_err_m',[0;0]);
            end
        
            % walking speed and equivalent time
            walking_speed = stride_length / t_stride;
            t_curr        = cumulative_dist / walking_speed;
            t_prev        = max(cumulative_dist - step_dist_m, 0) / walking_speed;
        
            % --------------------------------------------------------------
            % 2) position variances from paper (Eqs. 56–57)
            % --------------------------------------------------------------
            term_common = (2 - t_stride/4) * (r^2 * t_stride) / (fs * t_stance);
            sN2 = stride_length^2;
        
            % parallel variance
            sigpar2_curr = term_common * t_curr;
            sigpar2_prev = term_common * t_prev;
        
            % perpendicular variance parts
            as = 0.6;   % avg(sin theta), typical 0.5–0.7
            [P44_11, ~, ~] = emergencyExitExploration_drift_model_ES.computeP44(ARW, RRW, VRW, r, a_c, fs, t_stride, t_stance, g);
            sigma_gn2 = sqrt(P44_11);
            c_t4 = (as^2 / 12) * sigma_gn2;
        
            sigperp2_curr = term_common * t_curr ...
                          + (1/3)*(ARW^2)*sN2*t_curr^3 ...
                          + c_t4 * sN2 * t_curr^4 ...
                          + (1/30 + a_c^2/60)*(RRW^2)*sN2*t_curr^5;
        
            sigperp2_prev = term_common * t_prev ...
                          + (1/3)*(ARW^2)*sN2*t_prev^3 ...
                          + c_t4 * sN2 * t_prev^4 ...
                          + (1/30 + a_c^2/60)*(RRW^2)*sN2*t_prev^5;
        
            % incremental variances
            d_sigpar2     = max(sigpar2_curr  - sigpar2_prev, 0);
            d_sigperp2    = max(sigperp2_curr - sigperp2_prev, 0);
            sigma_par_inc  = sqrt(d_sigpar2);
            sigma_perp_inc = sqrt(d_sigperp2);
        
            % --------------------------------------------------------------
            % 3) yaw error growth (zero-mean random walk)
            % --------------------------------------------------------------
            sigpsi2_curr = ARW^2 * t_curr + (RRW^2 / 3) * t_curr^3;
            sigpsi2_prev = ARW^2 * t_prev + (RRW^2 / 3) * t_prev^3;
            d_sigpsi2 = max(sigpsi2_curr - sigpsi2_prev, 0);
            drift_state.yaw_err_rad = drift_state.yaw_err_rad + sqrt(d_sigpsi2)*randn();

            % 3.1) Scale yaw-coupling and deterministic lateral bias by IMU grade
            % Baselines (VN-200): ARW0, VRW0 are reference magnitudes
            ARW0 = 0.21;         % deg/sqrt(hr)  (industrial baseline)
            VRW0 = 0.14;         % mg/sqrt(Hz)   (industrial baseline)
            
            scale_arw = min(1.0, ARW_deg_sqrt_hr / ARW0);    % <=1 for better IMUs
            scale_vrw = min(1.0, VRW_mg_sqrtHz   / VRW0);    % <=1 for better IMUs
        
            % --------------------------------------------------------------
            % 4) local directions (RIGHT normal (local frame))
            % --------------------------------------------------------------
            d_true = (next_true - prev_true);
            if norm(d_true) < 1e-9
                t_hat = [1;0];
            else
                t_hat = d_true / norm(d_true);
            end
            % RIGHT normal (east), consistent with the standalone version
            n_hat = [ t_hat(2); -t_hat(1) ];
        
            % --------------------------------------------------------------
            % 5) incremental errors: parallel + (noise + yaw-coupled + bias) perp
            % --------------------------------------------------------------
            e_par        = sigma_par_inc  * randn();
            e_perp_noise = sigma_perp_inc * randn();
        
            % lateral bias per meter (choose to match target mean, e.g., 1.5–1.82 m at 100 m)
            target_right_drift_m = 1.5 * (0.35*scale_arw + 0.65*scale_vrw);         % adjust as desired (e.g., 1.82)
            total_length_m       = 100;
            b_perp_per_m         = target_right_drift_m / total_length_m;
        
            yaw_gain_base = 0.5;                                  % manual gain
            yaw_gain      = yaw_gain_base * scale_arw^1.2;        % much smaller for tactical IMUs

            e_perp_yaw  = yaw_gain * stride_length * drift_state.yaw_err_rad;
            e_perp_bias = b_perp_per_m * step_dist_m;
        
            e_perp = e_perp_noise + e_perp_bias + e_perp_yaw; % + e_perp_yaw
        
            % --------------------------------------------------------------
            % 6) update position error (meters)
            % --------------------------------------------------------------
            pos_inc_m = e_par*t_hat + e_perp*n_hat;
            drift_state.pos_err_m = drift_state.pos_err_m + pos_inc_m;
        
            % --------------------------------------------------------------
            % 7) compute drifted position in cells
            % --------------------------------------------------------------
            drifted_pos = next_true + drift_state.pos_err_m * CELLS_PER_METER;
        
            % clamp to map
            drifted_pos(1) = max(1, min(drifted_pos(1), size(bug.occgridnav,2)));
            drifted_pos(2) = max(1, min(drifted_pos(2), size(bug.occgridnav,1)));
        end


%% Query Function function
        function [pp, pp_true, goalReached, exitStairsNumber, final_drift_offset] = query(bug, start, goal, varargin)
            opt.animate = false;
            opt.movie = [];
            opt.current = false;
            opt.visionRadius = 30; % Default value
            opt.visionAngle = 170; % Default value

            goalReached = false;
            exitStairsNumber = 0;
            startHeight = start(3);
            goalHeight = goal(3);
            bug.H = zeros(1, 2);  % Reset H at the beginning of each query
            bug.j = 1;
    
            iter = 0; % count number of next steps done to avoid initial positioning ending with emergency stairs
    
            opt = tb_optparse(opt, varargin);
            bug.visionRadius = opt.visionRadius;
            bug.visionAngle = opt.visionAngle;

            
            if ~isempty(opt.movie)
                anim = Animate(opt.movie);
                opt.animate = true;
            end
       
            % make sure start and goal are set and valid
            bug.start = []; bug.goal = [];
            bug.checkquery(start(1:2), goal(1:2));
    
           if opt.animate
                bug.plot();
                xlim([0,120]);
                axis equal
                hold on;
           end
   
            bug.originalGoal = goal(1:2);  % Store the original goal
    
            % Initial 360-degree rotation to find all points of interest
            bug.perform360Rotation(start(1:2));

            % Set initial goal if any point of interest is found
            bug.updateGoalBasedOnPointsOfInterest(start(1:2), goal(1:2));
    
            % compute the m-line
            bug.mline = homline(bug.start(1), bug.start(2), bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            
            if opt.animate
%                 bug.plot_mline();
                hold on;
            end
            
            bug.step = 1;


            robot_true = bug.start(:);
            robot_est  = bug.start(:);
            
            path_true = bug.start(:);
            path_est  = bug.start(:);
    
            while true
                % Check if an emergency exit is visible and go straight towards it
                if bug.checkAndMoveToEmergencyExit(robot_true)
                    goalReached = true;
                    break;
                end

                iter = iter + 1;
                [closestPoint, pointType] = bug.findClosestPointOfInterest(robot_true);
                if ~isempty(closestPoint)
                    if isempty(bug.goal) || ~isequal(closestPoint', bug.goal)
                        bug.goal = closestPoint';
                        disp(['Found point of interest: ' pointType ' at (' num2str(closestPoint(1)) ',' num2str(closestPoint(2)) ')']);
                    end
                else
                    % If no point of interest is found, set an arbitrary goal
                    if isempty(closestPoint) && bug.isWallFollowing == false && isempty(bug.visitedSigns)
                        disp("Set Arbitrary Goal Query!")
                        bug.setArbitraryGoal(robot_true);
                    end
                end
    
                if opt.animate
                    plot(robot_true(1), robot_true(2), 'b.', 'MarkerSize', 8); hold on;
                    plot(robot_est(1), robot_est(2), 'g.', 'MarkerSize', 8);
    
                    if opt.current
                        h = plot(robot_true(1), robot_true(2), 'ko', 'MarkerSize', 4);
                    end
                    drawnow
                    if ~isempty(opt.movie)
                        anim.add();
                    end
                    if opt.current
                        delete(h)
                    end
                end
    
                % move to next point on path
                [robot_est, robot_true, goalReached, exitStairsNumber] = bug.next(robot_true, iter, startHeight, goalHeight);
                disp("Robot Position: " + num2str(robot_true(1)) + ", " +  num2str(robot_true(2)));
                disp("Goal Reached?: " + num2str(goalReached));
                disp("Exit Stairs Number: " + num2str(exitStairsNumber));

                % Append the new point to the path
                if ~isempty(robot_true)
                    path_est  = [path_est  robot_est(:)];
                    path_true = [path_true robot_true(:)];
                end
    
                % Check if we've reached the current goal or if we're at an emergency sign
                if isempty(robot_true) || norm(robot_true - bug.goal) < 1e-6
                    if isempty(robot_true)
                        % We've stopped at an emergency sign
                        disp('Stopped at an emergency sign');
                        signIndex = bug.findSignAtPosition(path_true(:,end));
                        if ~isempty(signIndex)
                            bug.handleEmergencySign(signIndex, path_true(:,end));
                            robot_true = bug.start; % Update robot position
                        end
                    else
                        % We've reached an emergency exit
                        if any(all(abs(bug.exitCells - robot_true') < 1e-6, 2))
                            % If the goal we've reached is an exit, stop
                            exitStairsNumber = find(all(abs(bug.exitCells - robot_true') < 1e-6, 2));
                            break;
                        end
                    end
                    
                    % Continue with the original goal or the next point of interest
                    [nextGoal, ~] = bug.findClosestPointOfInterest(robot_true);
                    if ~isempty(nextGoal)
                        bug.goal = nextGoal';
                    else
                        bug.goal = goal(1:2);
                    end

                    disp("Next Goal: " + num2str(bug.goal(1)) + ", " +  num2str(bug.goal(2)));
                    
                    bug.mline = homline(robot_true(1), robot_true(2), bug.goal(1), bug.goal(2));
                    bug.mline = bug.mline / norm(bug.mline(1:2));
                    bug.step = 1;
                    bug.j = bug.j + 1;
                    bug.H(bug.j, :) = robot_true';
                end
            end
            
            if ~isempty(opt.movie)
                anim.close();
            end

                        % --- plot drift / covariance growth ---
            figure(4); 
            t = bug.driftLog_time_s;
            d = bug.driftLog_dist_m;
            posErr = bug.driftLog_posErr_m;   % 2×N
            yawErr = bug.driftLog_yawErr_rad;
            sigpar = bug.driftLog_sigpar_m;
            sigperp = bug.driftLog_sigperp_m;
            
            subplot(3,2,1);
            plot(d, vecnorm(posErr,2,1), 'LineWidth', 1.5);
            xlabel('Distance walked [m]');
            ylabel('||pos err|| [m]');
            title('Position error growth (simulated)');
            grid on;
            
            subplot(3,2,2);
            plot(d, yawErr*180/pi, 'LineWidth', 1.5);
            xlabel('Distance walked [m]');
            ylabel('Yaw error [deg]');
            title('Yaw error growth');
            grid on;
            
            subplot(3,2,3);
            plot(d, sigpar, 'LineWidth', 1.5); hold on;
            plot(d, sigperp, '--', 'LineWidth', 1.5);
            xlabel('Distance walked [m]');
            ylabel('Incremental \sigma [m]');
            legend('\sigma_{||}', '\sigma_{\perp}', 'Location', 'best');
            title('Model incremental STDs (paper-like)');
            grid on;
            
            subplot(3,2,4);
            plot(t, vecnorm(posErr,2,1), 'LineWidth', 1.5);
            xlabel('Equivalent time [s]');
            ylabel('||pos err|| [m]');
            title('Error vs. time');
            grid on;

            d = bug.driftLog_dist_m;
            posErr = bug.driftLog_posErr_m;
            posErrNorm = vecnorm(posErr,2,1);
            sigparCum  = bug.driftLog_sigparCum_m;
            sigperpCum = bug.driftLog_sigperpCum_m;
            
            subplot(3,2,5);
            plot(d, posErrNorm, 'k', 'LineWidth', 1.5); hold on;
            plot(d, sigperpCum, 'r--', 'LineWidth', 1.5);
            plot(d, sigparCum,  'b-.', 'LineWidth', 1.5);
            xlabel('Distance walked [m]');
            ylabel('Error [m]');
            legend('simulated |e|','model \sigma_\perp','model \sigma_{||}','Location','northwest');
            title('Simulated vs model (cumulative)');
            grid on;
    
            % only return the path if required
            if nargout > 0
                pp = path_est';
            end

            if nargout > 1
                pp_true = path_true'; % True path (ground truth)
            end

            % Calculate final drift offset (estimated - true position)
            if nargout > 4
                final_drift_offset = path_est(:,end) - path_true(:,end); % [dx; dy]
            else
                final_drift_offset = [];
            end
        end

%% Plot Functions (Vision Cone, Emergency Sings) 
        function plot_mline(bug, ls)      
            if nargin < 2
                ls = 'k--';
            end
            dims = axis;
            xmin = dims(1); xmax = dims(2);
            ymin = dims(3); ymax = dims(4);
            
            hold on
            if bug.mline(2) == 0
                % handle the case that the line is vertical
                plot([bug.start(1) bug.start(1)], [ymin ymax], 'k--');
            else
                x = [xmin xmax]';
                y = -[x [1;1]] * [bug.mline(1); bug.mline(3)] / bug.mline(2);
                plot(x, y, ls);
                xlim([0,120]);
                axis equal
            end
        end

        function [handle, coneVertices] = plotVisionCone(bug, robot, heading) % 30,170
            visionAngle_var = bug.visionAngle*pi/180;
            visionRadius_var = bug.visionRadius*4; % 30, 10, 5 meters (4 conversion pxl to meters)
            expansionDistance = 1; % Distance to expand the cone in all directions to draw the cone until the wall
            
            % Increase the number of points for more detail
            numPoints = 100;

            % Plot the vision cone based on the robot's position, angle, and radius
            theta = linspace(-visionAngle_var/2, visionAngle_var/2, numPoints)  + heading;

            % Perform ray casting
            [x, y] = bug.rayCast(robot, theta, visionRadius_var);
            
            % Expand the cone uniformly in all directions
            expandedX = x + expansionDistance * cos(theta);
            expandedY = y + expansionDistance * sin(theta);
            
            % Include the robot position in the cone vertices
            coneVertices = [robot(1), expandedX; robot(2), expandedY];
            
            % Create a closed polygon by adding the robot position at the end
            closedX = [robot(1), expandedX, robot(1)];
            closedY = [robot(2), expandedY, robot(2)];
            
            % Plot the expanded vision cone
            handle = fill(closedX, closedY, 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
            
            % Check for visible signs
            bug.checkVisibleSigns(robot, coneVertices);
        end

        function [x, y] = rayCast(bug, start, angles, maxDist)
            x = zeros(1, length(angles));
            y = zeros(1, length(angles));
            
            for i = 1:length(angles)
                [x(i), y(i)] = bug.castSingleRay(start, angles(i), maxDist);
            end
        end

        function [endX, endY] = castSingleRay(bug, start, angle, maxDist)
            dx = cos(angle);
            dy = sin(angle);
            
            for dist = 1:maxDist
                checkX = round(start(1) + dist * dx);
                checkY = round(start(2) + dist * dy);
                
                % Check if the point is within the grid
                if checkX < 1 || checkY < 1 || checkX > size(bug.occgridnav, 2) || checkY > size(bug.occgridnav, 1)
                    break;
                end
                
                % Check if the point is occupied
                if bug.isoccupied([checkX; checkY])
                    break;
                end
            end
            
            endX = start(1) + (dist - 1) * dx;
            endY = start(2) + (dist - 1) * dy;
        end

        function checkVisibleSigns(bug, robot, conePoints)
            % Plot the polygon area in red (for debugging)
            hold on;
            h = fill([robot(1), conePoints(1,:)], [robot(2), conePoints(2,:)], 'r');
            set(h, 'FaceAlpha', 0.2); % Set transparency
            
            % Check for visible exit cells
            for i = 1:size(bug.exitCells, 1)
                if ~ismember(i, bug.spottedExitCells)
                    exitPos = bug.exitCells(i, :);
                    if inpolygon(exitPos(1), exitPos(2), [robot(1), conePoints(1,:)], [robot(2), conePoints(2,:)])                
                        bug.spottedExitCells = [bug.spottedExitCells, i];
                        bug.plotSingleExit(i);
                    end
                end
            end
        
            % Check for visible exit signs
            for i = 1:size(bug.signCells.location, 1)
                if ~ismember(i, bug.spottedSigns)
                    signPos = bug.signCells.location(i, :);
                    if inpolygon(signPos(1), signPos(2), [robot(1), conePoints(1,:)], [robot(2), conePoints(2,:)])
                        bug.spottedSigns = [bug.spottedSigns, i];
                        bug.spottedSignsPositions{end+1} = signPos;
                        bug.spottedSignsTypes{end+1} = bug.signCells.direction{i};
                        bug.plotSingleSign(i);
                    end
                end
            end
            
            % Remove the debug polygon after a short delay
            pause(0.01); % Adjust this value to control how long the polygon is visible
            delete(h);
        end

        function [newGoal, newGoalType] = checkVisiblePointsOfInterest(bug, robot)
            newGoal = [];
            newGoalType = '';
            visiblePoints = [];
            visibleTypes = {};
            coneVertices = bug.visionConeHandle.Vertices;
        
            % Check exit cells
            for i = 1:length(bug.spottedExitCells)
                exitIndex = bug.spottedExitCells(i);
                if inpolygon(bug.exitCells(exitIndex,1), bug.exitCells(exitIndex,2), ...
                             coneVertices(:,1), coneVertices(:,2))
                    visiblePoints = [visiblePoints; bug.exitCells(exitIndex,:)];
                    visibleTypes{end+1} = 'exit';
                end
            end
        
            % Check sign cells
            for i = 1:size(bug.signCells.location, 1)
                if inpolygon(bug.signCells.location(i,1), bug.signCells.location(i,2), ...
                             coneVertices(:,1), coneVertices(:,2))
                    visiblePoints = [visiblePoints; bug.signCells.location(i,:)];
                    visibleTypes{end+1} = 'sign';
                end
            end
        
            % If points of interest are visible, change the goal
            if ~isempty(visiblePoints)
                % First, check if any of the visible points are exits
                exitIndices = strcmp(visibleTypes, 'exit');
                if any(exitIndices)
                    % If there are exits, choose the closest one
                    exitPoints = visiblePoints(exitIndices, :);
                    exitDistances = sqrt(sum((exitPoints - robot').^2, 2));
                    [~, idx] = min(exitDistances);
                    newGoal = exitPoints(idx, :);
                    newGoalType = 'exit';
                else
                    % If no exits, choose the closest sign
                    distances = sqrt(sum((visiblePoints - robot').^2, 2));
                    [~, idx] = min(distances);
                    newGoal = visiblePoints(idx, :);
                    newGoalType = visibleTypes{idx};
                end
            end
        end    
    
        function plotSingleExit(bug, i)
            hold on;
            scatter(bug.exitCells(i, 1), bug.exitCells(i, 2), 15, 'r', 'filled');
        end
    
        function plotSingleSign(bug, i)
            color = bug.getColorByDirection(bug.signCells.direction(i));
            hold on;
            scatter(bug.signCells.location(i, 1), bug.signCells.location(i, 2), 15, 'filled', 'MarkerFaceColor', color, 'MarkerEdgeColor', color);
        end
    
        function color = getColorByDirection(bug, direction)
            switch direction
                case "left"
                    color = 'b';
                case "right"
                    color = "#EDB120";
                case "front"
                    color = 'c';
                case "back"
                    color = 'm';
                otherwise
                    color = 'k';
            end
        end

%% Subroutines Functions
        function perform360Rotation(bug, robot)
            % Perform a 360-degree rotation to scan for points of interest
            for angle = 0:2*pi/180:2*pi  % Rotate in 2-degree increments
                heading = angle;
                if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                    delete(bug.visionConeHandle);
                end
                [bug.visionConeHandle, coneVertices] = bug.plotVisionCone(robot, heading);
                drawnow;
                pause(0.01);
        
                % Update points of interest found during the scan
                bug.updateSpottedPointsOfInterest(robot, coneVertices);
            end
        end

        function updateSpottedPointsOfInterest(bug, robot, coneVertices)
            % Update spotted emergency exits and signs based on vision cone
            bug.checkVisibleSigns(robot, coneVertices);
        end

        function updateGoalBasedOnPointsOfInterest(bug, robot, fallbackGoal)
            % Update the robot's goal based on visible points of interest
            [closestPoint, pointType] = bug.findClosestPointOfInterest(robot);
            if strcmp(pointType, 'exit')
                bug.goal = closestPoint';
            elseif strcmp(pointType, 'sign')
                bug.goal = closestPoint';
            else
                % No point of interest found, set an arbitrary goal
                bug.setArbitraryGoal(robot);
            end
        end

        function setArbitraryGoal(bug, robot)
            % Set an arbitrary goal when no points of interest are found
            disp('No points of interest found. Setting arbitrary goal.');
            
            % Choose "forward" direction (positive y-axis)
            heading = pi/2; % -pi/2, pi, pi/2, 0
            
            % Set a temporary goal 20 units away in the chosen direction
            tempGoalDistance = 1000;
            tempGoal = robot + [tempGoalDistance * cos(heading); tempGoalDistance * sin(heading)];
            
            % Ensure the new goal is within the map boundaries
            tempGoal = max(tempGoal, [1; 1]);
            tempGoal = min(tempGoal, [size(bug.occgridnav, 2); size(bug.occgridnav, 1)]);
            
            bug.goal = tempGoal;
            bug.message('New arbitrary goal set: (%d,%d)', round(tempGoal(1)), round(tempGoal(2)));
            
            % Recompute m-line
            bug.mline = homline(robot(1), robot(2), bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            
            % Reset bug algorithm
            bug.step = 1;
            bug.j = 1;  % Reset hit point counter


            if size(robot, 1) == 2 && size(robot, 2) == 1
                % robot is a 2x1 vector
                bug.H = robot';
            elseif size(robot, 1) == 1 && size(robot, 2) == 2
                % robot is a 1x2 vector
                bug.H = robot;
            else
                % Handle unexpected dimensions
                error('Unexpected dimensions for robot vector');
            end

%             bug.H = robot';  % Reset hit points, starting with current position - robot'
            bug.edge = [];  % Clear the edge list
            bug.k = 1;  % Reset edge index
        end

        function [closestPoint, pointType] = findClosestPointOfInterest(bug, robot)
            closestPoint = [];
            pointType = '';
            minDistance = inf;
            currentPosition = robot';
            
            % First, check spotted exit cells (emergency exits)
            for i = 1:length(bug.spottedExitCells)
                exitIndex = bug.spottedExitCells(i);
                distance = norm(bug.exitCells(exitIndex,:) - currentPosition);
                if distance < minDistance && distance > 1e-6  % Ensure it's not the current position
                    minDistance = distance;
                    closestPoint = bug.exitCells(exitIndex,:);
                    pointType = 'exit';
                end
            end
        
            % If an emergency exit was found, return it immediately
            if ~isempty(closestPoint)
                return;
            end
        
            % If no emergency exit was found, then check spotted signs
            for i = 1:length(bug.spottedSigns)
                signIndex = bug.spottedSigns(i);
                % Check if this sign has not been visited
                if ~ismember(signIndex, bug.visitedSigns)
                    signPos = bug.signCells.location(signIndex, :);
                    distance = norm(signPos - currentPosition);
                    if distance < minDistance && distance > 1e-6  % Ensure it's not the current position
                        minDistance = distance;
                        closestPoint = signPos;
                        pointType = bug.signCells.direction{signIndex};
                    end
                elseif ismember(signIndex, bug.visitedSigns) && (robot(1) == bug.signCells.location(signIndex,1) && robot(2) == bug.signCells.location(signIndex,2))
                    % We're at a previously visited sign
                    % Continue in the direction of the last visited sign
                    disp('Visiting for a second time the same emergency sign. Continuing with previous emergency sign direction.')
                    if ~isempty(bug.visitedSigns)
                        lastVisitedSignIndex = bug.visitedSigns(end);
                        lastDirection = bug.signCells.direction{lastVisitedSignIndex};
                        angle = bug.getDirectionAngle(lastDirection);
                        
                        % Set a point in the direction of the last sign
                        distanceToMove = 1000; % You can adjust this value
                        newPoint = currentPosition + [cos(angle), sin(angle)] * distanceToMove;
                        
                        closestPoint = newPoint;
                        pointType = "continue_" + lastDirection;
                        return;
                    end
                end
            end
        
            if isempty(closestPoint)
                disp('No unvisited point of interest found');
            else
                disp(['Found point of interest: ' pointType ' at (' num2str(closestPoint(1)) ',' num2str(closestPoint(2)) ')']);
            end
        end

        function found = checkAndMoveToEmergencyExit(bug, robot)
            % Check if an emergency exit is visible and move towards it
            found = false;
            for i = 1:length(bug.spottedExitCells)
                exitPos = bug.exitCells(bug.spottedExitCells(i), :);
                if norm(robot' - exitPos) < 1e-6  % Close enough to move directly
                    bug.goal = exitPos';
                    found = true;
                    return;
                end
            end
        end

        function handleEmergencySign(bug, signIndex, robot)
            % Process actions upon reaching an emergency sign
            bug.visitedSigns = [bug.visitedSigns, signIndex]; % Mark sign as visited
            direction = bug.signCells.direction{signIndex}; % Get direction suggested by the sign
            disp("Following direction: " + direction);
            
            % Get the angle for the suggested direction
            heading = bug.getDirectionAngle(direction);
            
            % Move the robot in the suggested direction
            moveDistance = 1; % Move 1 unit in the suggested direction
            newRobotPos = robot + [moveDistance * cos(heading); moveDistance * sin(heading)];
            
            % Ensure the new position is within the map boundaries
            newRobotPos = max(newRobotPos, [1; 1]);
            newRobotPos = min(newRobotPos, [size(bug.occgridnav, 2); size(bug.occgridnav, 1)]);
            
            % Update the robot's position
            bug.start = newRobotPos;
            
            % Rotate and look for new points of interest in the sign's suggested direction
            bug.rotateTowardsDirection(newRobotPos, heading);
            
            % Check for visible emergency exits first
            visibleExits = bug.checkVisibleEmergencyExits(newRobotPos);
            if ~isempty(visibleExits)
                % If there are visible exits, choose the closest one
                exitDistances = sqrt(sum((visibleExits - newRobotPos').^2, 2));
                [~, idx] = min(exitDistances);
                closestExit = visibleExits(idx, :);
                bug.goal = closestExit';
                disp("Emergency exit found! Updated goal to exit at (" + num2str(closestExit(1)) + "," + num2str(closestExit(2)) + ")");
            else
                % If no emergency exits are visible, find the next closest point of interest
                [nextGoal, pointType] = bug.findClosestPointOfInterest(newRobotPos);
                if ~isempty(nextGoal)
                    bug.goal = nextGoal'; % Update goal to the next point of interest
                    disp("Updated goal to: " + pointType + " at (" + num2str(nextGoal(1)) + "," + num2str(nextGoal(2)) + ")");
                else
                    % If no point of interest is visible, set a temporary goal in the suggested direction
                    tempGoalDistance = 1000; % Set a temporary goal 50 units away in the suggested direction
                    bug.goal = newRobotPos + [tempGoalDistance * cos(heading); tempGoalDistance * sin(heading)];
                    disp("No points of interest visible, setting temporary goal in direction: " + num2str(heading));
                end
            end
            
            % Recompute m-line
            bug.mline = homline(newRobotPos(1), newRobotPos(2), bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            
            % Reset bug algorithm
            bug.step = 1;
            bug.j = 1;  % Reset hit point counter
            bug.H = newRobotPos';  % Reset hit points, starting with new position
            bug.edge = [];  % Clear the edge list
            bug.k = 1;  % Reset edge index
        end

        function visibleExits = checkVisibleEmergencyExits(bug, robot)
            visibleExits = [];
            if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                coneVertices = bug.visionConeHandle.Vertices;
                for i = 1:length(bug.spottedExitCells)
                    exitIndex = bug.spottedExitCells(i);
                    if inpolygon(bug.exitCells(exitIndex,1), bug.exitCells(exitIndex,2), ...
                                 coneVertices(:,1), coneVertices(:,2))
                        visibleExits = [visibleExits; bug.exitCells(exitIndex,:)];
                    end
                end
            end
        end
        
        function angle = getDirectionAngle(~, direction)
            % Map direction strings to angles (assumes robot's initial heading is along +x axis)
            switch lower(direction)
                case "left"
                    angle = pi;
                case "right"
                    angle = 0;
                case "front"
                    angle = pi/2;
                case "back"
                    angle = -pi/2;
                otherwise
                    error("Unknown direction: " + direction);
            end
        end
        
        function rotateTowardsDirection(bug, robot, heading)
            % Rotate and look for points of interest in the given direction
            if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                delete(bug.visionConeHandle);
            end
            [bug.visionConeHandle, coneVertices] = bug.plotVisionCone(robot, heading);
            drawnow;
            
            % Update points of interest based on the vision cone
            bug.updateSpottedPointsOfInterest(robot, coneVertices);
        end
        
        function signIndex = findSignAtPosition(bug, position)
            % Check if the current position matches any emergency sign location
            signIndex = find(all(abs(bug.signCells.location - position') < 1e-6, 2), 1);
        end

        function clearPath = checkClearPath(bug, start, goal)
            % Check if there's a clear path between start and goal
            direction = goal - start;
            distance = norm(direction);
            unitDirection = direction / distance;
            
            for i = 1:ceil(distance)
                checkPoint = start + i * unitDirection;
                if bug.isoccupied(round(checkPoint))
                    clearPath = false;
                    return;
                end
            end
            clearPath = true;
        end

%% Next Function function
        function [n_est, n_true, goalReached, exitStairsNumber] = next(bug, robot, iter, startHeight, goalHeight)
            %% NEXT.STEP.INIT
            % Implement the main state machine for Bug3
            n = [];
            goalReached = false;
            exitStairsNumber = 0;
            robot = robot(:);   % These are coordinates (x, y)

            % Check if we're moving towards an emergency exit
            movingTowardsExit = ~isempty(bug.spottedExitCells) && isequal(bug.goal, bug.exitCells(bug.spottedExitCells(1), :)');

            % Check if vision cone handle exists and is valid
            if isempty(bug.visionConeHandle) || ~isvalid(bug.visionConeHandle)
                % If not, create an initial vision cone
                heading = atan2(bug.goal(2) - robot(2), bug.goal(1) - robot(1));
                bug.visionConeHandle = bug.plotVisionCone(robot, heading);
            end
        
            % Check if current position matches any stop cells (emergency exits)
            if ismember(robot', bug.exitCells, 'rows') && iter > 5
                exitStairsNumber = find(all(abs(bug.exitCells - robot') < 1e-6, 2));
                bug.message('Stopped at predefined exit cell: (%d,%d)', robot);
                n_est = []; % Stop the algorithm
                n_true = robot;
                goalReached = false;
                return;
            end

            % In the next function, replace the existing emergency sign check with:
            signIndex = find(all(abs(bug.signCells.location - robot') < 1e-6, 2), 1);
            if ~isempty(signIndex)
                if ismember(signIndex, bug.visitedSigns) && (robot(1) == bug.signCells.location(signIndex,1) && robot(2) == bug.signCells.location(signIndex,2))
                    % Reset bug algorithm
%                     n = robot; % Set next position to the updated start position
                    disp('Here!');

                elseif ~ismember(signIndex, bug.visitedSigns)
                    disp("Stopped at predefined emergency sign cell: X="+ num2str(robot(1)) + ", Y=" + num2str(robot(2)));
                    bug.handleEmergencySign(signIndex, robot);
                    n_est = bug.start; % Set next position to the updated start position
                    n_true = robot;
                    goalReached = false;
                    return;
                end
            end
            
            % Check for visible points of interest and update goal if necessary
            [newGoal, newGoalType] = bug.checkVisiblePointsOfInterest(robot);
            visitedSignsLocation = bug.signCells.location(bug.visitedSigns',:);
            if ~isempty(newGoal) && ~ismember(newGoal, visitedSignsLocation,"rows")
                if strcmp(newGoalType, 'exit')
                    bug.goal = newGoal';
                    bug.message('New goal set: Emergency exit at (%d,%d)', newGoal(1), newGoal(2));
                elseif isempty(bug.spottedExitCells)  % Only change goal to a sign if no exit has been spotted
                    bug.goal = newGoal';
                    bug.message('New goal set: Emergency sign at (%d,%d)', newGoal(1), newGoal(2));
                end
                
                % Recompute m-line
                bug.mline = homline(robot(1), robot(2), bug.goal(1), bug.goal(2));
                bug.mline = bug.mline / norm(bug.mline(1:2));
                
                % Reset bug algorithm
                bug.step = 1;
                bug.j = 1;  % Reset hit point counter
                bug.H = robot';  % Reset hit points, starting with current position
                bug.edge = [];  % Clear the edge list
                bug.k = 1;  % Reset edge index
            end

            % Update the goal line
            if isempty(bug.goalLineHandle) || ~isvalid(bug.goalLineHandle)
                bug.goalLineHandle = line([robot(1), bug.goal(1)], [robot(2), bug.goal(2)], 'Color', 'r', 'LineStyle', '--');
            else
                set(bug.goalLineHandle, 'XData', [robot(1), bug.goal(1)], 'YData', [robot(2), bug.goal(2)]);
            end

            %% NEXT.STEP.1:
            if bug.step == 1
                disp('Step 1: Moving towards goal');
                % Step 1. Move along the M-line toward the goal
                if colnorm(bug.goal - robot) == 0
                    goalReached = true;
                    disp('Goal reached on the current floor.');
                    return
                end


                % Motion on the line toward goal
                d = bug.goal - robot;
                
                % Calculate the unit vector in the direction of the goal
                direction = d / norm(d);
                
                % Scale the movement to get a step size of 1
                step = direction * 1;
                
                % Round the step to get integer coordinates
                dx = round(step(1));
                dy = round(step(2));
                
                % Ensure we always move at least one unit in some direction
                if dx == 0 && dy == 0
                    if abs(direction(1)) > abs(direction(2))
                        dx = sign(direction(1));
                    else
                        dy = sign(direction(2));
                    end
                end


                %----------------------------------------------------------
                % Case we lose line of sight with the goal being the goal
                % one of the points of the points of interest data points,
                % omit the case of the goal being an arbitrary direction.
                % Check if there's a clear path to the goal
                % Check if the goal is a point of interest (exit or sign)
                isGoalPointOfInterest = any(all(abs(bug.exitCells - bug.goal') < 1e-6, 2)) || ...
                                        any(all(abs(bug.signCells.location - bug.goal') < 1e-6, 2));
                if isGoalPointOfInterest && ((dx == 0 && dy ~= 0) || (dx ~= 0 && dy == 0))
                    clearPath = bug.checkClearPath(robot, bug.goal);
                
                    if clearPath
                        % If there's a clear path, move directly towards the goal
                        n = robot + [dx; dy];
                    else
                        % Reculate the robot to again have a clear path.
                        if dx > 0 && dy == 0
                            previous_dx = dx;
                            previous_dy = dy;
                            n = robot + [-dx; 0];
                        elseif dx < 0 && dy == 0
                            previous_dx = dx;
                            previous_dy = dy;
                            n = robot + [abs(dx); 0];
                        elseif dx == 0 && dy > 0
                            previous_dx = dx;
                            previous_dy = dy;
                            n = robot + [0; -dy];
                        elseif dx == 0 && dy < 0
                            previous_dx = dx;
                            previous_dy = dy;
                            n = robot + [0; abs(dy)];
                        end

                        % Now try a new route.
                        if d(1) > 0 && d(2) > 0 % 1st Quadrant
                            if previous_dx > 0 || previous_dx < 0
                                dx = 0;
                                dy = 1;
                                n = robot + [dx; dy];
                            end
                        elseif d(1) < 0 && d(2) < 0 % 3rd Quadrant
                            if previous_dx > 0 || previous_dx < 0
                                dx = 0;
                                dy = -1;
                                n = robot + [dx; dy];
                            end
                        elseif d(1) > 0 && d(2) < 0 % 4th Quadrant
                            if previous_dy > 0 || previous_dy < 0
                                dx = 1;
                                dy = 0;
                                n = robot + [dx; dy];
                            end
                        elseif d(1) < 0 && d(2) > 0 % 2nd Quadrant
                            if previous_dy > 0 || previous_dy < 0
                                dx = -1;
                                dy = 0;
                                n = robot + [dx; dy];
                            end
                        end
                    end
                end


                %----------------------------------------------------------


                % Detect if next step is an obstacle
                if bug.isoccupied(robot + [dx; dy])
                    disp('Obstacle detected! Transitioning to Step 2 (obstacle avoidance)');
                    bug.message('(%d,%d) obstacle!', n);
                    bug.H(bug.j, :) = robot; % Define hit point
                    bug.step = 2;
                    % Get a list of all the points around the obstacle
                    bug.edge = edgelist(bug.occgridnav == 0, robot);
                    bug.k = 2;  % Skip the first edge point, we are already there

                    bug.isWallFollowing = true;
                    n = robot; % Stay in place for this step
                else
                    n_true = robot + [dx; dy];
                    [n_est, bug.cumulativeDistance, bug.driftState, sigma_par_inc, sigma_perp_inc, sigpar2_curr, sigperp2_curr, t_curr] = bug.applyDrift(robot, n_true, bug.cumulativeDistance, bug.driftState);

                    bug.driftLog_time_s      = [bug.driftLog_time_s,      t_curr];
                    bug.driftLog_dist_m      = [bug.driftLog_dist_m,      bug.cumulativeDistance];
                    bug.driftLog_posErr_m    = [bug.driftLog_posErr_m,    bug.driftState.pos_err_m];
                    bug.driftLog_yawErr_rad  = [bug.driftLog_yawErr_rad,  bug.driftState.yaw_err_rad];
                    bug.driftLog_sigpar_m    = [bug.driftLog_sigpar_m,    sigma_par_inc];
                    bug.driftLog_sigperp_m   = [bug.driftLog_sigperp_m,   sigma_perp_inc];
                    bug.driftLog_sigparCum_m = [bug.driftLog_sigparCum_m, sqrt(sigpar2_curr)];
                    bug.driftLog_sigperpCum_m=[bug.driftLog_sigperpCum_m, sqrt(sigperp2_curr)];

                    % Plot the vision cone after moving
                    delta = n_true - robot; % n_est - robot
                    heading = atan2(delta(2), delta(1));
    
                    % Delete previous vision cone if it exists
                    if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                        delete(bug.visionConeHandle);
                    end
    
                    % Plot the new vision cone
                    bug.visionConeHandle = bug.plotVisionCone(n_true, heading); % n_est, heading
                end   
            end % Step 1

            %% NEXT.STEP.2:
            if bug.step == 2
                disp('Step 2: Obstacle avoidance');

                bug.isWallFollowing = true;

                % Step 2. Move around the obstacle until we reach a point on the M-line closer than when we started.
                if colnorm(bug.goal - robot) == 0 % Are we there yet?
                    return
                end

                if bug.k <= numcols(bug.edge)
                    n_true = bug.edge(:, bug.k);

                    [n_est, bug.cumulativeDistance, bug.driftState, sigma_par_inc, sigma_perp_inc, sigpar2_curr, sigperp2_curr, t_curr] = bug.applyDrift(robot, n_true, bug.cumulativeDistance, bug.driftState);

                    bug.driftLog_time_s      = [bug.driftLog_time_s,      t_curr];
                    bug.driftLog_dist_m      = [bug.driftLog_dist_m,      bug.cumulativeDistance];
                    bug.driftLog_posErr_m    = [bug.driftLog_posErr_m,    bug.driftState.pos_err_m];
                    bug.driftLog_yawErr_rad  = [bug.driftLog_yawErr_rad,  bug.driftState.yaw_err_rad];
                    bug.driftLog_sigpar_m    = [bug.driftLog_sigpar_m,    sigma_par_inc];
                    bug.driftLog_sigperp_m   = [bug.driftLog_sigperp_m,   sigma_perp_inc];
                    bug.driftLog_sigparCum_m = [bug.driftLog_sigparCum_m, sqrt(sigpar2_curr)];
                    bug.driftLog_sigperpCum_m=[bug.driftLog_sigperpCum_m, sqrt(sigperp2_curr)];

                    if ~movingTowardsExit  % Check for visible points of interest while wall-following
                        [newGoal, newGoalType] = bug.checkVisiblePointsOfInterest(n_est);
                        if ~isempty(newGoal)
                            % If a point of interest is found, update the goal
                            bug.updateGoalBasedOnPointsOfInterest(n_est, bug.originalGoal);
                            bug.step = 1;  % Go back to step 1 to move towards the new goal
                            return;
                        end
                    end

                    % Plot the vision cone after moving
                    delta = n_true - robot; % n_est - robot
                    heading = atan2(delta(2), delta(1));
    
                    % Delete previous vision cone if it exists
                    if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                        delete(bug.visionConeHandle);
                    end
    
                    % Plot the new vision cone
                    bug.visionConeHandle = bug.plotVisionCone(n_true, heading); % n_est, heading

                else
                    % We are at the end of the list of edge points, we are back where we started. Step 2.c test.
                    disp('Robot is trapped');
                    error('RTB:bug2:noplan', 'Robot is trapped')    
                    return;
                end
    
                % Are we on the M-line now?
                if abs([robot' 1] * bug.mline') <= 0.5
                    bug.message('(%d,%d) moving along the M-line', n_est);
                    % Are we closer than when we encountered the obstacle?
                    if colnorm(robot - bug.goal) < colnorm(bug.H(bug.j, :)' - bug.goal)
                        disp('Back to moving along the M-line (Step 1)');
                        bug.j = bug.j + 1;
                        bug.step = 1;
                        return;
                    end
                end
                % No, keep going around
                bug.message('(%d,%d) keep moving around obstacle', n_est)
                bug.k = bug.k + 1;
            end % Step 2

        end % next

        function plan(bug)
            error('RTB:Bug2:badcall', 'This class has no plan method');
        end    
    end

    methods (Static)
        function [P44_11, sigma_gN, sigma_v] = computeP44(ARW, RRW, VRW, r, ac, fs, t_stride, t_stance, g)
            % computeP44  Implements Eq. (44) → Eq. (48) from
            % “Study on Estimation Errors in ZUPT-Aided Pedestrian Inertial Navigation
            %  Due to IMU Noises.”
            %
            % Inputs (SI):
            %   ARW      angle random walk        [rad/sqrt(s)]
            %   RRW      rate random walk         [rad/s/sqrt(s)]
            %   VRW      velocity random walk     [m/s/sqrt(s)]
            %   r        ZUPT velocity-meas. STD  [m/s]
            %   ac       gait coupling coeff      [-]  (paper ~0.84)
            %   fs       IMU sampling rate        [Hz]
            %   t_stride stride period            [s]
            %   t_stance stance duration          [s]
            %   g        gravity (9.80665)        [m/s^2]
            %
            % Outputs:
            %   P44_11   covariance term P44(1,1)  (gyroscope bias along North)
            %   sigma_gN = sqrt(P44_11)
            %   sigma_v  velocity STD from Eq. (44)
            %
            % References:
            %   - Quartic:  a x^4 + b x^2 + c x + d = 0  (paper Eq. 44)
            %   - Coeffs:   paper Eq. (44)–(46)  :contentReference[oaicite:2]{index=2}
            %   - P44:      paper Eq. (48)       :contentReference[oaicite:3]{index=3}

            % ---------------------------------------------------------------------
            % 1) Build quartic coefficients (paper lines 60–90)  :contentReference[oaicite:4]{index=4}
            % ---------------------------------------------------------------------
            num = fs * t_stance;
            den = 2 * g * r^2 * t_stride;
            
            a = (num / den)^2;
            b = -(fs * t_stance * VRW^2) / (2 * g^2 * r^2 * t_stride);
            c = -2 * ac * RRW / g * sqrt((r^2 * t_stride) / (fs * t_stance));
            d = (VRW^4) / (4 * g^2) - (ARW^2 * r^2 * t_stride) / (fs * t_stance);

            % ---------------------------------------------------------------------
            % 2) Solve the quartic for P22(1,1)
            % ---------------------------------------------------------------------
            % polynomial is: a x^4 + b x^2 + c x + d = 0
            % create as 5th→0th degree vector: [a 0 b c d]
            coeffs = [a 0 b c d];
            roots_all = roots(coeffs);
            
            % we only want real, positive root (covariance must be >= 0)
            roots_real = roots_all(abs(imag(roots_all)) < 1e-8);
            roots_pos = roots_real(real(roots_real) > 0);
            
            if isempty(roots_pos)
                error('No positive real root for P22(1,1)');
            end
            
            % often there will be one physically meaningful small positive root
            P22_11 = min(real(roots_pos));
            % the paper says: “The velocity uncertainty is simply σ_v = sqrt(P22(2,2)).”
            % Here we approximate P22(2,2) ≈ P22(1,1) (same order, symmetric)
            sigma_v = sqrt(P22_11);
            
            % ---------------------------------------------------------------------
            % 3) Compute P42(1,2) from Eq. (40) – we need the sqrt(...) factor
            %    but Eq. (48) already gives the final closed form, so we can
            %    directly use Eq. (48)
            % ---------------------------------------------------------------------
            factor_sqrt = sqrt((fs * t_stance) / (r^2 * t_stride));
        
            P44_11 = (RRW * ARW / ac)^2 + ...
                          (2 * sigma_v^2 * RRW^3 / (ac * g)) * factor_sqrt;
            
            % variance of P44_11
            sigma_gN = sqrt(P44_11); 
        end
    end
end