%% Caption
% BUG2_DRIFT_MODEL_ES  Bug2 navigation class with ZUPT-aided INS drift model (E. Sangenis)
%
% This class is derived from Peter Corke’s Bug2 implementation (RTB) and extends the
% standard Bug2 reactive navigation automaton by injecting an IMU-based, ZUPT-aided
% inertial navigation drift model. In addition to generating the nominal Bug2 path,
% the algorithm maintains:
%   (i)  a ground-truth robot trajectory (no drift),
%   (ii) an estimated/drifted trajectory (INS-like),
%   (iii) a drifted (perceived) goal and corresponding adjusted m-line, and
%   (iv) drift-state propagation driven by IMU noise parameters (e.g., ARW/RRW/VRW)
%        with distance-based time discretization.
%
% The drift model updates a cumulative drift state (position error + yaw error),
% supports inheritance of an initial drift offset/state from a previous phase, and
% logs drift/covariance-growth terms for post-run visualization.
%
% Methods::
%   Bug2_drift_model_ES  Constructor (supports initialDrift / initialDriftState / initialDistance)
%   query                Find a path from start to goal (returns true + drifted paths)
%   next                 Bug2 state machine step with drift injection
%   applyDrift            ZUPT-aided INS drift propagation per step (IMU-parameter dependent)
%   plot                 Display the obstacle map
%   display              Display state/parameters in human readable form
%   char                 Convert to string
%
% Example::
%         load map1
%         bug = Bug2_drift_model_ES(map, 'initialDrift', [0;0]);
%         start = [20,10];
%         goal  = [50,35];
%         [pp_est, pp_true] = bug.query(start, goal, 'animate');
%
% Reference (Bug2)::
% - Dynamic path planning for a mobile automaton with limited information on the environment,
%   V. Lumelsky and A. Stepanov,
%   IEEE Transactions on Automatic Control, vol. 31, pp. 1058–1063, Nov. 1986.
% - Robotics, Vision & Control, Sec 5.1.2,
%   Peter Corke, Springer, 2011.
%
% See also Navigation, DXform, Dstar, PRM.
%
% Original code Copyright (C) 1993-2017, by Peter I. Corke
% Modifications Copyright (C) 2026, Eudald Sangenis (drift model integration and logging)
%
% This file remains under the terms of the GNU Lesser General Public License (LGPL),
% consistent with the original Robotics Toolbox for MATLAB (RTB) distribution.
%
% http://www.petercorke.com

%% Class
classdef Bug2_drift_model_ES < Navigation

    properties(Access=protected)
        H       % hit points
        j       % number of hit points
        mline   % line from starting position to goal
        adjusted_mline  %  m-line adjusted for drift
        step    % state, in step 1 or step 2 of algorithm
        edge    % edge list
        k       % edge index
        visionConeHandle % Handle for the vision cone plot

        goal_true      % the real/ground-truth goal in cells
        goal_est       % the perceived (drifted) goal in cells

        cumulativeDistance  % Total distance traveled
        driftState          % struct: yaw_err_rad, pos_err_m, acc_step_m

        % Degub Drift Parameters:
        driftLog_time_s      % 1×N  equivalent time
        driftLog_dist_m      % 1×N  cumulative distance
        driftLog_posErr_m    % 2×N  [ex; ey] estimated pos error in meters
        driftLog_yawErr_rad  % 1×N  yaw error
        driftLog_sigpar_m    % 1×N  incremental parallel STD (model)
        driftLog_sigperp_m   % 1×N  incremental perpendicular STD (model)
        driftLog_sigparCum_m
        driftLog_sigperpCum_m
        initialDriftOffset  % Initial Drift offset heritage from previous algorithms
    end

    methods
        function bug = Bug2_drift_model_ES(varargin)
            %Bug2.Bug2 Construct a Bug2 navigation object 
            %
            % B = Bug2(MAP, OPTIONS) is a bug2 navigation object, and MAP is an occupancy grid,
            % a representation of a planar world as a matrix whose elements are 0 (free
            % space) or 1 (occupied).
            %
            % Options::
            % 'goal',G      Specify the goal point (1x2)
            % 'inflate',K   Inflate all obstacles by K cells.
            %
            % See also Navigation.Navigation.


            initialDrift = [0; 0];  % Default value
            initialDriftState = struct('yaw_err_rad', 0, 'pos_err_m', [0;0], 'acc_step_m', 0);
            initialDistance = 0;
            superArgs = varargin;
            
            % Extract initialDrift
            idx = find(strcmpi(superArgs, 'initialDrift'));
            if ~isempty(idx) && idx < length(superArgs)
                initialDrift = superArgs{idx+1};
                initialDrift = initialDrift(:);
                superArgs([idx, idx+1]) = [];
            end

            % Extract initialDriftState
            idx = find(strcmpi(superArgs, 'initialDriftState'));
            if ~isempty(idx) && idx < length(superArgs)
                initialDriftState = superArgs{idx+1};
                superArgs([idx, idx+1]) = [];
            end
            
            % Extract initialDistance
            idx = find(strcmpi(superArgs, 'initialDistance'));
            if ~isempty(idx) && idx < length(superArgs)
                initialDistance = superArgs{idx+1};
                superArgs([idx, idx+1]) = [];
            end


            bug = bug@Navigation(superArgs{:});
            bug.H = [];
            bug.j = 1;
            bug.step = 1;
            bug.visionConeHandle = []; % Initialize vision cone handle

            % Store initial drift offset
            bug.initialDriftOffset = initialDrift;
            % Initialize with cumulative drift state
            bug.cumulativeDistance = initialDistance;
            bug.driftState = initialDriftState;
            bug.driftState.pos_err_m = initialDrift / 4;  % Convert cells to meters

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

        function [pp, pp_true, goalReached_est, goalReached_true] = query(bug, start, goal, varargin)
            %Bug2.query  Find a path
            %
            % B.query(START, GOAL, OPTIONS) is the path (Nx2) from START (1x2) to GOAL
            % (1x2).  Row are the coordinates of successive points along the path.  If
            % either START or GOAL is [] the grid map is displayed and the user is
            % prompted to select a point by clicking on the plot.
            %
            % Options::
            %  'animate'   show a simulation of the robot moving along the path
            %  'movie',M   create a movie
            %  'current'   show the current position position as a black circle
            %
            % Notes::
            % - START and GOAL are given as X,Y coordinates in the grid map, not as
            %   MATLAB row and column coordinates.
            % - START and GOAL are tested to ensure they lie in free space.
            % - The Bug2 algorithm is completely reactive so there is no planning
            %   method.
            % - If the bug does a lot of back tracking it's hard to see the current
            %   position, use the 'current' option.
            % - For the movie option if M contains an extension a movie file with that
            %   extension is created.  Otherwise a folder will be created containing
            %   individual frames.
            %
            % See also Animate.
         
            opt.animate = false;
            opt.movie = [];
            drift_line_handle = [];
            opt.current = false;
            goalReached_true = false;
            goalReached_est  = false;
            opt = tb_optparse(opt, varargin);
            
            if ~isempty(opt.movie)
                anim = Animate(opt.movie);
                opt.animate = true;
            end
       
            % make sure start and goal are set and valid
            bug.start = []; bug.goal_true = [];
            bug.checkquery(start, goal);

            bug.goal_true = bug.goal(:); % keep the real goal
            bug.goal_est  = bug.goal_true + bug.initialDriftOffset(:); % Goal drifted by the previous exploration algorithm drift position
            % clamp goal_est to map if you want
            bug.goal_est(1) = max(1, min(bug.goal_est(1), size(bug.occgridnav,2)));
            bug.goal_est(2) = max(1, min(bug.goal_est(2), size(bug.occgridnav,1)));

            % compute the m-line no drifted goal
            bug.mline = homline(bug.start(1), bug.start(2), ...
                bug.goal_true(1), bug.goal_true(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));

            % compute the m-line using drifted goal due to previous drift
            % accumulated from previous algorithms
            bug.adjusted_mline = homline(bug.start(1), bug.start(2), ...
                bug.goal_est(1), bug.goal_est(2));
            bug.adjusted_mline = bug.adjusted_mline / norm(bug.adjusted_mline(1:2));
            
            if opt.animate
                bug.plot();
                bug.plot_mline(); hold on;
                bug.plot_adjusted_mline();

                plot(goal(1), goal(2), 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'DisplayName', 'True Goal');
                plot(bug.goal_est(1), bug.goal_est(2), 'mp', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'DisplayName', 'Adjusted Goal');
            end
            
            % iterate using the next() method until we reach the goal
            robot = bug.start(:) + bug.initialDriftOffset; % Drift Position
            robot_true = bug.start(:); % True Position
            bug.step = 1;
            path = bug.start(:);
            path_true = bug.start(:);

            while true
                if opt.animate
                    % Plot ESTIMATED position (green - what algorithm "sees")
                    plot(robot(1), robot(2), 'g.', 'MarkerSize', 8, 'DisplayName', 'Estimated Path');
                    
                    % Plot TRUE position (blue - ground truth)
                    plot(robot_true(1), robot_true(2), 'b.', 'MarkerSize', 8, 'DisplayName', 'True Path');

                    if ~isempty(drift_line_handle) && isvalid(drift_line_handle)
                        delete(drift_line_handle);
                    end
                    % Line connecting true and estimated positions to show drift
                    if norm(robot - robot_true) > 0.5  % Only draw if drift is significant
                        drift_line_handle = plot([robot(1), robot_true(1)], [robot(2), robot_true(2)], 'r--', 'LineWidth', 1);
                    end


                    if opt.current
                        h = plot(robot(1), robot(2), 'ko', 'MarkerSize', 8);
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
                [robot, goalReached_est, goalReached_true, robot_true] = bug.next(robot, robot_true);

                % are we there yet?
                if isempty(robot)
                    % yes, exit the loop
                    break
                else
                    % no, append it to the path
                    path = [path robot(:)];
                    path_true = [path_true robot_true(:)];
                end
            end
            
            if ~isempty(opt.movie)
                anim.close();
            end

            % --- plot drift / covariance growth ---
            figure(5); 
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
                pp = path'; % Estimated path (with drift)
            end

            if nargout > 1
                pp_true = path_true'; % True path (ground truth)
            end
        end

        function [drifted_pos, cumulative_dist, drift_state, sigma_par_inc, sigma_perp_inc, sigpar2_curr, sigperp2_curr, t_curr] = applyDrift( ...
            bug, prev_true, next_true, cumulative_dist, drift_state)
            
            % APPLYDRIFT  ZUPT-aided INS drift model (distance–based incremental form)
            %
            % Implements the ZUPT-aided pedestrian INS error-growth model from:
            %   "Study on Estimation Errors in ZUPT-Aided Pedestrian Inertial
            %    Navigation Due to IMU Noises."
            %
            % This version mirrors applyDriftStandalone:
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
            VRW = VRW_mg_sqrtHz * g / 1000;                     % [(m/s)/√Hz]
        
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
            target_right_drift_m = 1.5 * (0.35*scale_arw + 0.65*scale_vrw);   % ~0.5 m for tactical
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

%         function state = getDriftState(bug)
%             % Get the current drift state
%             state = bug.driftState;
%         end
        
%         function dist = getCumulativeDistance(bug)
%             % Get the cumulative distance traveled
%             dist = bug.cumulativeDistance;
%         end
            
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
                axis equal
            end
        end

        function plot_adjusted_mline(bug, ls)
            if nargin < 2
                ls = 'm--';
            end
            dims = axis;
            xmin = dims(1); xmax = dims(2);
            ymin = dims(3); ymax = dims(4);
            
            hold on
            if bug.adjusted_mline(2) == 0
                % handle the case that the line is vertical
                plot([bug.start(1) bug.start(1)], [ymin ymax], 'm--');
            else
                x = [xmin xmax]';
                y = -[x [1;1]] * [bug.adjusted_mline(1); bug.adjusted_mline(3)] / bug.adjusted_mline(2);
                plot(x, y, ls);
                axis equal
            end
        end

%         function handle = plotVisionCone(bug, robot, heading)
%             visionAngle = 45*pi/180;
%             visionRadius = 30*4; % 30 meters (4 conversion pxl to meters)
%             % Plot the vision cone based on the robot's position, angle, and radius
%             theta = linspace(-visionAngle/2, visionAngle/2, 30)  + heading;
%             [x, y] = bug.rayCast(robot, theta, visionRadius);
%             handle = fill([robot(1), x], [robot(2), y], 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none'); % Return handle
%         end

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

        function [n, goalReached_est, goalReached_true, n_true] = next(bug, robot, robot_true)
            % Inputs:
            %   robot: Estimated (drifted) position [x; y]
            %   robot_true: True position [x; y] (for motion planning and goal checking)
            % Outputs:
            %   n: Next estimated position
            %   goalReached: Boolean flag
            %   n_true: Next true position
            
            n = [];
            n_true = [];
            goalReached_true = false;
            goalReached_est  = false;
            GOAL_TOL_M       = 1.0;    % [m] tolerance radius for estimated trajectory
            CELLS_PER_METER  = 4;
            goal_tol_cells   = GOAL_TOL_M * CELLS_PER_METER;
            robot = robot(:);
            robot_true = robot_true(:);
            
            % Step 1. Move along the M-line toward the goal
            if bug.step == 1
                
                % --- 1) TRUE trajectory check ---
                % If True Robot Reaches → Adjusted Goal
                if norm(bug.goal_est(:) - robot_true(:)) <= 1
                    % If True Robot Reaches → True Goal
                    if norm(bug.goal_true(:) - robot_true(:)) <= goal_tol_cells
                        goalReached_true = true;
                        disp('TRUE trajectory → True Goal reached.');
                    
                        % --- 2) ESTIMATED trajectory evaluation ---
                        % If True Robot Reaches → True Goal &&
                        % If Drifted Robot Reaches → Adjusted Goal
                        if norm(bug.goal_est(:) - robot(:)) <= goal_tol_cells
                            goalReached_est = true;
                            disp('ESTIMATED trajectory → Adjusted Goal reached.');

                            % If True Robot Reaches → True Goal &&
                            % If Drifted Robot Reaches → Adjusted Goal &&
                            % If Drifted Robot Reaches → True Goal
                            if norm(bug.goal_true(:) - robot(:)) <= goal_tol_cells
                                disp('ESTIMATED trajectory → True Goal Reached.');
                            % If Drifted Robot NOT Reaches → True Goal
                            else
                                disp('ESTIMATED trajectory → True Goal NOT Reached (> 1 m away).');
                            end
                        % If Drifted Robot NOT Reaches → Adjusted Goal
                        else
                            goalReached_est = false;
                            disp('ESTIMATED trajectory → Adjusted Goal NOT reached (> 1 m away).');
                        end
                    
                        return

                    % If True Robot NOT Reaches → True Goal
                    else
                        goalReached_true = false;
                        disp('TRUE trajectory → True Goal NOT reached (> 1 m away).');

                        % --- 2) ESTIMATED trajectory evaluation ---
                        % If True Robot NOT Reaches → True Goal &&
                        % If Drifted Robot Reaches → Adjusted Goal
                        if norm(bug.goal_est(:) - robot(:)) <= goal_tol_cells
                            goalReached_est = true;
                            disp('ESTIMATED trajectory → Adjusted Goal reached.');

                            % If True Robot NOT Reaches → True Goal &&
                            % If Drifted Robot Reaches → Adjusted Goal &&
                            % If Drifted Robot Reaches → True Goal
                            if norm(bug.goal_true(:) - robot(:)) <= goal_tol_cells
                                disp('ESTIMATED trajectory → True Goal Reached.');
                            % If Drifted Robot NOT Reaches → True Goal
                            else
                                disp('ESTIMATED trajectory → True Goal NOT Reached (> 1 m away).');
                            end
                        else
                            goalReached_est = false;
                            disp('ESTIMATED trajectory → Adjusted Goal NOT reached (> 1 m away).');
                        end

                        return

                    end 
                end 

                d = bug.goal_est - robot_true;    % Changed from: d = bug.goal_true - robot;
        
                if abs(d(1)) > abs(d(2))
                    dx = sign(d(1));
                    L = bug.adjusted_mline;
                    if L(2) >= 1e-5 || L(2) <= -1e-5
                        y = -((robot_true(1) + dx) * L(1) + L(3)) / L(2);
                        dy = round(y - robot_true(2));
                    else
                        dy = sign(d(2));
                    end
                else
                    dy = sign(d(2));
                    L = bug.adjusted_mline;
                    x = -((robot_true(2) + dy) * L(2) + L(3)) / L(1);
                    dx = round(x - robot_true(1));
                end
        
                % Obstacle detection uses TRUE position
                if bug.isoccupied(robot_true + [dx; dy])
                    bug.message('(%d,%d) obstacle!', n);
                    bug.H(bug.j,:) = robot_true;
                    bug.step = 2;
                    bug.edge = edgelist(bug.occgridnav == 0, robot_true);
                    bug.k = 2;
                else
                    % Calculate next TRUE position
                    n_true = robot_true + [dx; dy];

                    [n, bug.cumulativeDistance, bug.driftState, sigma_par_inc, sigma_perp_inc, sigpar2_curr, sigperp2_curr, t_curr] = bug.applyDrift(robot_true, n_true, bug.cumulativeDistance, bug.driftState);

                    bug.driftLog_time_s      = [bug.driftLog_time_s,      t_curr];
                    bug.driftLog_dist_m      = [bug.driftLog_dist_m,      bug.cumulativeDistance];
                    bug.driftLog_posErr_m    = [bug.driftLog_posErr_m,    bug.driftState.pos_err_m];
                    bug.driftLog_yawErr_rad  = [bug.driftLog_yawErr_rad,  bug.driftState.yaw_err_rad];
                    bug.driftLog_sigpar_m    = [bug.driftLog_sigpar_m,    sigma_par_inc];
                    bug.driftLog_sigperp_m   = [bug.driftLog_sigperp_m,   sigma_perp_inc];
                    bug.driftLog_sigparCum_m = [bug.driftLog_sigparCum_m, sqrt(sigpar2_curr)];
                    bug.driftLog_sigperpCum_m=[bug.driftLog_sigperpCum_m, sqrt(sigperp2_curr)];
                    
                    % Plot vision cone (optional)
                    delta = n - robot;
                    heading = atan2(delta(2), delta(1));
        
                    if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                        delete(bug.visionConeHandle);
                    end
                end
            end % step 1
        
            % Step 2. Move around the obstacle
            if bug.step == 2
                            
                % --- 1) TRUE trajectory check ---
                % If True Robot Reaches → Adjusted Goal
                if norm(bug.goal_est(:) - robot_true(:)) <= 1
                    % If True Robot Reaches → True Goal
                    if norm(bug.goal_true(:) - robot_true(:)) <= goal_tol_cells
                        goalReached_true = true;
                        disp('TRUE trajectory → True Goal reached.');
                    
                        % --- 2) ESTIMATED trajectory evaluation ---
                        % If True Robot Reaches → True Goal &&
                        % If Drifted Robot Reaches → Adjusted Goal
                        if norm(bug.goal_est(:) - robot(:)) <= goal_tol_cells
                            goalReached_est = true;
                            disp('ESTIMATED trajectory → Adjusted Goal reached.');

                            % If True Robot Reaches → True Goal &&
                            % If Drifted Robot Reaches → Adjusted Goal &&
                            % If Drifted Robot Reaches → True Goal
                            if norm(bug.goal_true(:) - robot(:)) <= goal_tol_cells
                                disp('ESTIMATED trajectory → True Goal Reached.');
                            % If Drifted Robot NOT Reaches → True Goal
                            else
                                disp('ESTIMATED trajectory → True Goal NOT Reached (> 1 m away).');
                            end
                        % If Drifted Robot NOT Reaches → Adjusted Goal
                        else
                            goalReached_est = false;
                            disp('ESTIMATED trajectory → Adjusted Goal NOT reached (> 1 m away).');
                        end
                    
                        return

                    % If True Robot NOT Reaches → True Goal
                    else
                        goalReached_true = false;
                        disp('TRUE trajectory → True Goal NOT reached (> 1 m away).');

                        % --- 2) ESTIMATED trajectory evaluation ---
                        % If True Robot NOT Reaches → True Goal &&
                        % If Drifted Robot Reaches → Adjusted Goal
                        if norm(bug.goal_est(:) - robot(:)) <= goal_tol_cells
                            goalReached_est = true;
                            disp('ESTIMATED trajectory → Adjusted Goal reached.');

                            % If True Robot NOT Reaches → True Goal &&
                            % If Drifted Robot Reaches → Adjusted Goal &&
                            % If Drifted Robot Reaches → True Goal
                            if norm(bug.goal_true(:) - robot(:)) <= goal_tol_cells
                                disp('ESTIMATED trajectory → True Goal Reached.');
                            % If Drifted Robot NOT Reaches → True Goal
                            else
                                disp('ESTIMATED trajectory → True Goal NOT Reached (> 1 m away).');
                            end
                        else
                            goalReached_est = false;
                            disp('ESTIMATED trajectory → Adjusted Goal NOT reached (> 1 m away).');
                        end

                        return

                    end
                end
    
                if bug.k <= numcols(bug.edge)
                    % Get next edge point (this is TRUE position)
                    n_true = bug.edge(:,bug.k);

                    [n, bug.cumulativeDistance, bug.driftState, sigma_par_inc, sigma_perp_inc, sigpar2_curr, sigperp2_curr, t_curr] = bug.applyDrift(robot_true, n_true, bug.cumulativeDistance, bug.driftState);

                    bug.driftLog_time_s      = [bug.driftLog_time_s,      t_curr];
                    bug.driftLog_dist_m      = [bug.driftLog_dist_m,      bug.cumulativeDistance];
                    bug.driftLog_posErr_m    = [bug.driftLog_posErr_m,    bug.driftState.pos_err_m];
                    bug.driftLog_yawErr_rad  = [bug.driftLog_yawErr_rad,  bug.driftState.yaw_err_rad];
                    bug.driftLog_sigpar_m    = [bug.driftLog_sigpar_m,    sigma_par_inc];
                    bug.driftLog_sigperp_m   = [bug.driftLog_sigperp_m,   sigma_perp_inc];
                    bug.driftLog_sigparCum_m = [bug.driftLog_sigparCum_m, sqrt(sigpar2_curr)];
                    bug.driftLog_sigperpCum_m=[bug.driftLog_sigperpCum_m, sqrt(sigperp2_curr)];

                    % Plot vision cone (optional)
                    delta = n - robot;
                    heading = atan2(delta(2), delta(1));
        
                    if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                        delete(bug.visionConeHandle);
                    end
                else
                    return;
                end
        
                % M-line check uses TRUE position
                if abs([robot_true' 1]*bug.adjusted_mline') <= 0.9
                    bug.message('(%d,%d) moving along the M-line', n);
                    if colnorm(robot_true - bug.goal_est) < colnorm(bug.H(bug.j,:)' - bug.goal_est)
                        bug.j = bug.j + 1;
                        bug.step = 1;
                        return;
                    end
                end
                bug.message('(%d,%d) keep moving around obstacle', n)
                bug.k = bug.k+1;
            end % step 2
        end % next
                
        function plan(bug)
            error('RTB:Bug2:badcall', 'This class has no plan method');
        end

    end % methods

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
