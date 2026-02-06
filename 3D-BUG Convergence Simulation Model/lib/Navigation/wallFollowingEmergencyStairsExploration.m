%% Caption
% WALLFOLLOWINGEMERGENCYSTAIRSEXPLORATION  Wall-following exploration with vision-cone emergency-stair detection and Bug-style navigation-to-exit (E. Sangenis)
%
% A Navigation subclass implementing an original two-phase strategy to (i) explore
% an occupancy-grid map via goal-directed wall-following and (ii) immediately
% transition to dedicated emergency-stair navigation once a stair/exit is detected.
% The explorer moves along an m-line toward a nominal exploration goal, switches
% to boundary following when obstacles are encountered, and continuously scans for
% emergency stairs using a configurable vision cone with explicit line-of-sight
% validation. When an emergency stair is detected, the class re-initializes a Bug
% state machine to navigate to the detected exit with obstacle avoidance.
%
% Core behaviors::
%   • Exploration (default mode):
%       - goal set to a forward “corridor-following” waypoint (e.g., y = 600),
%       - Step 1: follow the m-line toward the exploration goal with discrete grid motion,
%       - Step 2: wall-following using an edge list around obstacles until the m-line is
%         re-acquired at a closer point than the original hit point.
%   • Perception and detection:
%       - vision parameters (visionRadius [m] converted to pixels, visionAngle [deg]),
%       - cone-based gating (range + angular sector) plus hasLineOfSight() ray checks,
%       - maintains spottedExitCells and visualizes discovered stairs on the map.
%   • Emergency navigation (triggered mode):
%       - on first valid detection: sets targetExit and switches isMovingToExit = true,
%       - redefines goal and m-line toward the detected exit,
%       - runs bugAlgorithmStep(): direct-to-goal motion with wall-following fallback,
%         including recovery logic when the robot becomes stuck.
%   • Safety and robustness:
%       - boundary clamping to keep positions within occgrid limits,
%       - step limit (e.g., 10,000) to avoid infinite exploration loops.
%
% Main methods::
%   wallFollowingEmergencyStairsExploration  Constructor (sets vision parameters, initializes state)
%   query                                   Run exploration and (if triggered) navigate to exit
%   setExitCells / getExitCells              Configure and retrieve emergency stair locations
%   checkForEmergencyStairs                  Cone + LOS-based detection during motion
%   hasLineOfSight                           Discrete ray-march LOS validation through occgrid
%   next                                    Bug-like exploration step (m-line + wall following)
%   bugAlgorithmStep                         Dedicated Bug-style motion toward targetExit
%   findAlternativeMove                      Fallback local move selection when stuck
%   plotVisionCone / rayCast / castSingleRay Visualization cone with occlusion-aware boundary
%
% Example::
%   nav = wallFollowingEmergencyStairsExploration(map, 'visionRadius', 30, 'visionAngle', 170);
%   nav.setExitCells(exitXY);
%   [path, reached, exitIdx] = nav.query(startXYZ, goalXYZ, 'animate', true);
%
% Copyright (C) 2026, Eudald Sangenis


%% Class
classdef wallFollowingEmergencyStairsExploration < Navigation

    properties(Access=protected)
        H       % hit points
        j       % number of hit points
        mline   % line from starting position to goal
        step    % state, in step 1 or step 2 of algorithm
        edge    % edge list
        k       % edge index
        visionConeHandle % Handle for the vision cone plot
        visionRadius        % Vision range in pixels
        visionAngle         % Vision cone angle in degrees
    
        exitCells % Emergency exit/stairs locations (Nx2 matrix)
        spottedExitCells       % Discovered emergency stairs
        emergencyStairFound   % Flag to indicate if emergency stair is found
        foundExitNumber       % Which exit was found



        stepSize = 1;
        isMovingToExit         % Flag indicating if robot is moving to emergency stairs
        targetExit             % Current target emergency stair position
    end

    methods
        function wallF = wallFollowingEmergencyStairsExploration(varargin)           
            % invoke the superclass constructor
            wallF = wallF@Navigation(varargin{:});
            wallF.H = [];
            wallF.j = 1;
            wallF.step = 1;
            wallF.visionConeHandle = []; % Initialize vision cone handle

            % Parse vision parameters
            opt.visionRadius = 30;
            opt.visionAngle = 170;
            opt = tb_optparse(opt, varargin);
            
            wallF.visionRadius = opt.visionRadius;
            wallF.visionAngle = opt.visionAngle;

            wallF.isMovingToExit = false;
            wallF.targetExit = [];       
        end

        function [pp, goalReached, exitStairsNumber] = query(wallF, start, goal, varargin)
            % Add maxTime and maxSteps options
            opt.animate = false;
            opt.movie = [];
            opt.current = false;
            goalReached = false;
            exitStairsNumber = 0;
            
            opt = tb_optparse(opt, varargin);
            
            if ~isempty(opt.movie)
                anim = Animate(opt.movie);
                opt.animate = true;
            end
        
            % make sure start and goal are set and valid
            wallF.start = []; wallF.goal = [];
            wallF.checkquery(start, goal);
            
            % Initialize flags for this query
            wallF.spottedExitCells = [];
            wallF.emergencyStairFound = false;
            wallF.foundExitNumber = 0;
            wallF.isMovingToExit = false;
            wallF.targetExit = [];
            
            wallF.goal = [wallF.start(1), 600, wallF.start(3)];
            fprintf('Wall-Follow goal: %d', wallF.goal)
        
            % compute the m-line
            wallF.mline = homline(wallF.start(1), wallF.start(2), ...
                wallF.goal(1), wallF.goal(2));
            wallF.mline = wallF.mline / norm(wallF.mline(1:2));
            
            if opt.animate
                wallF.plot();
                xlim([0,120]);
                axis equal;
                wallF.plot_mline();
            end
            
            % iterate using the next() method until we reach the goal
            robot = wallF.start(:);
            wallF.step = 1;
            path = wallF.start(:);
            stepCount = 0;
            stuckCounter = 0;  % Initialize this variable
            
            while true        
                if opt.animate
                    plot(robot(1), robot(2), 'g.', 'MarkerSize', 12);
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
        
                %%%%
                % Check for emergency stairs in vision (only if not already moving to one)
                if ~wallF.isMovingToExit
                    wallF.checkForEmergencyStairs(robot);
                end
               
                % If emergency stair found, switch to emergency navigation mode
                if wallF.emergencyStairFound && ~wallF.isMovingToExit
                    fprintf('Emergency stair found! Switching to emergency navigation mode.\n');
                    exitStairsNumber = wallF.foundExitNumber;
                    
                    % Set target and switch to emergency navigation mode
                    wallF.targetExit = wallF.exitCells(wallF.foundExitNumber, :);
                    wallF.isMovingToExit = true;
                    
                    % Update goal and m-line for emergency navigation
                    wallF.goal = [wallF.targetExit(1), wallF.targetExit(2), wallF.start(3)];
                    wallF.mline = homline(robot(1), robot(2), wallF.goal(1), wallF.goal(2));
                    wallF.mline = wallF.mline / norm(wallF.mline(1:2));
                    
                    % Reset Bug algorithm state for emergency navigation
                    wallF.step = 1;
                    wallF.H = [];
                    wallF.j = 1;
                    wallF.edge = [];
                    wallF.k = 1;
                    
                    fprintf('Target set to emergency exit at (%.1f, %.1f)\n', wallF.targetExit(1), wallF.targetExit(2));
                    fprintf('Distance to emergency exit: %.1f pixels\n', norm(robot(1:2) - wallF.targetExit));
                    
                    % DON'T BREAK HERE! Continue to navigate to the emergency exit
                end
                %%%%
        
                % Determine next position based on current mode
                if wallF.isMovingToExit
                    % Use Bug algorithm for obstacle avoidance to emergency stairs
                    nextPos = wallF.bugAlgorithmStep(robot);
        
                    % Check if we got a valid next position
                    if isempty(nextPos)
                        stuckCounter = stuckCounter + 1;
                        if stuckCounter > 10
                            fprintf('Robot appears stuck, resetting Bug algorithm state...\n');
                            wallF.step = 1;
                            wallF.H = [];
                            wallF.j = 1;
                            wallF.edge = [];
                            wallF.k = 1;
                            stuckCounter = 0;
                        end
                        continue;
                    else
                        stuckCounter = 0;
                    end
        
                    robot = nextPos;
                else
                    % Normal wall following exploration
                    [robot, goalReached] = wallF.next(robot);
                end

                % ✅ MOVE THE EXIT CHECK HERE - AFTER robot position is updated
                if wallF.isMovingToExit && ~isempty(wallF.targetExit) && norm(robot(1:2) - wallF.targetExit') < 0.5
                    fprintf('Reached emergency stairs!\n');
                    goalReached = true;
                    exitStairsNumber = wallF.foundExitNumber;
                    break;
                end
                
                % are we there yet?
                if isempty(robot)
                    % yes, exit the loop
                    break
                else
                    % no, append it to the path
                    path = [path [robot(:); wallF.start(3)]];
                end
                
                % Safety check
                stepCount = stepCount + 1;
                if stepCount > 10000
                    fprintf('Maximum steps reached. Stopping exploration.\n');
                    break;
                end
            end
            
            if ~isempty(opt.movie)
                anim.close();
            end
        
            % only return the path if required
            if nargout > 0
                pp = path';
            end
        end

        % Set emergency stairs locations
        function setExitCells(wallF, cells)
            % Ensure cells is in the correct format (Nx2 matrix)
            if size(cells, 1) == 2 && size(cells, 2) > 2
                % If cells is 2xN (columns are exits), transpose to Nx2
                wallF.exitCells = cells';
            else
                % If cells is already Nx2, use as is
                wallF.exitCells = cells;
            end
            
            fprintf('Set %d emergency exit cells\n', size(wallF.exitCells, 1));
            
            % Debug: print the exit cells
            for i = 1:size(wallF.exitCells, 1)
                fprintf('Exit %d: (%.1f, %.1f)\n', i, wallF.exitCells(i, 1), wallF.exitCells(i, 2));
            end
        end

        function cells = getExitCells(wallF, exitIndex)
            if nargin < 2
                % Return all exit cells if no index specified
                cells = wallF.exitCells;
            else
                % Return specific exit cell based on index
                if exitIndex > 0 && exitIndex <= size(wallF.exitCells, 1)
                    cells = wallF.exitCells(exitIndex, :);
                else
                    error('Invalid exit index: %d. Valid range is 1 to %d', exitIndex, size(wallF.exitCells, 1));
                end
            end
        end
        
        function plotEmergencyStair(wallF, position)
            hold on;
            scatter(position(1), position(2), 50, 'r', 'filled', 's', 'DisplayName', 'Emergency Stair');
        end

        function hasLOS = hasLineOfSight(wallF, startPos, endPos)
            % Check if there's a clear line of sight between two points
            % Returns true if no obstacles block the path
            
            startPos = startPos(:);  % Ensure column vector
            endPos = endPos(:);      % Ensure column vector
            
            % Calculate direction and distance
            direction = endPos - startPos;
            distance = norm(direction);
            
            if distance == 0
                hasLOS = true;
                return;
            end
            
            % Normalize direction
            direction = direction / distance;
            
            % Step along the line and check for obstacles
            stepSize = 0.5;  % Check every 0.5 pixels for accuracy
            numSteps = ceil(distance / stepSize);
            
            for step = 1:numSteps
                % Calculate current position along the line
                currentPos = startPos + (step * stepSize) * direction;
                
                % Round to grid coordinates
                gridPos = round(currentPos);
                
                % Check bounds
                if gridPos(1) < 1 || gridPos(2) < 1 || ...
                   gridPos(1) > size(wallF.occgridnav, 2) || ...
                   gridPos(2) > size(wallF.occgridnav, 1)
                    hasLOS = false;
                    return;
                end
                
                % Check if this position is occupied (wall)
                if wallF.isoccupied(gridPos)
                    hasLOS = false;
                    return;
                end
            end
            
            hasLOS = true;
        end
        
        function checkForEmergencyStairs(wallF, robot)
            if isempty(wallF.exitCells)
                return;
            end
            
            robot = robot(:);
            if length(robot) > 2
                robot = robot(1:2);
            end
            
            % Get current heading (direction of movement)
            if wallF.step == 1
                % Moving toward goal
                direction = wallF.goal(1:2) - robot;
            elseif wallF.step == 2 && wallF.k > 1
                % Following wall edge
                if wallF.k <= size(wallF.edge, 2)
                    direction = wallF.edge(:, wallF.k) - robot;
                else
                    direction = [1; 0]; % Default direction
                end
            else
                direction = [1; 0]; % Default direction
            end
            
            heading = atan2(direction(2), direction(1));
            
            % Create vision cone
            visionAngleRad = wallF.visionAngle * pi / 180;
            visionRadiusPx = wallF.visionRadius * 4;
            
            % Check each emergency stair
            for i = 1:size(wallF.exitCells, 1)
                exitPos = wallF.exitCells(i, :)';
                
                % Check if within vision radius
                distance = norm(robot - exitPos);
                if distance > visionRadiusPx
                    continue;
                end
                
                % Check if within vision cone angle
                toExit = exitPos - robot;
                angleToExit = atan2(toExit(2), toExit(1));
                angleDiff = abs(atan2(sin(angleToExit - heading), cos(angleToExit - heading)));
                
                if angleDiff > visionAngleRad / 2
                    continue;
                end
                
                % Check line of sight
                if wallF.hasLineOfSight(robot, exitPos)
                    fprintf('*** EMERGENCY STAIR FOUND IN VISION CONE! ***\n');
                    wallF.emergencyStairFound = true;
                    wallF.foundExitNumber = i;
                    
                    if isempty(wallF.spottedExitCells) || ~any(all(wallF.spottedExitCells == wallF.exitCells(i,:), 2))
                        wallF.spottedExitCells = [wallF.spottedExitCells; wallF.exitCells(i,:)];
                        wallF.plotEmergencyStair(wallF.exitCells(i,:));
                    end
                    return;
                end
            end
        end

%         function checkVisibleEmergencyStairs(wallF, robot, coneVertices)
%             if isempty(wallF.exitCells)
%                 return;
%             end
%             
%             % Extract only X,Y coordinates from robot position
%             robot = robot(:);
%             if length(robot) > 2
%                 robot = robot(1:2);  % Take only X,Y coordinates, ignore Z
%             end
%             
%             % Check each emergency stair location
%             for i = 1:size(wallF.exitCells, 1)
%                 exitPos = wallF.exitCells(i, :);  % Row vector (1x2)
%                 
%                 % Check if the exit is within the vision cone
%                 if inpolygon(exitPos(1), exitPos(2), coneVertices(1, :), coneVertices(2, :))
%                     % Mark as found
%                     wallF.emergencyStairFound = true;
%                     wallF.foundExitNumber = i;
%                     
%                     % Add to spotted exits if not already there
%                     if isempty(wallF.spottedExitCells) || ~any(all(wallF.spottedExitCells == exitPos, 2))
%                         wallF.spottedExitCells = [wallF.spottedExitCells; exitPos];
%                         wallF.plotEmergencyStair(exitPos);
%                         fprintf('Emergency stair %d spotted at position (%.1f, %.1f)\n', i, exitPos(1), exitPos(2));
%                     end
%                     
%                     return; % Stop at first found stair
%                 end
%             end
%         end
        
        function plot_mline(wallF, ls)
            % parameters of the M-line, direct from initial position to goal
            % as a vector mline, such that [robot 1]*mline = 0
            
            if nargin < 2
                ls = 'k--';
            end
            dims = axis;
            xmin = dims(1); xmax = dims(2);
            ymin = dims(3); ymax = dims(4);
            
            hold on
            if wallF.mline(2) == 0
                % handle the case that the line is vertical
                plot([wallF.start(1) wallF.start(1)], [ymin ymax], 'k--');
            else
                x = [xmin xmax]';
                y = -[x [1;1]] * [wallF.mline(1); wallF.mline(3)] / wallF.mline(2);
                plot(x, y, ls);
                axis equal
            end
        end

        function handle = plotVisionCone(bug, robot, heading)
            visionAngle = 45*pi/180;
            visionRadius = 30*4; % 30 meters (4 conversion pxl to meters)
            % Plot the vision cone based on the robot's position, angle, and radius
            theta = linspace(-visionAngle/2, visionAngle/2, 30)  + heading;
            [x, y] = bug.rayCast(robot, theta, visionRadius);
            handle = fill([robot(1), x], [robot(2), y], 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none'); % Return handle
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

        function [n, goalReached] = next(wallF, robot)
            
            % implement the main state machine for wallF
            n = [];
            goalReached = false;
            robot = robot(:);
            % these are coordinates (x,y)
          
            if wallF.step == 1
                % Step 1.  Move along the M-line toward the goal

                if colnorm(wallF.goal(1:2) - robot(1:2)') == 0 % are we there yet?
                    goalReached = true;
                    disp('Goal reached.');
                    return
                end

                % Motion on the line toward goal
                d = wallF.goal(1:2)' - robot(1:2);    
    
                if abs(d(1)) > abs(d(2))
                    % Line slope less than 45 degrees
                    dx = sign(d(1));
                    L = wallF.mline;
                    if L(2) >= 1e-5 || L(2) <= -1e-5
                        y = -((robot(1) + dx) * L(1) + L(3)) / L(2);
                        dy = round(y - robot(2));
                    else
                        % Vertical line case
                        dy = sign(d(2));
                    end
                else
                    % Line slope greater than 45 degrees
                    dy = sign(d(2));
                    L = wallF.mline;
                    x = -((robot(2) + dy) * L(2) + L(3)) / L(1);
                    dx = round(x - robot(1));
                end

                % detect if next step is an obstacle
                if wallF.isoccupied(robot(1:2) + [dx; dy])
                    wallF.message('(%d,%d) obstacle!', n);
                    wallF.H(wallF.j,:) = robot; % define hit point
                    wallF.step = 2;
                    % get a list of all the points around the obstacle
                    wallF.edge = edgelist(wallF.occgridnav == 0, robot);
                    wallF.k = 2;  % skip the first edge point, we are already there
                else
                    n = robot(1:2) + [dx; dy];

                    % Plot the vision cone after moving
                    delta = n - robot(1:2);
                    heading = atan2(delta(2), delta(1));

                    % Delete previous vision cone if it exists
                    if ~isempty(wallF.visionConeHandle) && isvalid(wallF.visionConeHandle)
                        delete(wallF.visionConeHandle);
                    end

%                     % Plot the new vision cone
                    wallF.visionConeHandle = wallF.plotVisionCone(n, heading);
                end
            end % step 1

            if wallF.step == 2
                % Step 2.  Move around the obstacle until we reach a point
                % on the M-line closer than when we started.
                if colnorm(wallF.goal-robot) == 0 % are we there yet?
                    return
                end

                if wallF.k <= numcols(wallF.edge)
                    n = wallF.edge(:,wallF.k);  % next edge point

                    % Plot the vision cone after moving
                    delta = n - robot;
                    heading = atan2(delta(2), delta(1));

                    % Delete previous vision cone if it exists
                    if ~isempty(wallF.visionConeHandle) && isvalid(wallF.visionConeHandle)
                        delete(wallF.visionConeHandle);
                    end

                    % Plot the new vision cone
                    wallF.visionConeHandle = wallF.plotVisionCone(n, heading);
                else
                    % we are at the end of the list of edge points, we
                    % are back where we started.  Step 2.c test.
%                     error('RTB:wallF:noplan', 'robot is trapped')
                    return;
                end

                % are we on the M-line now ?
                if abs( [robot' 1]*wallF.mline') <= 0.9 % 0.7 EUDALD CHANGED THRESHOLD FOR COUNTING M-LINE CROSSING SQUARED OBJECTS VERTEXS: OLD "0.5"
                    wallF.message('(%d,%d) moving along the M-line', n);
                    % are closer than when we encountered the obstacle?
                    if colnorm(robot-wallF.goal) < colnorm(wallF.H(wallF.j,:)'-wallF.goal)
                        % back to moving along the M-line
                        wallF.j = wallF.j + 1;
                        wallF.step = 1;
                        return;
                    end
                end
                % no, keep going around
                wallF.message('(%d,%d) keep moving around obstacle', n)
                wallF.k = wallF.k+1;
            end % step 2
        end % next

%         function initializeBugAlgorithm(wallF, start, goal)
%             % Initialize Bug algorithm for moving to emergency stairs
%             wallF.H = zeros(1, 2);
%             wallF.j = 1;
%             wallF.step = 1;
%             wallF.edge = [];
%             wallF.k = 1;
%             
%             % Compute m-line from current position to emergency stairs
%             wallF.mline = homline(start(1), start(2), goal(1), goal(2));
%             wallF.mline = wallF.mline / norm(wallF.mline(1:2));
%             
%             % Update goal to be the emergency stairs
%             wallF.goal = [goal(1), goal(2), wallF.goal(3)]; % Keep original Z coordinate
%             
%             fprintf('Bug algorithm initialized: moving from (%.1f,%.1f) to emergency stairs at (%.1f,%.1f)\n', ...
%                     start(1), start(2), goal(1), goal(2));
%         end
        
        function plan(wallF)
            error('RTB:wallF2:badcall', 'This class has no plan method');
        end

        function nextPos = bugAlgorithmStep(wallF, robot)
            % Bug algorithm step for moving to emergency stairs with obstacle avoidance
            nextPos = [];
            
            % Ensure robot position is within grid boundaries
            [rows, cols] = size(wallF.occgridnav);
            robot(1) = max(1, min(cols, robot(1)));
            robot(2) = max(1, min(rows, robot(2)));
            
            if wallF.step == 1
                % Step 1: Move towards goal along m-line
                direction = wallF.targetExit' - robot(1:2);
                if norm(direction) < 1e-6
                    nextPos = robot; % Already at goal
                    return;
                end
                
                % Calculate unit step towards goal
                unitDirection = direction / norm(direction);
                step = unitDirection * wallF.stepSize;
                
                % Round to get discrete movement
                dx = round(step(1));
                dy = round(step(2));
                
                % Ensure minimum movement
                if dx == 0 && dy == 0
                    if abs(unitDirection(1)) > abs(unitDirection(2))
                        dx = sign(unitDirection(1));
                    else
                        dy = sign(unitDirection(2));
                    end
                end
                
                candidatePos = robot(1:2) + [dx; dy];
                
                % Ensure candidate position is within bounds
                candidatePos(1) = max(1, min(cols, candidatePos(1)));
                candidatePos(2) = max(1, min(rows, candidatePos(2)));
                
                % Check for obstacles
                if wallF.isoccupied([candidatePos(1); candidatePos(2)])
                    fprintf('Obstacle detected! Switching to wall following mode\n');
                    % Hit an obstacle - switch to step 2
                    wallF.H(wallF.j, :) = robot';
                    wallF.step = 2;
                    
                    % Validate robot position before calling edgelist
                    try
                        % Ensure robot is within valid bounds for edgelist
                        robotInt = round(robot);
                        robotInt(1) = max(1, min(cols, robotInt(1)));
                        robotInt(2) = max(1, min(rows, robotInt(2)));
                        
                        % Get edge list for wall following
                        wallF.edge = edgelist(wallF.occgridnav == 0, robotInt);
                        wallF.k = 2; % Skip first point (current position)
                        
                        if wallF.k <= size(wallF.edge, 2)
                            nextPos = wallF.edge(:, wallF.k);
                            % Ensure next position is within bounds
                            nextPos(1) = max(1, min(cols, nextPos(1)));
                            nextPos(2) = max(1, min(rows, nextPos(2)));
                        else
                            % No edge found, try alternative movement
                            nextPos = wallF.findAlternativeMove(robot, candidatePos);
                        end
                    catch ME
                        fprintf('Error in edgelist: %s\n', ME.message);
                        % Fallback to alternative movement strategy
                        nextPos = wallF.findAlternativeMove(robot, candidatePos);
                    end
                else
                    nextPos = candidatePos;
                end
                
            elseif wallF.step == 2
                % Step 2: Wall following mode
                if wallF.k <= size(wallF.edge, 2)
                    nextPos = wallF.edge(:, wallF.k);
                    % Ensure next position is within bounds
                    nextPos(1) = max(1, min(cols, nextPos(1)));
                    nextPos(2) = max(1, min(rows, nextPos(2)));
                    wallF.k = wallF.k + 1;
                else
                    fprintf('Completed wall following - trying alternative approach\n');
                    % Try alternative movement when wall following is complete
                    nextPos = wallF.findAlternativeMove(robot, wallF.targetExit');
                end
                
                % Check if we're back on m-line and closer to goal
                if ~isempty(nextPos) && abs([robot' 1] * wallF.mline') <= 2.0
                    currentDist = norm(robot - wallF.targetExit');
                    hitDist = norm(wallF.H(wallF.j, :)' - wallF.targetExit');
                    
                    if currentDist < hitDist
                        fprintf('Back on m-line and closer to goal - resuming direct movement\n');
                        wallF.j = wallF.j + 1;
                        wallF.step = 1;
                    end
                end
            end
        end

        function nextPos = findAlternativeMove(wallF, robot, targetPos)
            % Find alternative movement when stuck
            directions = [1,0; -1,0; 0,1; 0,-1; 1,1; 1,-1; -1,1; -1,-1];
            
            robot = robot(1:2); % Ensure only x,y coordinates
            
            for i = 1:size(directions, 1)
                candidatePos = robot + directions(i, :)';
                
                % Check bounds
                if candidatePos(1) >= 1 && candidatePos(2) >= 1 && ...
                   candidatePos(1) <= size(wallF.occgridnav, 2) && ...
                   candidatePos(2) <= size(wallF.occgridnav, 1)
                    
                    % Check if free
                    if ~wallF.isoccupied(candidatePos)
                        nextPos = candidatePos;
                        return;
                    end
                end
            end
            
            nextPos = robot; % Stay in place if no alternative found
        end

    end % methods
end % classdef