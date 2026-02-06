%% Caption
% RANDOMPATHEMERGENCYSTAIRSEXPLORATION  Random exploration with vision-based emergency-stair discovery and Bug-style obstacle avoidance (E. Sangenis)
%
% A Navigation subclass implementing an original two-mode exploration policy for
% locating and reaching emergency stairs (exits) in an occupancy-grid environment.
% The agent performs unbiased/random walk exploration using an 8-direction motion
% model, continuously scanning with a configurable vision cone (ray-cast for
% occlusions). Once an emergency stair becomes visible, the agent switches from
% random exploration to a directed “move-to-exit” mode that uses a simplified Bug
% state machine for obstacle avoidance until the exit is reached.
%
% Core behaviors::
%   • Random exploration mode:
%       - 8 discrete headings (0–315° in 45° increments) with a fixed stepSize,
%       - direction persistence for minStepsBeforeChange steps, with probabilistic
%         heading changes and a blocked-direction mechanism to escape dead-ends.
%   • Perception and discovery:
%       - vision cone (visionRadius, visionAngle) rendered for visualization,
%       - ray-casting to compute line-of-sight boundaries and detect exits only,
%       - discovered exits tracked via spottedExitCells and highlighted on the map.
%   • Directed move-to-exit mode:
%       - upon first sighting an exit, sets targetExit and initializes an m-line,
%       - Step 1: greedy motion toward the exit along the m-line,
%       - Step 2: wall-following using an edge list when an obstacle blocks motion,
%         returning to Step 1 when re-intersecting the m-line closer than the hit point,
%       - fallback “alternative move” when edge extraction fails or no edge remains.
%   • Safety and robustness:
%       - boundary clamping to keep robot positions within occgrid limits,
%       - maxTime / maxSteps termination in query() to prevent infinite loops.
%
% Main methods::
%   randomPathEmergencyStairsExploration  Constructor (sets exploration/perception parameters)
%   setExitCells / getExitCells           Define and retrieve emergency stair locations
%   query                                Run exploration; returns path and exit index if reached
%   plotVisionCone                        Render cone and trigger exit detection
%   randomExplorationStep                 Random-walk step with blocked-direction handling
%   bugAlgorithmStep                      Directed step toward targetExit with Bug-style avoidance
%   rayCast / castSingleRay               Line-of-sight computation for cone boundaries
%
% Example::
%   exp = randomPathEmergencyStairsExploration(map, ...
%           'visionRadius', 30, 'visionAngle', 170, 'stepSize', 2);
%   exp.setExitCells(exitXY);
%   [path, reached, exitIdx] = exp.query(startXYZ, goalXYZ, 'animate', true);
%
% Copyright (C) 2026, Eudald Sangenis

%% Class
classdef randomPathEmergencyStairsExploration < Navigation
    properties(Access=protected)
        % Vision and detection properties
        visionRadius
        visionAngle
        visionConeHandle
        
        % Emergency infrastructure
        exitCells              % Emergency stairs locations
        spottedExitCells       % Discovered emergency stairs
        
        % Random movement
        randStream             % Dedicated random stream
        currentHeading         % Current movement direction (index 1-8)
        stepSize               % Size of each movement step
        stepsInCurrentDirection % Counter for steps in current direction
        minStepsBeforeChange   % Minimum steps before allowing direction change
        validDirections        % 8 valid directions in radians
        blockedDirections      % Temporarily blocked directions
        
        % Bug algorithm for obstacle avoidance when moving to emergency stairs
        isMovingToExit         % Flag indicating if robot is moving to emergency stairs
        targetExit             % Current target emergency stair position
        H                      % Hit points for Bug algorithm
        j                      % Number of hit points
        step                   % Bug algorithm step (1 or 2)
        edge                   % Edge list for wall following
        k                      % Edge index
        mline                  % M-line to target
    end
    
    methods
        function explorer = randomPathEmergencyStairsExploration(occgrid, varargin)
            % Constructor
            explorer = explorer@Navigation(occgrid, varargin{:});
            
            % Parse options
            opt.visionRadius = 30;
            opt.visionAngle = 170;
            opt.stepSize = 2;
            opt.minStepsBeforeChange = 15*2; %*3
            opt = tb_optparse(opt, varargin);
            
            % Initialize properties
            explorer.visionRadius = opt.visionRadius;
            explorer.visionAngle = opt.visionAngle;
            explorer.stepSize = opt.stepSize;
            explorer.minStepsBeforeChange = opt.minStepsBeforeChange;
            
            % Initialize arrays
            explorer.exitCells = [];
            explorer.spottedExitCells = [];
            explorer.blockedDirections = [];
            
            % Define 8 valid directions
            explorer.validDirections = [0, 45, 90, 135, 180, 225, 270, 315] * pi/180;
            
            % Initialize random stream and heading
            explorer.randStream = RandStream('mt19937ar');
            explorer.currentHeading = randi(explorer.randStream, 8);
            explorer.stepsInCurrentDirection = 0;
            
            % Initialize Bug algorithm properties
            explorer.isMovingToExit = false;
            explorer.targetExit = [];
            explorer.H = zeros(1, 2);
            explorer.j = 1;
            explorer.step = 1;
            explorer.edge = [];
            explorer.k = 1;
            explorer.mline = [];
            
            explorer.visionConeHandle = [];
        end

        % Set emergency stairs locations
        function setExitCells(explorer, cells)
            explorer.exitCells = cells;
        end

        function cells = getExitCells(explorer)
            cells = explorer.exitCells;
        end

        function [handle, coneVertices] = plotVisionCone(explorer, robot, headingIndex)
            % Plot vision cone and detect emergency stairs
            heading = explorer.validDirections(headingIndex);
            visionAngle_var = explorer.visionAngle * pi / 180;
            visionRadius_var = explorer.visionRadius * 4;
            
            numPoints = 100;
            theta = linspace(-visionAngle_var/2, visionAngle_var/2, numPoints) + heading;
            
            % Perform ray casting
            [x, y] = explorer.rayCast(robot, theta, visionRadius_var);
            
            % Create vision cone vertices
            coneVertices = [robot(1), x; robot(2), y];
            
            % Plot vision cone
            closedX = [robot(1), x, robot(1)];
            closedY = [robot(2), y, robot(2)];
            handle = fill(closedX, closedY, 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
            
            % Check for visible emergency stairs only
            explorer.checkVisibleEmergencyStairs(robot, coneVertices);
        end
        
        function checkVisibleEmergencyStairs(explorer, robot, coneVertices)
            % Check for emergency stairs only
            for i = 1:size(explorer.exitCells, 1)
                if ~ismember(i, explorer.spottedExitCells)
                    exitPos = explorer.exitCells(i, :);
                    if inpolygon(exitPos(1), exitPos(2), coneVertices(1,:), coneVertices(2,:))
                        explorer.spottedExitCells = [explorer.spottedExitCells, i];
                        explorer.plotEmergencyStair(exitPos);
                        fprintf('Emergency stair discovered at (%.1f, %.1f)\n', exitPos(1), exitPos(2));
                        
                        % Switch to Bug algorithm mode for obstacle avoidance
                        explorer.isMovingToExit = true;
                        explorer.targetExit = exitPos;
                        explorer.initializeBugAlgorithm(robot, exitPos);
                    end
                end
            end
        end

        function initializeBugAlgorithm(explorer, start, goal)
            % Initialize Bug algorithm for moving to emergency stairs
            explorer.H = zeros(1, 2);
            explorer.j = 1;
            explorer.step = 1;
            explorer.edge = [];
            explorer.k = 1;
            
            % Compute m-line from current position to emergency stairs
            explorer.mline = homline(start(1), start(2), goal(1), goal(2));
            explorer.mline = explorer.mline / norm(explorer.mline(1:2));
            
            fprintf('Bug algorithm initialized: moving from (%.1f,%.1f) to emergency stairs at (%.1f,%.1f)\n', ...
                    start(1), start(2), goal(1), goal(2));
        end

        function [path, goalReached, exitStairsNumber] = query(explorer, start, goal, varargin)
            % Main exploration function
            opt.animate = false;
            opt.maxTime = 300;
            opt.maxSteps = 1000;
            opt = tb_optparse(opt, varargin);
            
            goalReached = false;
            exitStairsNumber = 0;
            
            % Initialize
            robot = start(1:2);
            if size(robot, 1) == 1
                robot = robot';
            end
            
            path = robot';
            startTime = tic;
            stepCount = 0;
            stuckCounter = 0;
            
            if opt.animate
                explorer.plot();
                hold on;
                xlim([0, 120]);
                axis equal;
            end
            
            fprintf('Starting exploration with initial direction: %.0f°\n', ...
                    explorer.validDirections(explorer.currentHeading)*180/pi);
            
            % Main exploration loop
            while toc(startTime) < opt.maxTime && stepCount < opt.maxSteps
                stepCount = stepCount + 1;
                
                % Check if we've reached emergency stairs
                if explorer.isMovingToExit && norm(robot - explorer.targetExit') < 3
                    fprintf('Reached emergency stairs!\n');
                    goalReached = true;
                    exitStairsNumber = find(all(abs(explorer.exitCells - explorer.targetExit) < 1e-6, 2));
                    break;
                end
                
                % Determine next position based on current mode
                if explorer.isMovingToExit
                    % Use Bug algorithm for obstacle avoidance to emergency stairs
                    nextPos = explorer.bugAlgorithmStep(robot);
                else
                    % Use random exploration
                    nextPos = explorer.randomExplorationStep(robot);
                end
                
                % Check if we got a valid next position
                if isempty(nextPos)
                    stuckCounter = stuckCounter + 1;
                    if stuckCounter > 10
                        explorer.blockedDirections = [];
                        stuckCounter = 0;
                    end
                    continue;
                else
                    stuckCounter = 0;
                end
                
                % Move to next position
                robot = nextPos;
                if size(robot, 1) == 1
                    robot = robot';
                end
                path = [path; robot'];
                
                % Update vision cone
                if ~explorer.isMovingToExit
                    if ~isempty(explorer.visionConeHandle) && isvalid(explorer.visionConeHandle)
                        delete(explorer.visionConeHandle);
                    end
                    explorer.visionConeHandle = explorer.plotVisionCone(robot, explorer.currentHeading);
                end
                
                if opt.animate
                    plot(robot(1), robot(2), 'g.', 'MarkerSize', 8);
                    drawnow;
                    pause(0.05);
                end
                
                if mod(stepCount, 10) == 0
                    if explorer.isMovingToExit
                        fprintf('Step %d: Moving to emergency stairs at (%.1f, %.1f), robot at (%.1f, %.1f)\n', ...
                                stepCount, explorer.targetExit(1), explorer.targetExit(2), robot(1), robot(2));
                    else
                        fprintf('Step %d: Random exploration, robot at (%.1f, %.1f), direction %.0f°\n', ...
                                stepCount, robot(1), robot(2), ...
                                explorer.validDirections(explorer.currentHeading)*180/pi);
                    end
                end
            end
            
            if ~goalReached
                fprintf('Exploration completed. No emergency stairs reached.\n');
                fprintf('Completed %d steps in %.2f seconds\n', stepCount, toc(startTime));
            end
        end

        function nextPos = bugAlgorithmStep(explorer, robot)
            % Bug algorithm step for moving to emergency stairs with obstacle avoidance
            nextPos = [];
            
            % Ensure robot position is within grid boundaries
            [rows, cols] = size(explorer.occgridnav);
            robot(1) = max(1, min(cols, robot(1)));
            robot(2) = max(1, min(rows, robot(2)));
            
            if explorer.step == 1
                % Step 1: Move towards goal along m-line
                direction = explorer.targetExit' - robot;
                if norm(direction) < 1e-6
                    nextPos = robot; % Already at goal
                    return;
                end
                
                % Calculate unit step towards goal
                unitDirection = direction / norm(direction);
                step = unitDirection * explorer.stepSize;
                
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
                
                candidatePos = robot + [dx; dy];
                
                % Ensure candidate position is within bounds
                candidatePos(1) = max(1, min(cols, candidatePos(1)));
                candidatePos(2) = max(1, min(rows, candidatePos(2)));
                
                % Check for obstacles
                if explorer.isoccupied([candidatePos(1); candidatePos(2)])
                    fprintf('Obstacle detected! Switching to wall following mode\n');
                    % Hit an obstacle - switch to step 2
                    explorer.H(explorer.j, :) = robot';
                    explorer.step = 2;
                    
                    % Validate robot position before calling edgelist
                    try
                        % Ensure robot is within valid bounds for edgelist
                        robotInt = round(robot);
                        robotInt(1) = max(1, min(cols, robotInt(1)));
                        robotInt(2) = max(1, min(rows, robotInt(2)));
                        
                        % Get edge list for wall following
                        explorer.edge = edgelist(explorer.occgridnav == 0, robotInt);
                        explorer.k = 2; % Skip first point (current position)
                        
                        if explorer.k <= size(explorer.edge, 2)
                            nextPos = explorer.edge(:, explorer.k);
                            % Ensure next position is within bounds
                            nextPos(1) = max(1, min(cols, nextPos(1)));
                            nextPos(2) = max(1, min(rows, nextPos(2)));
                        else
                            % No edge found, try alternative movement
                            nextPos = explorer.findAlternativeMove(robot, candidatePos);
                        end
                    catch ME
                        fprintf('Error in edgelist: %s\n', ME.message);
                        % Fallback to alternative movement strategy
                        nextPos = explorer.findAlternativeMove(robot, candidatePos);
                    end
                else
                    nextPos = candidatePos;
                end
                
            elseif explorer.step == 2
                % Step 2: Wall following mode
                if explorer.k <= size(explorer.edge, 2)
                    nextPos = explorer.edge(:, explorer.k);
                    % Ensure next position is within bounds
                    nextPos(1) = max(1, min(cols, nextPos(1)));
                    nextPos(2) = max(1, min(rows, nextPos(2)));
                    explorer.k = explorer.k + 1;
                else
                    fprintf('Completed wall following - trying alternative approach\n');
                    % Try alternative movement when wall following is complete
                    nextPos = explorer.findAlternativeMove(robot, explorer.targetExit');
                end
                
                % Check if we're back on m-line and closer to goal
                if ~isempty(nextPos) && abs([robot' 1] * explorer.mline') <= 2.0
                    currentDist = norm(robot - explorer.targetExit');
                    hitDist = norm(explorer.H(explorer.j, :)' - explorer.targetExit');
                    
                    if currentDist < hitDist
                        fprintf('Back on m-line and closer to goal - resuming direct movement\n');
                        explorer.j = explorer.j + 1;
                        explorer.step = 1;
                    end
                end
            end
        end

        function nextPos = findAlternativeMove(explorer, robot, blockedPos)
            % Find alternative movement when direct path or wall following fails
            [rows, cols] = size(explorer.occgridnav);
            
            % Try 8 directions around current position
            directions = [1,0; 1,1; 0,1; -1,1; -1,0; -1,-1; 0,-1; 1,-1];
            
            for i = 1:size(directions, 1)
                testPos = robot + directions(i,:)' * explorer.stepSize;
                
                % Ensure within bounds
                testPos(1) = max(1, min(cols, testPos(1)));
                testPos(2) = max(1, min(rows, testPos(2)));
                
                % Check if position is free
                if ~explorer.isoccupied([testPos(1); testPos(2)])
                    nextPos = testPos;
                    return;
                end
            end
            
            % If no free position found, stay in place
            nextPos = robot;
            fprintf('No alternative move found, staying in place\n');
        end
        
        function nextPos = randomExplorationStep(explorer, robot)
            % Random exploration step (original logic)
            nextPos = [];
            
            % Random movement: change direction after minimum steps
            if explorer.stepsInCurrentDirection >= explorer.minStepsBeforeChange && ...
               explorer.randStream.rand() < 0.3
                
                oldSteps = explorer.stepsInCurrentDirection;
                availableDirections = setdiff(1:8, explorer.blockedDirections);
                if ~isempty(availableDirections)
                    newDirection = availableDirections(randi(explorer.randStream, length(availableDirections)));
                else
                    newDirection = randi(explorer.randStream, 8);
                    explorer.blockedDirections = [];
                end
                
                explorer.currentHeading = newDirection;
                explorer.stepsInCurrentDirection = 0;
                fprintf('Changing direction to %.0f° after %d steps\n', ...
                        explorer.validDirections(explorer.currentHeading)*180/pi, oldSteps);
            end
            
            % Calculate next position using current direction
            currentAngle = explorer.validDirections(explorer.currentHeading);
            candidatePos = robot + explorer.stepSize * [cos(currentAngle); sin(currentAngle)];
            
            % Check boundaries
            [rows, cols] = size(explorer.occgridnav);
            candidatePos(1) = max(1, min(cols, candidatePos(1)));
            candidatePos(2) = max(1, min(rows, candidatePos(2)));
            
            % Check if next position is occupied
            if explorer.isoccupied([candidatePos(1); candidatePos(2)])
                % Mark direction as blocked and find alternative
                if ~ismember(explorer.currentHeading, explorer.blockedDirections)
                    explorer.blockedDirections = [explorer.blockedDirections, explorer.currentHeading];
                end
                
                % Try to find free direction
                availableDirections = setdiff(1:8, explorer.blockedDirections);
                if isempty(availableDirections)
                    explorer.blockedDirections = [];
                    availableDirections = 1:8;
                end
                
                availableDirections = availableDirections(randperm(explorer.randStream, length(availableDirections)));
                
                for dirIndex = availableDirections
                    testAngle = explorer.validDirections(dirIndex);
                    testPos = robot + explorer.stepSize * [cos(testAngle); sin(testAngle)];
                    
                    testPos(1) = max(1, min(cols, testPos(1)));
                    testPos(2) = max(1, min(rows, testPos(2)));
                    
                    if ~explorer.isoccupied([testPos(1); testPos(2)])
                        explorer.currentHeading = dirIndex;
                        nextPos = testPos;
                        explorer.stepsInCurrentDirection = 0;
                        break;
                    end
                end
                
                if isempty(nextPos)
                    nextPos = robot; % Stay in place if no free direction found
                end
            else
                nextPos = candidatePos;
                explorer.stepsInCurrentDirection = explorer.stepsInCurrentDirection + 1;
            end
            
            % Clear blocked directions periodically
            if mod(explorer.stepsInCurrentDirection, 50) == 0
                explorer.blockedDirections = [];
            end
        end
        
        function plotEmergencyStair(explorer, position)
            hold on;
            scatter(position(1), position(2), 100, 'r', 'filled', 's', 'DisplayName', 'Emergency Stair');
        end
        
        % Ray casting implementation
        function [x, y] = rayCast(explorer, start, angles, maxDist)
            x = zeros(1, length(angles));
            y = zeros(1, length(angles));
            
            for i = 1:length(angles)
                [x(i), y(i)] = explorer.castSingleRay(start, angles(i), maxDist);
            end
        end
        
        function [endX, endY] = castSingleRay(explorer, start, angle, maxDist)
            dx = cos(angle);
            dy = sin(angle);
            
            for dist = 1:maxDist
                checkX = round(start(1) + dist * dx);
                checkY = round(start(2) + dist * dy);
                
                if checkX < 1 || checkY < 1 || checkX > size(explorer.occgridnav, 2) || checkY > size(explorer.occgridnav, 1)
                    break;
                end
                
                if explorer.isoccupied([checkX; checkY])
                    break;
                end
            end
            
            endX = start(1) + (dist - 1) * dx;
            endY = start(2) + (dist - 1) * dy;
        end

        % Required abstract methods from Navigation class
        function plan(explorer, goal)
            if nargin >= 2
                explorer.goal = goal(:);
            end
        end
        
        function n = next(explorer, robot)
            if explorer.isMovingToExit
                n = explorer.bugAlgorithmStep(robot);
            else
                n = explorer.randomExplorationStep(robot);
            end
        end
    end
end