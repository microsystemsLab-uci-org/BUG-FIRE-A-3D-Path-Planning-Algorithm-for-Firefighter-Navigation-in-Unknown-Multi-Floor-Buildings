%% Caption
% DSTARLITE_ES  Incremental D* Lite navigation with partial observability and vision-cone obstacle discovery (E. Sangenis)
%
% A Navigation subclass implementing an original D* Lite–based path planner for
% occupancy-grid environments where the robot starts with an *unknown* map and
% incrementally discovers obstacles through a ray-cast vision cone. The class
% maintains standard D* Lite state (g, rhs, priority queue U, and key modifier km),
% replans online when newly observed obstacles change traversal costs, and produces
% a collision-free path from start to goal under limited field-of-view sensing.
%
% Core behaviors::
%   • Unknown-world initialization:
%       - initializeUnknownCostmap(): optimistic free-space costmap (cost=1) with a
%         separate knownCostmap used only to simulate sensing; observedOccGrid tracks
%         which cells have been observed.
%   • D* Lite planning and incremental repair:
%       - plan(): initializes rhs(goal)=0 and seeds the open list U with goal key.
%       - compute_shortest_path(): performs D* Lite vertex expansions until the start
%         is locally consistent or the queue ordering condition is satisfied.
%       - update_vertex(), calculate_key(), compare_keys(), insert_or_update(), and
%         get_neighbors()/get_cost() implement the core algorithm mechanics.
%       - handleCostChanges(): updates vertices affected by newly discovered obstacles
%         and adjusts km for incremental searches.
%   • Online sensing and map updates:
%       - plotVisionCone() + rayCast()/castSingleRay() form an occlusion-aware FOV.
%       - updateCostmapFromVision(): converts newly visible occupied cells to Inf cost,
%         throttles per-iteration obstacle processing, and triggers replanning.
%       - isVisibleFromRobot() uses Bresenham line checks against observed obstacles
%         to enforce realistic visibility constraints.
%   • Motion selection:
%       - next(): selects the next grid cell by minimizing g(neighbor)+edge_cost while
%         rejecting neighbors that are occupied in the true map; rejected moves are
%         injected as cost changes to force immediate local replanning.
%   • Debugging utilities:
%       - verbose progress prints (debug flag), debugPathfinding() traces local neighbor
%         costs/queue state, and optional visualization of detected obstacles.
%
% Notes::
%   - This implementation assumes an 8-connected grid (diagonal moves allowed) and uses
%     an octile-distance heuristic with a small tie-breaker term to stabilize ordering.
%   - Vision parameters are specified in meters and converted to pixels via a fixed scale
%     factor (e.g., *4 px per meter*), consistent with the rest of the navigation suite.
%
% Example::
%   ds = DstarLite_ES(map, 'visionRadius', 30, 'visionAngle', 170, 'debug', true);
%   [path, reached] = ds.query([xs,ys,zs], [xg,yg,zg], 'animate', true);
%
% Copyright (C) 2026, Eudald Sangenis


%% Class
classdef DstarLite_ES < Navigation
    
    properties (SetAccess=private, GetAccess=private)
        % D* Lite specific properties
        U           % priority queue (open list)
        rhs         % right-hand side values
        g           % g-values (cost-to-come)
        km          % key modifier for incremental search
        
        % Vision system properties
        visionConeHandle    % Handle for vision cone visualization
        visionRadius        % Vision range in pixels
        visionAngle         % Vision cone angle in degrees
        
        % Navigation state
        goalLineHandle      % Handle for goal line visualization
        originalGoal        % Store the original goal
        isWallFollowing     % Flag for wall following mode
        observedOccGrid     % Observed occupancy grid
        
        % Algorithm state
        start_pos           % Start position
        goal_pos            % Goal position
        validplan           % Flag indicating if plan is valid
        changed_cells       % Cells that have changed cost
        knownCostmap        % True map for sensor simulation
        
        % Debug properties
        debug               % Debug flag
        lastStart           % Last start position for km updates
    end
    
    properties (SetAccess=private, GetAccess=public)
        niter               % Number of iterations
        costmap             % World cost map: obstacle = Inf
    end
    
    methods
        
        function ds = DstarLite_ES(world, varargin)
            % Parse options
            opt.visionRadius = 30;
            opt.visionAngle = 170;
            opt.progress = true;
            opt.debug = true;
            [opt, args] = tb_optparse(opt, varargin);
            
            % Invoke superclass constructor
            ds = ds@Navigation(world, args{:});
            
            % Initialize debug flag
            ds.debug = opt.debug;
            
            % Initialize D* Lite structures
            ds.reset();
            
            % Initialize vision system
            ds.visionRadius = opt.visionRadius;
            ds.visionAngle = opt.visionAngle;
            ds.visionConeHandle = [];
            
            % Initialize navigation state
            ds.goalLineHandle = [];
            ds.originalGoal = [];
            ds.isWallFollowing = false;
            ds.observedOccGrid = zeros(size(ds.occgridnav));
            
            % Initialize with unknown map instead of full knowledge
            ds.initializeUnknownCostmap(world);
            ds.lastStart = [];
            
            if ds.debug
                fprintf('D*-Lite initialized with unknown map of size %dx%d\n', size(world));
            end
        end
        
        function reset(ds)
            if isempty(ds.occgridnav)
                return;
            end
            
            % Initialize D* Lite data structures
            [rows, cols] = size(ds.occgridnav);
            ds.g = inf(rows, cols);
            ds.rhs = inf(rows, cols);
            ds.U = [];
            ds.km = 0;
            
            ds.validplan = false;
            ds.changed_cells = [];
            ds.niter = 0;
            ds.lastStart = [];
            
            if ds.debug
                fprintf('D*-Lite reset: g and rhs matrices initialized to inf\n');
            end
        end
              
        function s = char(ds)
            s = char@Navigation(ds);
            
            if ~isempty(ds.costmap)
                s = char(s, sprintf('  costmap: %dx%d', size(ds.costmap)));
            end
            
            if ds.validplan
                s = char(s, '  plan: valid');
            else
                s = char(s, '  plan: stale');
            end
        end
        
        function plot(ds, varargin)
            plot@Navigation(ds, varargin{:});
        end
        
        function plan(ds, goal, varargin)
            opt.progress = true;
            opt = tb_optparse(opt, varargin);
            
            if nargin > 1
                if length(goal) >= 2
                    ds.goal_pos = goal(1:2);
                    ds.setgoal(ds.goal_pos);
                end
            end
            
            if isempty(ds.goal)
                error('RTB:DstarLittle_ES:nogoal', 'No goal specified');
            end
            
            % Initialize goal in D* Lite
            [gy, gx] = deal(round(ds.goal(2)), round(ds.goal(1)));
            if gy >= 1 && gy <= size(ds.rhs,1) && gx >= 1 && gx <= size(ds.rhs,2)
                ds.rhs(gy, gx) = 0;
                ds.U = ds.insert_or_update(ds.U, [gx, gy], ds.calculate_key([gx, gy]));
                
                if ds.debug
                    fprintf('Goal set at (%d, %d), rhs=0, added to queue\n', gx, gy);
                    fprintf('Initial queue size: %d\n', size(ds.U, 1));
                end
            end
            
            ds.validplan = true;
            ds.niter = 0;
            
            if opt.progress
                fprintf('D* Lite planning initialized for unknown environment\n');
            end
        end
        
        function [pp, goalReached, exitStairsNumber] = query(ds, start, goal, varargin)
            opt.animate = false;
            opt.movie = [];
            opt.current = false;
            opt = tb_optparse(opt, varargin);
            
            goalReached = false;
            exitStairsNumber = 0;
            
            % Validate inputs
            ds.checkquery(start(1:2), goal(1:2));
            
            % Store original goal and initialize
            ds.originalGoal = goal(1:2);
            ds.start_pos = start(1:2);
            ds.goal_pos = goal(1:2);
            ds.lastStart = start(1:2);
            
            % Plan initial path
            ds.plan(goal);
            
            % Compute initial path from start to goal
            start_grid = [round(start(1)), round(start(2))];
            ds.compute_shortest_path(start_grid);
            
            if ds.debug
                fprintf('Initial path computation complete\n');
                fprintf('g(start) = %.2f, rhs(start) = %.2f\n', ...
                    ds.g(start_grid(2), start_grid(1)), ds.rhs(start_grid(2), start_grid(1)));
            end
            
            % Initialize animation if requested
            if opt.animate
                ds.plot();
                axis equal;
                hold on;
            end
        
            if ds.debug
                fprintf('Starting D*-Lite navigation from (%.1f, %.1f) to (%.1f, %.1f)\n', ...
                    start(1), start(2), goal(1), goal(2));
            end
            
            % Main navigation loop
            robot = ds.start_pos(:);
            path = robot;
            iter = 0;
            startHeight = start(3);
            goalHeight = goal(3);
            maxIter = 1000;
            
            while iter < maxIter
                iter = iter + 1;
                
                if ds.debug && mod(iter, 5) == 1
                    fprintf('Iteration %d: Robot at (%.1f, %.1f)\n', iter, robot(1), robot(2));
                end
                
                % Check if we've reached the goal
                if startHeight == goalHeight && norm(robot' - goal(1:2)) < 2.0
                    goalReached = true;
                    fprintf('Goal reached at iteration %d\n', iter);
                    break;
                end
        
                % Update vision based on current heading toward goal
                if ~isempty(ds.visionConeHandle) && isvalid(ds.visionConeHandle)
                    delete(ds.visionConeHandle);
                end


                if iter == 1
                    fprintf('=== D* LITE INITIAL PATH DEBUG ===\n');
                    
                    % Trace the actual path D* Lite computed
                    current_trace = [round(robot(1)), round(robot(2))];
                    path_trace = [];
                    
                    for trace_step = 1:20  % Trace first 20 steps
                        [cy, cx] = deal(current_trace(2), current_trace(1));
                        
                        % Find the neighbor D* Lite would choose
                        neighbors = ds.get_neighbors([cx, cy]);
                        best_cost = inf;
                        best_next = [];
                        
                        for i = 1:size(neighbors, 1)
                            nx = neighbors(i, 1); ny = neighbors(i, 2);
                            if nx >= 1 && nx <= size(ds.g,2) && ny >= 1 && ny <= size(ds.g,1)
                                edge_cost = ds.get_cost([cx, cy], [nx, ny]);
                                if edge_cost < inf && ds.g(ny, nx) < inf
                                    total_cost = ds.g(ny, nx) + edge_cost;
                                    if total_cost < best_cost
                                        best_cost = total_cost;
                                        best_next = [nx, ny];
                                    end
                                end
                            end
                        end
                        
                        if isempty(best_next)
                            break;
                        end
                        
                        path_trace = [path_trace; best_next];
                        fprintf('  Step %d: (%d,%d) -> (%d,%d)\n', trace_step, cx, cy, best_next(1), best_next(2));
                        current_trace = best_next;
                        
                        % Stop if we're getting close to goal
                        if norm(best_next - [round(ds.goal(1)), round(ds.goal(2))]) < 50
                            break;
                        end
                    end
                    fprintf('================================\n');
                end
        
%                 % Calculate heading towards goal
%                 delta = ds.goal - robot;
%                 heading = atan2(delta(2), delta(1));

                robot_next_preview = ds.next(robot);
                if ~isempty(robot_next_preview)
                    % Point vision cone toward next move direction
                    delta = robot_next_preview - robot;
                    heading = atan2(delta(2), delta(1));
                else
                    % Fallback: point toward goal if no next move available
                    delta = ds.goal - robot;
                    heading = atan2(delta(2), delta(1));
                end
                
                % Update map knowledge from current field of view
                [ds.visionConeHandle, coneVertices] = ds.plotVisionCone(robot, heading);
                obstacles_detected = ds.updateCostmapFromVision(robot, coneVertices);
     
                if ds.debug && obstacles_detected > 0
                    fprintf('  Detected %d new obstacles\n', obstacles_detected);
                end
                
                % Find next position using D*-Lite
                robot_next = ds.next(robot);
                
                if isempty(robot_next)
                    if ds.debug
                        fprintf('No valid path found at iteration %d\n', iter);
                        ds.debugPathfinding(robot);
                    end
                    break;
                end
                
                % Check if robot is stuck
                if norm(robot_next - robot) < 0.1
                    if ds.debug
                        fprintf('Robot stuck at iteration %d\n', iter);
                    end
                    break;
                end
                
                % Update robot position
                robot = robot_next;
                path = [path, robot];
                
                % Visualization
                if opt.animate
                    plot(robot(1), robot(2), 'g.', 'MarkerSize', 12);
                    drawnow;
                    pause(0.01);
                end
            end
            
            if iter >= maxIter
                fprintf('Maximum iterations reached (%d)\n', maxIter);
            end
            
            % Return path
            if nargout > 0
                pp = path';
            end
        end
        
%         function n = next(ds, current)
%             if ~ds.validplan
%                 error('RTB:DstarLittle_ES:noplan', 'No valid plan, call plan() first');
%             end
%             
%             % Convert to grid coordinates
%             [cy, cx] = deal(round(current(2)), round(current(1)));
%             
%             % Check bounds
%             if cy < 1 || cy > size(ds.g,1) || cx < 1 || cx > size(ds.g,2)
%                 if ds.debug
%                     fprintf('Current position out of bounds: (%d, %d)\n', cx, cy);
%                 end
%                 n = [];
%                 return;
%             end
% 
%             % Run D* Lite computation
%             ds.compute_shortest_path([cx, cy]);
%             
%             % Check if path exists
%             if ds.g(cy, cx) == inf
%                 if ds.debug
%                     fprintf('No path exists from current position\n');
%                 end
%                 n = [];
%                 return;
%             end
%             
%             % CORRECTED: Find best neighbor using proper D*-Lite selection
%             neighbors = ds.get_neighbors([cx, cy]);
%             best_total_cost = inf;
%             best_neighbor = [];
%             
%             for i = 1:size(neighbors, 1)
%                 nx = neighbors(i, 1);
%                 ny = neighbors(i, 2);
%                 
%                 if nx >= 1 && nx <= size(ds.g,2) && ny >= 1 && ny <= size(ds.g,1)
%                     edge_cost = ds.get_cost([cx, cy], [nx, ny]);
%                     
%                     if edge_cost < inf && ds.g(ny, nx) < inf  % Valid move and reachable
%                         
%                         % Check if the next position is actually free in the real world
%                         if ds.isoccupied([nx; ny])
%                             % This cell is actually occupied! Update costmap and replan
%                             ds.costmap(ny, nx) = inf;
%                             ds.changed_cells = [ds.changed_cells; nx, ny];
%                             ds.handleCostChanges(current);
%                             continue; % Skip this neighbor
%                         end
% 
%                         total_cost = ds.g(ny, nx) + edge_cost;
%                         
%                         if total_cost < best_total_cost
%                             best_total_cost = total_cost;
%                             best_neighbor = [nx, ny];
%                         end
%                     end
%                 end
%             end
%             
%             if isempty(best_neighbor)
%                 if ds.debug
%                     fprintf('No valid neighbor found\n');
%                 end
%                 n = [];
%             else
%                 if ds.debug
%                     fprintf('  Next: (%d, %d) with g=%.2f + edge=%.2f = total=%.2f\n', ...
%                         best_neighbor(1), best_neighbor(2), ds.g(best_neighbor(2), best_neighbor(1)), ...
%                         ds.get_cost([cx, cy], best_neighbor), best_total_cost);
%                 end
%                 n = [best_neighbor(1); best_neighbor(2)];
%             end
%         end

        function n = next(ds, current)
            if ~ds.validplan
                error('RTB:DstarLittle_ES:noplan', 'No valid plan, call plan() first');
            end
            
            % Convert to grid coordinates
            [cy, cx] = deal(round(current(2)), round(current(1)));
            
            % Check bounds
            if cy < 1 || cy > size(ds.g,1) || cx < 1 || cx > size(ds.g,2)
                if ds.debug
                    fprintf('Current position out of bounds: (%d, %d)\n', cx, cy);
                end
                n = [];
                return;
            end
            
            % REMOVED: ds.updateLocalObstacles(current); - this method doesn't exist
            
            % Run D* Lite computation
            ds.compute_shortest_path([cx, cy]);
            
            % Check if path exists
            if ds.g(cy, cx) == inf
                if ds.debug
                    fprintf('No path exists from current position\n');
                end
                n = [];
                return;
            end
            
            % Find best neighbor using proper D*-Lite selection
            neighbors = ds.get_neighbors([cx, cy]);
            best_total_cost = inf;
            best_neighbor = [];
            
            for i = 1:size(neighbors, 1)
                nx = neighbors(i, 1);
                ny = neighbors(i, 2);
                
                if nx >= 1 && nx <= size(ds.g,2) && ny >= 1 && ny <= size(ds.g,1)
                    edge_cost = ds.get_cost([cx, cy], [nx, ny]);
                    
                    if edge_cost < inf && ds.g(ny, nx) < inf  % Valid move and reachable
                        
                        % Check if the next position is actually free in the real world
                        if ds.isoccupied([nx; ny])
                            % This cell is actually occupied! Update costmap and replan
                            ds.costmap(ny, nx) = inf;
                            ds.changed_cells = [ds.changed_cells; nx, ny];
                            ds.handleCostChanges(current);
                            continue; % Skip this neighbor
                        end
        
                        total_cost = ds.g(ny, nx) + edge_cost;
                        
                        if total_cost < best_total_cost
                            best_total_cost = total_cost;
                            best_neighbor = [nx, ny];
                        end
                    end
                end
            end
            
            if isempty(best_neighbor)
                if ds.debug
                    fprintf('No valid neighbor found\n');
                end
                n = [];
            else
                if ds.debug
                    fprintf('  Next: (%d, %d) with g=%.2f + edge=%.2f = total=%.2f\n', ...
                        best_neighbor(1), best_neighbor(2), ds.g(best_neighbor(2), best_neighbor(1)), ...
                        ds.get_cost([cx, cy], best_neighbor), best_total_cost);
                end
                n = [best_neighbor(1); best_neighbor(2)];
            end
        end
             
        function [handle, coneVertices] = plotVisionCone(ds, robot, heading)
            visionAngle_rad = ds.visionAngle * pi/180;
            visionRadius_pix = ds.visionRadius * 4; % Convert meters to pixels
            expansionDistance = 1; % Distance to expand the cone
            
            numPoints = 100;
            theta = linspace(-visionAngle_rad/2, visionAngle_rad/2, numPoints) + heading;
            
            % Ray casting for realistic vision (FIXED: Use same method as working example)
            [x, y] = ds.rayCast(robot, theta, visionRadius_pix);
            
            % Expand the cone uniformly in all directions
            expandedX = x + expansionDistance * cos(theta);
            expandedY = y + expansionDistance * sin(theta);
            
            % Include the robot position in the cone vertices
            coneVertices = [robot(1), expandedX; robot(2), expandedY];
            
            % Create a closed polygon
            closedX = [robot(1), expandedX, robot(1)];
            closedY = [robot(2), expandedY, robot(2)];
            
            % Plot the vision cone
            handle = fill(closedX, closedY, 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
        end
        
        function [x, y] = rayCast(ds, start, angles, maxDist)
            x = zeros(1, length(angles));
            y = zeros(1, length(angles));
            
            for i = 1:length(angles)
                [x(i), y(i)] = ds.castSingleRay(start, angles(i), maxDist);
            end
        end
        
        function [endX, endY] = castSingleRay(ds, start, angle, maxDist)
            % FIXED: Use the same method as your working example
            dx = cos(angle);
            dy = sin(angle);
            
            for dist = 1:maxDist
                checkX = round(start(1) + dist * dx);
                checkY = round(start(2) + dist * dy);
                
                % Check if the point is within the grid
                if checkX < 1 || checkY < 1 || checkX > size(ds.occgridnav, 2) || checkY > size(ds.occgridnav, 1)
                    break;
                end
                
                % Check if the point is occupied (SAME AS YOUR WORKING EXAMPLE)
                if ds.isoccupied([checkX; checkY])
                    break;
                end
            end
            
            endX = start(1) + (dist - 1) * dx;
            endY = start(2) + (dist - 1) * dy;
        end
          
        function obstacles_detected = updateCostmapFromVision(ds, robot, coneVertices)
            [rows, cols] = size(ds.costmap);
            MAX_OBSTACLES_PER_ITERATION = 5; % Limit processing
            obstacles_detected = 0;
            new_obstacles = []; % Collect all visible obstacles first 
            for i = 1:size(coneVertices, 2) 
                x = round(coneVertices(1, i));
                y = round(coneVertices(2, i));
                if x >= 1 && x <= cols && y >= 1 && y <= rows
                    if ds.isVisibleFromRobot(robot, [x, y])
                        ds.observedOccGrid(y, x) = 1;
                        oldCost = ds.costmap(y, x);
                        if ds.isoccupied([x; y])
                            newCost = inf;
                        else
                            newCost = 1;
                        end
                        if oldCost ~= newCost && newCost == inf
                            new_obstacles = [new_obstacles; x, y];
                        end
                    end
                end
            end % Process only closest obstacles first 
            if ~isempty(new_obstacles)
                distances = sqrt(sum((new_obstacles - robot(1:2)').^2, 2));
                [~, idx] = sort(distances); % Process only the closest obstacles 
                process_count = min(MAX_OBSTACLES_PER_ITERATION, size(new_obstacles, 1));
                for i = 1:process_count
                    obstacle_idx = idx(i);
                    x = new_obstacles(obstacle_idx, 1);
                    y = new_obstacles(obstacle_idx, 2);
                    ds.costmap(y, x) = inf;
                    ds.changed_cells = [ds.changed_cells; x, y];
                    obstacles_detected = obstacles_detected + 1;

                    % DEBUG OBSTACLE DETECTION
                    plot(x, y, 'bo', 'MarkerSize', 2, 'MarkerFaceColor', 'b');
                    drawnow;
                end
            end
            if obstacles_detected > 0 
                ds.handleCostChanges(robot); 
            end 
        end

        function visible = isVisibleFromRobot(ds, robot, target)
            x0 = round(robot(1)); y0 = round(robot(2));
            x1 = target(1); y1 = target(2);
            
            % Get line points using Bresenham's algorithm
            [x_line, y_line] = ds.bresenhamLine(x0, y0, x1, y1);
            
            % Check each point along the line for obstacles
            for i = 1:length(x_line)-1  % Don't check the target itself
                x = x_line(i); y = y_line(i);
                
                % Check bounds
                if x < 1 || x > size(ds.costmap, 2) || y < 1 || y > size(ds.costmap, 1)
                    visible = false;
                    return;
                end
                
                % If we've observed this cell and it's an obstacle, vision is blocked
                if ds.observedOccGrid(y, x) == 1 && ds.costmap(y, x) == inf
                    visible = false;
                    return;
                end
            end
            
            visible = true;
        end
        
        function handleCostChanges(ds, current_robot_pos)
            if isempty(ds.changed_cells)
                return;
            end
            
            if ds.debug
                fprintf('  Handling %d cost changes\n', size(ds.changed_cells, 1));
            end

            if ~isempty(ds.lastStart) && ~isempty(ds.goal)
                h_old = ds.heuristic(ds.lastStart, [round(ds.goal(1)), round(ds.goal(2))]);
                ds.km = ds.km + h_old;
                ds.lastStart = [round(current_robot_pos(1)), round(current_robot_pos(2))]; % Update last start
            end

%             % Simple km increment when obstacles are discovered
%             ds.km = ds.km + 1;  % Much simpler than heuristic calculations

            % Update all affected vertices
            for i = 1:size(ds.changed_cells, 1)
                cell = ds.changed_cells(i, :);
                ds.update_vertex(cell);
                
                % Update all neighbors of changed cells
                neighbors = ds.get_neighbors(cell);
                for j = 1:size(neighbors, 1)
                    ds.update_vertex(neighbors(j, :));
                end
            end
            
            % Clear changed cells after processing
            ds.changed_cells = [];
        end

        function [x_line, y_line] = bresenhamLine(~, x0, y0, x1, y1)
            dx = abs(x1 - x0);
            dy = abs(y1 - y0);
            
            if x0 < x1
                sx = 1;
            else
                sx = -1;
            end
            
            if y0 < y1
                sy = 1;
            else
                sy = -1;
            end
            
            err = dx - dy;
            x = x0; y = y0;
            
            x_line = []; y_line = [];
            
            while true
                x_line = [x_line, x];
                y_line = [y_line, y];
                
                if x == x1 && y == y1
                    break;
                end
                
                e2 = 2 * err;
                
                if e2 > -dy
                    err = err - dy;
                    x = x + sx;
                end
                
                if e2 < dx
                    err = err + dx;
                    y = y + sy;
                end
            end
        end

        function debugPathfinding(ds, current)
            [cy, cx] = deal(round(current(2)), round(current(1)));
            [gy, gx] = deal(round(ds.goal(2)), round(ds.goal(1)));
            
            fprintf('=== DEBUG PATHFINDING ===\n');
            fprintf('Current: (%d, %d), Goal: (%d, %d)\n', cx, cy, gx, gy);
            fprintf('g(current) = %.2f, rhs(current) = %.2f\n', ds.g(cy, cx), ds.rhs(cy, cx));
            fprintf('g(goal) = %.2f, rhs(goal) = %.2f\n', ds.g(gy, gx), ds.rhs(gy, gx));
            fprintf('Queue size: %d, km = %.2f\n', size(ds.U, 1), ds.km);
            
            % Check a few neighbors
            neighbors = ds.get_neighbors([cx, cy]);
            fprintf('Sample neighbors:\n');
            for i = 1:min(5, size(neighbors, 1))
                nx = neighbors(i, 1);
                ny = neighbors(i, 2);
                if nx >= 1 && nx <= size(ds.g,2) && ny >= 1 && ny <= size(ds.g,1)
                    edge_cost = ds.get_cost([cx, cy], [nx, ny]);
                    fprintf('  (%d,%d): g=%.2f, edge_cost=%.2f\n', ...
                        nx, ny, ds.g(ny, nx), edge_cost);
                end
            end
            fprintf('========================\n');
        end
    
    end % public methods
    
    methods (Access=protected)
        
        function initializeUnknownCostmap(ds, world)
            FREE_COST = 1;
            % REMOVE THIS LINE - it's causing the problem!
            % UNKNOWN_COST = 2;  
            
            % Start with optimistic free space assumption
            ds.costmap = ones(size(world)) * FREE_COST;
            
            % Store the true map for sensor simulation
            ds.knownCostmap = world;
            ds.observedOccGrid = zeros(size(world));
            
            if ds.debug
                [rows, cols] = size(world);
                fprintf('Unknown costmap initialized: %dx%d, all cells cost %d\n', rows, cols, FREE_COST);
            end
        end

        function occgrid2costmap(ds, og, cost)
            if nargin < 3
                cost = 1;
            end
            % Override parent method
        end
        
        % D* Lite algorithm implementation
        
        function key = calculate_key(ds, s)
            [sy, sx] = deal(s(2), s(1));
            if sy >= 1 && sy <= size(ds.g,1) && sx >= 1 && sx <= size(ds.g,2)
                g_val = ds.g(sy, sx);
                rhs_val = ds.rhs(sy, sx);
                h_val = ds.heuristic(s, [round(ds.goal(1)), round(ds.goal(2))]);
                key = [min(g_val, rhs_val) + h_val + ds.km; min(g_val, rhs_val)];
            else
                key = [inf; inf];
            end
        end

%         function h = heuristic(ds, s1, s2)
%             % Standard Euclidean distance
%             dx = s1(1) - s2(1);
%             dy = s1(2) - s2(2);
% %             h = sqrt(dx^2 + dy^2);
%             h = (dx + dy) + (sqrt(2) - 2)*min(dx,dy);
%             
%             % STRONGER tie-breaking bias toward diagonal movement
%             cross = abs(dx * (ds.goal(2) - s2(2)) - dy * (ds.goal(1) - s2(1)));
%             h = h + cross * 0.01;  % INCREASED from 0.001 to 0.01
%             
%             % Additional diagonal preference
%             if abs(dx) > 0 && abs(dy) > 0  % If this would be a diagonal move
%                 h = h - 0.1;  % Small bonus for diagonal moves
%             end
%         end
        function h = heuristic(ds, s1, s2)
            dx = abs(s1(1) - s2(1));
            dy = abs(s1(2) - s2(2));
            
            % Standard octile distance (appropriate for 8-connected grid)
            h = max(dx, dy) + (sqrt(2) - 1) * min(dx, dy);
            
            % Minimal tie-breaking (much smaller coefficient)
            cross = abs(dx * (ds.goal(2) - s2(2)) - dy * (ds.goal(1) - s2(1)));
            h = h + cross * 0.001;  % Very small tie-breaker
        end
        
        function U_new = insert_or_update(ds, U, s, key)
            U_new = U;
            
            % Remove existing entry if present
            if ~isempty(U_new)
                for i = size(U_new, 1):-1:1
                    if isequal(U_new(i, 1:2), s)
                        U_new(i, :) = [];
                        break;
                    end
                end
            end
            
            % Insert new entry
            new_entry = [s, key'];
            U_new = [U_new; new_entry];
            
            % Sort by key
            if size(U_new, 1) > 1
                [~, idx] = sortrows(U_new(:, 3:4));
                U_new = U_new(idx, :);
            end
        end
        
        function [s, U_new] = top_key_and_pop(ds, U)
            if isempty(U)
                s = [];
                U_new = U;
                return;
            end
            
            s = U(1, 1:2);
            U_new = U(2:end, :);
        end
        
        function neighbors = get_neighbors(ds, s)
            [sx, sy] = deal(s(1), s(2));
            
            % PRIORITIZE DIAGONAL MOVES TOWARD GOAL
            goal_x = round(ds.goal(1));
            goal_y = round(ds.goal(2));
            
            % Calculate direction to goal
            dx_to_goal = sign(goal_x - sx);
            dy_to_goal = sign(goal_y - sy);
            
            % Start with the diagonal move toward goal
            preferred_moves = [];
            if dx_to_goal ~= 0 && dy_to_goal ~= 0
                preferred_moves = [dx_to_goal, dy_to_goal];  % Diagonal toward goal
            end
            
            % Add all other moves
            all_moves = [];
            for dx = -1:1
                for dy = -1:1
                    if dx == 0 && dy == 0
                        continue;
                    end
                    % Skip if this is already the preferred move
                    if ~isempty(preferred_moves) && dx == preferred_moves(1) && dy == preferred_moves(2)
                        continue;
                    end
                    all_moves = [all_moves; dx, dy];
                end
            end
            
            % Combine: preferred move first, then others
            if ~isempty(preferred_moves)
                move_order = [preferred_moves; all_moves];
            else
                move_order = all_moves;
            end
            
            neighbors = [];
            for i = 1:size(move_order, 1)
                dx = move_order(i, 1);
                dy = move_order(i, 2);
                
                nx = sx + dx;
                ny = sy + dy;
                
                if nx >= 1 && nx <= size(ds.costmap, 2) && ny >= 1 && ny <= size(ds.costmap, 1)
                    neighbors = [neighbors; nx, ny];
                end
            end
        end
        
        function cost = get_cost(ds, s1, s2)
            [s1y, s1x] = deal(s1(2), s1(1));
            [s2y, s2x] = deal(s2(2), s2(1));
            
            if s1y >= 1 && s1y <= size(ds.costmap,1) && s1x >= 1 && s1x <= size(ds.costmap,2) && ...
               s2y >= 1 && s2y <= size(ds.costmap,1) && s2x >= 1 && s2x <= size(ds.costmap,2)
                
                if ds.costmap(s1y, s1x) == inf || ds.costmap(s2y, s2x) == inf
                    cost = inf;
                else
                    % Calculate base Euclidean distance
                    base_cost = sqrt((s1x - s2x)^2 + (s1y - s2y)^2);
                    
                    % Check if this is a diagonal move toward the goal
                    dx = s2x - s1x;  % Direction of move
                    dy = s2y - s1y;
                    
                    % Direction to goal from s1
                    goal_x = round(ds.goal(1));
                    goal_y = round(ds.goal(2));
                    goal_dx = sign(goal_x - s1x);
                    goal_dy = sign(goal_y - s1y);
                    
                    % If this is a diagonal move AND it's toward the goal, reduce cost
                    if abs(dx) == 1 && abs(dy) == 1 && dx == goal_dx && dy == goal_dy
                        cost = 1.0;  % Make diagonal toward goal same cost as straight!
                    else
                        cost = base_cost;  % Normal cost
                    end
                end
            else
                cost = inf;
            end
        end
        
        function update_vertex(ds, u)
            [uy, ux] = deal(u(2), u(1));
            
            if uy < 1 || uy > size(ds.g,1) || ux < 1 || ux > size(ds.g,2)
                return;
            end
            
            % Check if u is not the goal
            goal_x = round(ds.goal(1));
            goal_y = round(ds.goal(2));
            
            if ~(ux == goal_x && uy == goal_y)
                neighbors = ds.get_neighbors(u);
                min_rhs = inf;
                
                for i = 1:size(neighbors, 1)
                    neighbor = neighbors(i, :);
                    [ny, nx] = deal(neighbor(2), neighbor(1));
                    
                    if ny >= 1 && ny <= size(ds.g,1) && nx >= 1 && nx <= size(ds.g,2)
                        cost = ds.get_cost(u, neighbor) + ds.g(ny, nx);
                        min_rhs = min(min_rhs, cost);
                    end
                end
                
                ds.rhs(uy, ux) = min_rhs;
            end
            
            % Remove u from U if present
            if ~isempty(ds.U)
                for i = size(ds.U, 1):-1:1
                    if isequal(ds.U(i, 1:2), u)
                        ds.U(i, :) = [];
                        break;
                    end
                end
            end
            
            % Insert u into U if g(u) != rhs(u)
            if ds.g(uy, ux) ~= ds.rhs(uy, ux)
                key = ds.calculate_key(u);
                ds.U = ds.insert_or_update(ds.U, u, key);
            end
        end
        
%         function compute_shortest_path(ds, start)
%             if isempty(ds.goal)
%                 return;
%             end
%             
%             max_iterations = 50000;  % Increase limit
%             iteration_count = 0;
%             
%             % Get start key for comparison
%             start_key = ds.calculate_key(start);
%             
%             while ~isempty(ds.U) && iteration_count < max_iterations
%                 
%                 % Check termination conditions (from D*-Lite literature)
%                 top_key = ds.U(1, 3:4)';
%                 start_key = ds.calculate_key(start);
% 
% %                 if ds.compare_keys(top_key, start_key) || ...
% %                    ds.g(start(2), start(1)) == ds.rhs(start(2), start(1))
% %                     break;
% %                 end
%                 if ~ds.compare_keys(top_key, start_key) && ...
%                        ds.g(start(2), start(1)) == ds.rhs(start(2), start(1))
%                     break;  % Termination condition met
%                 end
%                 
%                 ds.niter = ds.niter + 1;
%                 iteration_count = iteration_count + 1;
%                 
%                 [u, ds.U] = ds.top_key_and_pop(ds.U);
%                 [uy, ux] = deal(u(2), u(1));
%                 
%                 if uy < 1 || uy > size(ds.g,1) || ux < 1 || ux > size(ds.g,2)
%                     continue;
%                 end
%                 
%                 % Debug output every 1000 iterations
%                 if ds.debug && mod(iteration_count, 1000) == 0
%                     fprintf('Compute iteration %d: processing (%d,%d), g=%.2f, rhs=%.2f\n', ...
%                         iteration_count, ux, uy, ds.g(uy, ux), ds.rhs(uy, ux));
%                 end
%                 
%                 if ds.g(uy, ux) > ds.rhs(uy, ux)
%                     ds.g(uy, ux) = ds.rhs(uy, ux);
%                     neighbors = ds.get_neighbors(u);
%                     
%                     for i = 1:size(neighbors, 1)
%                         ds.update_vertex(neighbors(i, :));
%                     end
%                 else
%                     ds.g(uy, ux) = inf;
%                     neighbors = [ds.get_neighbors(u); u];
%                     
%                     for i = 1:size(neighbors, 1)
%                         ds.update_vertex(neighbors(i, :));
%                     end
%                 end
%                 
%                 % Recalculate start key for next iteration
%                 start_key = ds.calculate_key(start);
%             end
% 
%             if iteration_count == 1 && ds.debug
%                 fprintf('=== INITIAL COMPUTATION DEBUG ===\n');
%                 fprintf('Start: (%d,%d), Goal: (%d,%d)\n', start(1), start(2), round(ds.goal(1)), round(ds.goal(2)));
%                 
%                 % Check the first few nodes being processed
%                 for i = 1:min(10, size(ds.U,1))
%                     node = ds.U(i, 1:2);
%                     key = ds.U(i, 3:4);
%                     fprintf('Queue[%d]: (%d,%d) key=[%.2f, %.2f]\n', i, node(1), node(2), key(1), key(2));
%                 end
%                 fprintf('==================================\n');
%             end
% 
%             
%             if iteration_count >= max_iterations
%                 fprintf('Warning: compute_shortest_path reached maximum iterations (%d)\n', max_iterations);
%                 fprintf('Queue size: %d, g(start)=%.2f, rhs(start)=%.2f\n', ...
%                     size(ds.U, 1), ds.g(start(2), start(1)), ds.rhs(start(2), start(1)));
%             else
%                 if ds.debug
%                     fprintf('Compute_shortest_path converged after %d iterations\n', iteration_count);
%                 end
%             end
%         end

        function compute_shortest_path(ds, start)
            if isempty(ds.goal)
                return;
            end
            
            max_iterations = 50000;
            iteration_count = 0;
            
            while ~isempty(ds.U) && iteration_count < max_iterations
                iteration_count = iteration_count + 1;
                
                % Get current keys
                top_key = ds.U(1, 3:4)';
                start_key = ds.calculate_key(start);
                
                % CORRECTED termination condition from D* Lite paper
                if (~ds.compare_keys(top_key, start_key)) && ...
                   (ds.g(start(2), start(1)) == ds.rhs(start(2), start(1)))
                    break;
                end
                
                [u, ds.U] = ds.top_key_and_pop(ds.U);
                [uy, ux] = deal(u(2), u(1));
                
                % Bounds check
                if uy < 1 || uy > size(ds.g,1) || ux < 1 || ux > size(ds.g,2)
                    continue;
                end
                
                % Standard D* Lite vertex expansion
                if ds.g(uy, ux) > ds.rhs(uy, ux)
                    ds.g(uy, ux) = ds.rhs(uy, ux);
                    neighbors = ds.get_neighbors(u);
                    for i = 1:size(neighbors, 1)
                        ds.update_vertex(neighbors(i, :));
                    end
                else
                    ds.g(uy, ux) = inf;
                    all_vertices = [ds.get_neighbors(u); u];  % Include u itself
                    for i = 1:size(all_vertices, 1)
                        ds.update_vertex(all_vertices(i, :));
                    end
                end
            end
            
            if ds.debug && iteration_count < 10
                fprintf('Compute_shortest_path converged after %d iterations\n', iteration_count);
            end
        end
        
%         function result = compare_keys(~, key1, key2)
%             if key1(1) < key2(1)
%                 result = true;
%             elseif key1(1) == key2(1) && key1(2) < key2(2)
%                 result = true;
%             else
%                 result = false;
%             end
%         end
        function result = compare_keys(~, key1, key2)
            % Returns true if key1 < key2 (lexicographically)
            if key1(1) < key2(1) || (key1(1) == key2(1) && key1(2) < key2(2))
                result = true;
            else
                result = false;
            end
        end
        
    end % protected methods
    
end % classdef
