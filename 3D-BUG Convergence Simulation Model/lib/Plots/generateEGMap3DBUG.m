function [maps, figHandle] = generateEGMap3DBUG()
    % generateEGMap3DBUG Generates the 3D map of the Engineering 
    % Gateway Building based on a gridmap structure (containing the
    % emergency signs and exits information) and returns a maps container.
    % Output:
    % maps.grid.floor#
    % maps.emergencyStairs.floor#
    % maps.emergencySigns.floor#.location, maps.emergencySigns.floor#.direction
    
    clear all; close all; clc;
    
    %% Libraries
    currentFolder = pwd;
    addpath(genpath('C:\Users\Eudald\Documents\Git Repos\PeterCork_PathPlanning_Lib\rtb'));
    addpath(genpath('C:\Users\Eudald\Documents\Git Repos\PeterCork_PathPlanning_Lib\common'));
    addpath(genpath('C:\Users\Eudald\Documents\Git Repos\PeterCork_PathPlanning_Lib\smtb'));
    addpath(genpath([currentFolder, '\MAE_Occupancy_Map']));
    addpath(genpath([currentFolder, '\lib\Plots']));
    
    %% 3D Plot
    figHandle = figure('Name', '3D Trajectory MAE UCI'); 
    title('3D Trajectory MAE UCI', 'Interpreter', 'latex', 'FontSize', 14);
    hold on; grid on; axis equal;
    xlabel('X, [px]', 'Interpreter', 'latex', 'FontSize', 14); 
    ylabel('Y, [px]', 'Interpreter', 'latex', 'FontSize', 14); 
    zlabel('Z, [px]', 'Interpreter', 'latex', 'FontSize', 14);
    set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 14);
    
    campos([-1131.6   -0727.6    1601.3]);
    camtarget([61.5000  181.0000  100.0000]);
    camup([0, 0, 1]);
    view(-52.7112, 45.0308);
    xlim([8, 115]); ylim([0, 365]);
    
    zticks([0, 100, 200]);
    zticklabels({'2nd Floor', '3rd Floor', '4th Floor'});
    
    %% Load and Plot Maps
    z = 0;
    maps = struct('grid', struct(), 'emergencyStairs', struct(), 'emergencySigns', struct());
    
    for floorNumber = 2:4
        typeOfMap = num2str(floorNumber) + "Floor";
        mapName = "EG-" + num2str(floorNumber) + "Floor_ply.mat";
        loadedMap = load(mapName);
    
        switch typeOfMap
            case '2Floor'
                % Map Translation
                xTranslationDistance = 0; yTranslationDistance = 0;
                % Emergency Exits
                y = 135;   x = 95;
                y1 = 349;  x1 = 76; % 76 
                y2 = 9;    x2 = 76;
                % Emergency Signs
                xx1 = 86; yy1 = 29; direction1 = "back";
%                 xx2 = 59; yy2 = 31; direction2 = "right";
                xx3 = 59; yy3 = 110; direction3 = "right";  
                xx4 = 82; yy4 = 130; direction4 = "front";  
                xx5 = 90; yy5 = 135; direction5 = "right";  % xx5 = 90; yy5 = 135;
                xx6 = 84; yy6 = 349; direction6 = "left";  
                xx7 = 80; yy7 = 54; direction7 = "back";
                xx8 = 81; yy8 = 9; direction8 = "left";
                xx9 = 84; yy9 = 185;  direction9 = "back"; 
    
            case '3Floor'
                % Map Translation
                xTranslationDistance = 39; yTranslationDistance = 0;
                % Emergency Exits
                y = 135;  x = 94; 
                y1 = 349; x1 = 75; % 76
                y2 = 10;   x2 = 76; % y2 = 11;   x2 = 76;

                % Emergency Signs
                xx1 = 81; yy1 = 11; direction1 = "left";    % 82 9 
                xx2 = 81; yy2 = 42; direction2 = "back";    % 86 42
                xx3 = 86; yy3 = 130; direction3 = "front"; 
                xx4 = 90; yy4 = 135; direction4 = "right";  % xx5 = 90; yy5 = 135;
                xx5 = 84; yy5 = 349; direction5 = "left";
                xx6 = 84;  yy6 = 170;  direction6 = "back"; 
    
            case '4Floor'
                % Map Translation
                xTranslationDistance = 37; yTranslationDistance = 0;
                % Emergency Exits
                y = 135;  x = 94;
                y1 = 349; x1 = 75; % 76
                y2 = 10;   x2 = 76; % y2 = 11;   x2 = 76;

                % Emergency Signs
                xx1 = 90;  yy1 = 135; direction1 = "right"; 
                xx2 = 84;  yy2 = 349; direction2 = "left";
                xx3 = 81;  yy3 = 11;   direction3 = "left";
                xx4 = 78;  yy4 = 130;  direction4 = "front"; % xx4 = 74;  yy4 = 130;  direction4 = "front";
                xx5 = 81;  yy5 = 42;  direction5 = "back"; % 84 42
                xx6 = 84;  yy6 = 170;  direction6 = "back"; 
    
            otherwise
                error('Invalid map type selected.');
        end
    
        % Map Translations
        translatedGrid = translateGrid(loadedMap.occupancyGrid, xTranslationDistance, yTranslationDistance);
        maps.grid.(sprintf('floor%d', floorNumber)) = translatedGrid;
    
        plotOccupancyGrid3D(translatedGrid, [z, z]);
    
        % Emergency Stairs:
        stairs = [];
        if exist('x', 'var') && exist('y', 'var')
            stairs = [stairs, [x; y]];
%             plot3(x, y, ones(length(x),1)*z,'r-','LineWidth',5);
            scatter3(x, y, z, 'filled', 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'red');
            clear x y
        end
        if exist('x1', 'var') && exist('y1', 'var')
            stairs = [stairs, [x1; y1]];
%             plot3(x1, y1, ones(length(x1),1)*z,'r-','LineWidth',5);
            scatter3(x1, y1, z, 'filled', 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'red');
            clear x1 y1
        end
        if exist('x2', 'var') && exist('y2', 'var')
            stairs = [stairs, [x2; y2]];
%             plot3(x2, y2, ones(length(x2),1)*z,'r-','LineWidth',5);
            scatter3(x2, y2, z, 'filled', 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'red');
            clear x2 y2
        end
        maps.emergencyStairs.(sprintf('floor%d', floorNumber)) = stairs;
    
        % Emergency Signs:
        signs = [];
        directions = [];
        for i = 1:9
            xx_var = sprintf('xx%d', i);
            yy_var = sprintf('yy%d', i);
            direction_var = sprintf('direction%d', i);
    
            if exist(xx_var, 'var') && exist(yy_var, 'var')
                xx = eval(xx_var);
                yy = eval(yy_var);
                direction = eval(direction_var);
    
                signs = [signs, [xx; yy]];
                directions = [directions, direction];
    
                color = getColorByDirection(direction);
                scatter3(xx, yy, z, 'filled', 'MarkerFaceColor', color, 'MarkerEdgeColor', color);
    
                clear(xx_var, yy_var, direction_var);
            end
        end
        maps.emergencySigns.(sprintf('floor%d', floorNumber)).location = signs;
        maps.emergencySigns.(sprintf('floor%d', floorNumber)).direction = directions;
    
        z = z + 100;
    end

end

function translatedGrid = translateGrid(occupancyGrid, xTranslation, yTranslation)
    translatedGrid = zeros(size(occupancyGrid));
    translatedGrid(yTranslation+1:end, xTranslation+1:end) = ...
        occupancyGrid(1:end-yTranslation, 1:end-xTranslation);
end

function color = getColorByDirection(direction)
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