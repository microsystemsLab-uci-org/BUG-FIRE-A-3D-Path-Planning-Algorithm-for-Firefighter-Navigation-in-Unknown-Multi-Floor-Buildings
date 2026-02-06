%% File Header
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description:
%   This script plots the Monte Carlo Simulation Results of the BUGFIRE,
%   Random Exploration + D*-Lite, and Wall-Following Exploration + D*-Lite
%   performances with different vision Radius.
%
%   Author: Eudald Sangenis Rafart
%   Affiliation: University of California, Irvine
%   Email: esangeni@uci.edu
%   Date: Feb. 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc; warning off;
set(0, 'defaultTextInterpreter', 'latex');
set(0, 'defaultAxesTickLabelInterpreter', 'latex');
set(0, 'defaultLegendInterpreter', 'latex');

%% Import all the datasets with the performance of the BUG3 algorithm with
% different radius parameters

% BUG-FIRE:
dataset_type = "BUGFIRE"; 
t_r30 = readtable('Monte Carlo Simulation Results\BUGFIRE\250iterResults_v170_d30_2025-10-08_17-45-54.xlsx'); t_r30 = t_r30(1:250,:);
t_r10 = readtable('Monte Carlo Simulation Results\BUGFIRE\250iterResults_v170_d10_2025-10-08_17-51-35.xlsx'); t_r10 = t_r10(1:250,:);
t_r5 = readtable('Monte Carlo Simulation Results\BUGFIRE\250iterResults_v170_d5_2025-10-08_17-52-34.xlsx'); t_r5 = t_r5(1:250,:);
t_r1 = readtable('Monte Carlo Simulation Results\BUGFIRE\250iterResults_v170_d1_2025-10-08_17-56-56.xlsx'); t_r1 = t_r1(1:250,:);
t_r05 = readtable('Monte Carlo Simulation Results\BUGFIRE\250iterResults_v170_d0.5_2025-10-08_18-03-53.xlsx'); t_r05 = t_r05(1:250,:);

% Random Emergency Stairs Exploration + D*-Lite
% dataset_type = "random-d"; 
% t_r30 = readtable('Monte Carlo Simulation Results\Random_Exp_and_DstarLite\250iterResults_v170_d30_2025-10-09_10-26-37.xlsx'); t_r30 = t_r30(1:250,:);
% t_r10 = readtable('Monte Carlo Simulation Results\Random_Exp_and_DstarLite\250iterResults_v170_d10_2025-10-09_10-28-48.xlsx'); t_r10 = t_r10(1:250,:);
% t_r5 = readtable('Monte Carlo Simulation Results\Random_Exp_and_DstarLite\250iterResults_v170_d5_2025-10-09_10-28-23.xlsx'); t_r5 = t_r5(1:250,:);
% t_r1 = readtable('Monte Carlo Simulation Results\Random_Exp_and_DstarLite\250iterResults_v170_d1_2025-10-09_10-27-57.xlsx'); t_r1 = t_r1(1:250,:);
% t_r05 = readtable('Monte Carlo Simulation Results\Random_Exp_and_DstarLite\250iterResults_v170_d0.5_2025-10-09_10-27-24.xlsx'); t_r05 = t_r05(1:250,:);

% Wall Emergency Stairs Exploration + D*-Lite
% dataset_type = "wall-d"; 
% t_r30 = readtable('Monte Carlo Simulation Results\Wall_Exp_and_DstarLite\250iterResults_v170_d30_2025-10-08_15-37-13.xlsx'); t_r30 = t_r30(1:250,:);
% t_r10 = readtable('Monte Carlo Simulation Results\Wall_Exp_and_DstarLite\250iterResults_v170_d10_2025-10-08_15-38-31.xlsx'); t_r10 = t_r10(1:250,:);
% t_r5 = readtable('Monte Carlo Simulation Results\Wall_Exp_and_DstarLite\250iterResults_v170_d5_2025-10-08_15-41-34.xlsx'); t_r5 = t_r5(1:250,:);
% t_r1 = readtable('Monte Carlo Simulation Results\Wall_Exp_and_DstarLite\250iterResults_v170_d1_2025-10-08_14-56-00.xlsx'); t_r1 = t_r1(1:250,:);
% t_r05 = readtable('Monte Carlo Simulation Results\Wall_Exp_and_DstarLite\250iterResults_v170_d0.5_2025-10-08_15-18-43.xlsx'); t_r05 = t_r05(1:250,:);

%% Genearte Summary Results
generateSummaryTable(t_r05, t_r1, t_r5, t_r10, t_r30, dataset_type);

% Initialize counters for each floor combination
floor_combinations = {...
    '2nd-2nd', '3rd-3rd', '4th-4th', ...
    '2nd-3rd', '2nd-4th', ...
    '3rd-2nd', '3rd-4th', ...
    '4th-2nd', '4th-3rd'...
};
counts = zeros(1, length(floor_combinations));

% Bar plot - Number Simulations Per Floor Combination
plotNumberSimulationsPerFloorCombination(t_r30, floor_combinations, counts)
% 3D bar plot - Average Trajectory Length Per Floor Combination and Radius
plot3DBarAverageTrajectoryLength(t_r05, t_r1, t_r5, t_r10, t_r30, floor_combinations);

%% Helper functions:
function plotNumberSimulationsPerFloorCombination(t, floor_combinations, counts)
    % Count occurrences of each floor combination
    for i = 1:size(t, 1)
        % Parse start point
        startStr = t.StartPoint{i};
        startPoint = str2num(startStr(2:end-1));
        
        % Parse goal point
        goalStr = t.GoalPoint{i};
        goalPoint = str2num(goalStr(2:end-1));
        
        % Determine start and goal floors
        startFloor = startPoint(3)/100 + 2; % (2nd, 3rd, 4th floor)
        goalFloor = goalPoint(3)/100 + 2; % (2nd, 3rd, 4th floor)
        
        % Increment appropriate counter
        if startFloor == goalFloor % cases: '2nd-2nd', '3rd-3rd', '4th-4th'
            counts(startFloor - 1) = counts(startFloor - 1) + 1;
        else
            if startFloor == 2 
                if goalFloor == 3
                    counts(4) = counts(4) + 1;
                elseif goalFloor == 4
                    counts(5) = counts(5) + 1;
                end
            elseif startFloor == 3
                if goalFloor == 2
                    counts(6) = counts(6) + 1;
                elseif goalFloor == 4
                    counts(7) = counts(7) + 1;
                end
            elseif startFloor == 4
                if goalFloor == 2
                    counts(8) = counts(8) + 1;
                elseif goalFloor == 3
                    counts(9) = counts(9) + 1;
                end
            end
        end
    end
    
    % Create bar plot
    figure;
    bar(counts);
    xlabel('Floor Combinations');
    ylabel('Number of Routes');
    title('Distribution of Routes Across Floor Combinations');
    xticks(1:length(floor_combinations));
    xticklabels(floor_combinations);
    xtickangle(45);
    
    % Add value labels on top of each bar
    for i = 1:length(counts)
        text(i, counts(i), num2str(counts(i)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
    
    % Calculate and display percentages
    total_routes = sum(counts);
    percentages = (counts / total_routes) * 100;
    
    disp('Distribution of routes:');
    for i = 1:length(floor_combinations)
        fprintf('%s: %d (%.2f%%)\n', floor_combinations{i}, counts(i), percentages(i));
    end
    
    % Display total number of routes
    fprintf('\nTotal number of routes: %d\n', total_routes);
end

function plot3DBarAverageTrajectoryLength(t_r05, t_r1, t_r5, t_r10, t_r30, floor_combinations)
    % Define radii
    radii = [0.5, 1, 5, 10, 30];
    
    % Initialize matrix to store average lengths
    avg_lengths = zeros(length(radii), length(floor_combinations));
    
    % Process data for each radius
    tables = {t_r05, t_r1, t_r5, t_r10, t_r30};
    for r = 1:length(radii)
        t = tables{r};
        avg_lengths(r, :) = calculateAverageLengths(t, floor_combinations);
    end
    
    % Create 3D bar plot
    figure;
    h = bar3(avg_lengths);    
    
    % Customize the plot
    xlabel('Floor Combinations','FontSize',18);
    ylabel('Vision Radius (m)','FontSize',18);
    zlabel('Average Trajectory Length (m)','FontSize',18);
    title('Average Trajectory Length by Floor Combination and Vision Radius','FontSize',18);
    
    % Set x-axis labels
    xticklabels(floor_combinations);
    xtickangle(45);
    
    % Set y-axis labels
    yticks(1:length(radii));
    yticklabels(radii);

    % Set font size for tick labels
    ax = gca;
    ax.XAxis.FontSize = 18;
    ax.YAxis.FontSize = 18;
    ax.ZAxis.FontSize = 18;
    
    % Adjust view for better visibility
    view(19.5,25.7);

    % Display numeric values on top of each bar
    for i = 1:size(avg_lengths, 1)
        for j = 1:size(avg_lengths, 2)
            % Determine text color based on floor combination
            if ismember(j, [1, 2, 8])  % 2nd-2nd, 3rd-3rd, and 4th-2nd
                textColor = 'white';
            else
                textColor = 'black';
            end
            
            text(j, i, avg_lengths(i,j), num2str(avg_lengths(i,j), '%.1f'), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                 'FontSize', 18, 'Color', textColor);
        end
    end

    % Face Colors:
    f_colors = [0 0 0;
                .5 .5 .5;
                .9 .9 .9;
                .85 .33 .1;
                .93 .69 .13;
                0.3647, 0.8000, 0.4549;
                0.59, 1, 0.59;
                0 .45 .74;
                .07 .62 1];

    % Set face colors for each bar
    for i = 1:length(h)
        h(i).FaceColor = f_colors(i,:);
    end

    disp("-------------------------------")
    disp("Average w/o in-floor trajectories")
    disp("Average Length path radius 0.5m: " + num2str(sum(avg_lengths(1,4:end)/(length(avg_lengths)-3))));
    disp("Average Length path radius 1m: " + num2str(sum(avg_lengths(2,4:end)/(length(avg_lengths)-3))));
    disp("Average Length path radius 5m: " + num2str(sum(avg_lengths(3,4:end)/(length(avg_lengths)-3))));
    disp("Average Length path radius 10m: " + num2str(sum(avg_lengths(4,4:end)/(length(avg_lengths)-3))));
    disp("Average Length path radius 30m: " + num2str(sum(avg_lengths(5,4:end)/(length(avg_lengths)-3))));
    disp("-------------------------------")
    disp("Average w in-floor trajectories")
    disp("Average Length path radius 0.5m: " + num2str(sum(avg_lengths(1,:)/(length(avg_lengths)))));
    disp("Average Length path radius 1m: " + num2str(sum(avg_lengths(2,:)/(length(avg_lengths)))));
    disp("Average Length path radius 5m: " + num2str(sum(avg_lengths(3,:)/(length(avg_lengths)))));
    disp("Average Length path radius 10m: " + num2str(sum(avg_lengths(4,:)/(length(avg_lengths)))));
    disp("Average Length path radius 30m: " + num2str(sum(avg_lengths(5,:)/(length(avg_lengths)))));
end

function avg_lengths = calculateAverageLengths(t, floor_combinations)
    avg_lengths = zeros(1, length(floor_combinations));
    counts = zeros(1, length(floor_combinations));
    
    for i = 1:size(t, 1)
        % Parse start and goal points
        startStr = t.StartPoint{i};
        startPoint = str2num(startStr(2:end-1));
        goalStr = t.GoalPoint{i};
        goalPoint = str2num(goalStr(2:end-1));
        
        % Determine start and goal floors
        startFloor = startPoint(3)/100 + 2;
        goalFloor = goalPoint(3)/100 + 2;
        
        % Determine the index for the floor combination
        if startFloor == goalFloor % cases: '2nd-2nd', '3rd-3rd', '4th-4th'
            idx = startFloor - 1;
        else
            if startFloor == 2 
                if goalFloor == 3
                    idx = 4;
                elseif goalFloor == 4
                    idx = 5;
                end
            elseif startFloor == 3
                if goalFloor == 2
                    idx = 6;
                elseif goalFloor == 4
                    idx = 7;
                end
            elseif startFloor == 4
                if goalFloor == 2
                    idx = 8;
                elseif goalFloor == 3
                    idx = 9;
                end
            end
        end
        
        % Add path length to the corresponding combination
        avg_lengths(idx) = avg_lengths(idx) + t.PathLength(i);
        counts(idx) = counts(idx) + 1;
    end
    
    % Calculate averages
    for idx = 1:length(floor_combinations)
        if counts(idx) > 0
            avg_lengths(idx) = avg_lengths(idx) / counts(idx);
        else
            avg_lengths(idx) = NaN;  % or 0, depending on how you want to handle no data
        end
    end
end

function m = meanIgnoreZero(x)
    % Mean that ignores NaNs and zeros; returns NaN if no valid samples
    x = x(~isnan(x) & x~=0);
    if isempty(x), m = NaN; else, m = mean(x); end
end

function col = pickColByType(dataset_type, which)
    % which = 'exploration' or 'goal'
    switch dataset_type
        case "BUGFIRE"
            col = iff(strcmp(which,'exploration'), 'EmergencyTrajectoryLength', 'Bug2TrajectoryLength');
        case "wall-d"
            col = iff(strcmp(which,'exploration'), 'WallTrajectoryLength', 'DstarTrajectoryLength');
        case "random-d"
            col = iff(strcmp(which,'exploration'), 'RandomTrajectoryLength', 'DstarTrajectoryLength');
        otherwise
            error('Unknown dataset_type: %s', dataset_type);
    end
end

function out = iff(cond, a, b)
    if cond
        out=a;
    else
        out=b;
    end
end

function generateSummaryTable(t_r05, t_r1, t_r5, t_r10, t_r30, dataset_type)
    % Define radii and corresponding tables
    radii  = [0.5, 1, 5, 10, 30];
    tables = {t_r05, t_r1, t_r5, t_r10, t_r30};

    % Initialize arrays to store summary statistics
    successRates          = zeros(length(radii), 1);
    avgExecutionTimes     = zeros(length(radii), 1);
    avgPathLengths_total  = zeros(length(radii), 1);

    % New: per-leg averages, ignoring zeros
    avgExplorationLen     = nan(length(radii), 1);
    avgGoalPlannerLen     = nan(length(radii), 1);

    % Column names to use for legs (depend on dataset_type)
    explorationCol = pickColByType(dataset_type, 'exploration');
    goalCol        = pickColByType(dataset_type, 'goal');

    for i = 1:length(radii)
        t = tables{i};

        % Success rate
        successCount        = sum(t.GoalReached);
        totalSimulations    = height(t);
        successRates(i)     = (successCount / totalSimulations) * 100;

        % Avg execution time (all runs or only successful—keep your original choice)
        avgExecutionTimes(i) = mean(t.ExecutionTime, 'omitnan');

        % Avg total path only for successful runs and > 0
        maskSucc             = t.GoalReached == true;
        totalPath            = t.PathLength;
        totalPath(~maskSucc) = NaN;           % drop unsuccessful
        totalPath(totalPath==0) = NaN;        % ignore zeros if any
        avgPathLengths_total(i) = mean(totalPath, 'omitnan');

        % --- New: exploration / goal legs (ignore zeros) over successful runs ---
        if ismember(explorationCol, t.Properties.VariableNames)
            x = t.(explorationCol);
            x(~maskSucc) = 0; % ensure we only consider successful runs
            avgExplorationLen(i) = meanIgnoreZero(x);
        end

        if ismember(goalCol, t.Properties.VariableNames)
            y = t.(goalCol);
            y(~maskSucc) = 0;
            avgGoalPlannerLen(i) = meanIgnoreZero(y);
        end
    end

    % Create summary table
    summaryTable = table( ...
        radii', successRates, avgExecutionTimes, avgPathLengths_total, ...
        avgExplorationLen, avgGoalPlannerLen, ...
        'VariableNames', { ...
            'RadiusSize', ...
            'SuccessRate_Percent', ...
            'AvgExecutionTime_Seconds', ...
            'AvgPathLength_Meters', ...
            [char(explorationCol) '_Avg_m_NoZeros_SuccessOnly'], ...
            [char(goalCol)        '_Avg_m_NoZeros_SuccessOnly'] ...
        } ...
    );

    % Display (pretty print)
    disp('========================================================================');
    disp(['SUMMARY TABLE - Monte Carlo Simulation Results (' char(dataset_type) ')']);
    disp('========================================================================');
    fprintf('%-10s | %-16s | %-21s | %-18s | %-18s | %-18s\n', ...
        'Radius', 'Success Rate (%)', 'Avg Exec Time (s)', 'Avg Path (m)', ...
        [char(explorationCol) ' (m)'], [char(goalCol) ' (m)']);
    disp('------------------------------------------------------------------------');
    for i = 1:length(radii)
        fprintf('%-10.1f | %-16.2f | %-21.2f | %-18.2f | %-18.2f | %-18.2f\n', ...
            summaryTable.RadiusSize(i), ...
            summaryTable.SuccessRate_Percent(i), ...
            summaryTable.AvgExecutionTime_Seconds(i), ...
            summaryTable.AvgPathLength_Meters(i), ...
            summaryTable{i,5}, summaryTable{i,6});
    end
    disp('========================================================================');

    % Optional: quick plots (kept your original, now totals). You can add bars
    % for exploration/goal if you want; left minimal to avoid clutter.
    createSummaryPlots(summaryTable);
end

function createSummaryPlots(summaryTable)
    % Create subplots for visual representation
    figure('Position', [100, 100, 1200, 800]);
    
    % Success Rate Plot
    subplot(2, 2, 1);
    bar(summaryTable.RadiusSize, summaryTable.SuccessRate_Percent, 'FaceColor', [0.2, 0.6, 0.8]);
    xlabel('Vision Radius (m)', 'FontSize', 12);
    ylabel('Success Rate (%)', 'FontSize', 12);
    title('Success Rate vs Vision Radius', 'FontSize', 14);
    grid on;
    ylim([0, 100]);
    
    % Add value labels on bars
    for i = 1:length(summaryTable.RadiusSize)
        text(summaryTable.RadiusSize(i), summaryTable.SuccessRate_Percent(i) + 2, ...
            sprintf('%.1f%%', summaryTable.SuccessRate_Percent(i)), ...
            'HorizontalAlignment', 'center', 'FontSize', 10);
    end
    
    % Average Execution Time Plot
    subplot(2, 2, 2);
    bar(summaryTable.RadiusSize, summaryTable.AvgExecutionTime_Seconds, 'FaceColor', [0.8, 0.4, 0.2]);
    xlabel('Vision Radius (m)', 'FontSize', 12);
    ylabel('Average Execution Time (s)', 'FontSize', 12);
    title('Average Execution Time vs Vision Radius', 'FontSize', 14);
    grid on;
    
    % Add value labels on bars
    for i = 1:length(summaryTable.RadiusSize)
        text(summaryTable.RadiusSize(i), summaryTable.AvgExecutionTime_Seconds(i) + max(summaryTable.AvgExecutionTime_Seconds)*0.02, ...
            sprintf('%.1fs', summaryTable.AvgExecutionTime_Seconds(i)), ...
            'HorizontalAlignment', 'center', 'FontSize', 10);
    end
    
    % Average Path Length Plot
    subplot(2, 2, 3);
    bar(summaryTable.RadiusSize, summaryTable.AvgPathLength_Meters, 'FaceColor', [0.2, 0.8, 0.4]);
    xlabel('Vision Radius (m)', 'FontSize', 12);
    ylabel('Average Path Length (m)', 'FontSize', 12);
    title('Average Path Length vs Vision Radius', 'FontSize', 14);
    grid on;
    
    % Add value labels on bars
    for i = 1:length(summaryTable.RadiusSize)
        if ~isnan(summaryTable.AvgPathLength_Meters(i))
            text(summaryTable.RadiusSize(i), summaryTable.AvgPathLength_Meters(i) + max(summaryTable.AvgPathLength_Meters, [], 'omitnan')*0.02, ...
                sprintf('%.1fm', summaryTable.AvgPathLength_Meters(i)), ...
                'HorizontalAlignment', 'center', 'FontSize', 10);
        end
    end
    
    % Combined Plot
    subplot(2, 2, 4);
    yyaxis left;
    plot(summaryTable.RadiusSize, summaryTable.SuccessRate_Percent, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    ylabel('Success Rate (%)', 'FontSize', 12);
    ylim([0, 100]);
    
    yyaxis right;
    plot(summaryTable.RadiusSize, summaryTable.AvgPathLength_Meters, 's-', 'LineWidth', 2, 'MarkerSize', 8);
    ylabel('Average Path Length (m)', 'FontSize', 12);
    
    xlabel('Vision Radius (m)', 'FontSize', 12);
    title('Success Rate and Path Length vs Vision Radius', 'FontSize', 14);
    legend('Success Rate', 'Avg Path Length', 'Location', 'best');
    grid on;
    
    sgtitle('Monte Carlo Simulation Summary Analysis', 'FontSize', 16, 'FontWeight', 'bold');
end