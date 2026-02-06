%% File Header
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description:
%   This script plots the Monte Carlo Simulation Results of the BUGFIRE,
%   performances with different vision Radius with different IMU drift 
%   characteristics.
%
%   Author: Eudald Sangenis Rafart
%   Affiliation: University of California, Irvine
%   Email: esangeni@uci.edu
%   Date: Feb. 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;
set(0, 'defaultTextInterpreter', 'latex');
set(0, 'defaultAxesTickLabelInterpreter', 'latex');
set(0, 'defaultLegendInterpreter', 'latex');

%% Import all the datasets with the performance of the BUG3 algorithm with
% different radius parameters

% BUG-FIRE (ADIS16490):
dataset_type = "BUGFIRE"; 
t_r30 = readtable('Monte Carlo Simulation Results\BUGFIRE_DriftModel_ADIS16490\250iterResults_v170_d30_2025-11-06_14-02-08.xlsx'); t_r30 = t_r30(1:250,:);
t_r10 = readtable('Monte Carlo Simulation Results\BUGFIRE_DriftModel_ADIS16490\250iterResults_v170_d10_2025-11-06_13-59-59.xlsx'); t_r10 = t_r10(1:250,:);
t_r5 = readtable('Monte Carlo Simulation Results\BUGFIRE_DriftModel_ADIS16490\250iterResults_v170_d5_2025-11-06_14-14-36.xlsx'); t_r5 = t_r5(1:250,:);
t_r1 = readtable('Monte Carlo Simulation Results\BUGFIRE_DriftModel_ADIS16490\250iterResults_v170_d1_2025-11-06_14-06-04.xlsx'); t_r1 = t_r1(1:250,:);
t_r05 = readtable('Monte Carlo Simulation Results\BUGFIRE_DriftModel_ADIS16490\250iterResults_v170_d0.5_2025-11-06_14-11-44.xlsx'); t_r05 = t_r05(1:250,:);

% BUG-FIRE (VN-200):
% dataset_type = "BUGFIRE"; 
% t_r30 = readtable('Monte Carlo Simulation Results\BUGFIRE_DriftModel_VN200\250iterResults_v170_d30_2025-11-06_16-29-45.xlsx'); t_r30 = t_r30(1:250,:);
% t_r10 = readtable('Monte Carlo Simulation Results\BUGFIRE_DriftModel_VN200\250iterResults_v170_d10_2025-11-06_16-33-49.xlsx'); t_r10 = t_r10(1:250,:);
% t_r5 = readtable('Monte Carlo Simulation Results\BUGFIRE_DriftModel_VN200\250iterResults_v170_d5_2025-11-06_16-45-38.xlsx'); t_r5 = t_r5(1:250,:);
% t_r1 = readtable('Monte Carlo Simulation Results\BUGFIRE_DriftModel_VN200\250iterResults_v170_d1_2025-11-06_16-41-55.xlsx'); t_r1 = t_r1(1:250,:);
% t_r05 = readtable('Monte Carlo Simulation Results\BUGFIRE_DriftModel_VN200\250iterResults_v170_d0.5_2025-11-06_16-49-51.xlsx'); t_r05 = t_r05(1:250,:);

% BUG-FIRE (ICM20948):
% dataset_type = "BUGFIRE"; 


%% Genearte Summary Results
generateSummaryTableV2(t_r05, t_r1, t_r5, t_r10, t_r30, dataset_type);

% Initialize counters for each floor combination
floor_combinations = {...
    '2nd-2nd', '3rd-3rd', '4th-4th', ...
    '2nd-3rd', '2nd-4th', ...
    '3rd-2nd', '3rd-4th', ...
    '4th-2nd', '4th-3rd'...
};
counts = zeros(1, length(floor_combinations));

% Bar plot - Number Simulations Per Floor Combination
plotNumberSimulationsPerFloorCombination(t_r30, floor_combinations, counts);

% 3D bar with True and Estimated (two figures):
plot3DBarAverageTrajectoryLengthV2(t_r05, t_r1, t_r5, t_r10, t_r30, floor_combinations, false);
plot3DBarAverageTrajectoryLengthV2(t_r05, t_r1, t_r5, t_r10, t_r30, floor_combinations, true);

% Deep-dive for the single d=1 table:
quickEstVsTrueAndDrift(t_r1, dataset_type, '(d=1)');

%% Functions:
function cols = pickColsByTypeV2(dataset_type, use_est)
    % use_est = true -> use *_Est columns, else *_True
    if use_est
        suf = '_Est';
    else
        suf = '_True';
    end

    switch dataset_type
        case "BUGFIRE"
            cols.exploration = ['EmergencyLength' suf];
            cols.goal        = ['Bug2Length' suf];
        case "wall-d"
            cols.exploration = ['WallTrajectoryLength' suf];   % if your new XLS kept old name
            cols.goal        = ['DstarTrajectoryLength' suf];
        case "random-d"
            cols.exploration = ['RandomTrajectoryLength' suf];
            cols.goal        = ['DstarTrajectoryLength' suf];
        otherwise
            error('Unknown dataset_type: %s', dataset_type);
    end

    cols.path_total  = ['PathLength' suf];
    cols.goal_true   = 'GoalReached_True';
    cols.goal_est    = 'GoalReached_Est';
    cols.drift_m     = 'DriftError_m';
    cols.drift_pct   = 'DriftError_Percent';
end

function v = parsePoint(vecStr)
    % vecStr like "[87, 225, 100]"; robust to spaces
    nums = sscanf(vecStr, '[%f,%f,%f]');
    if numel(nums)~=3
        nums = sscanf(strrep(strrep(vecStr,'[',''),']',''), '%f,%f,%f');
    end
    v = nums(:).';
end

function avg_lengths = calculateAverageLengthsV2(t, floor_combinations, use_est)
    avg_lengths = nan(1, length(floor_combinations));
    sums = zeros(1, length(floor_combinations));
    cnts = zeros(1, length(floor_combinations));

    cols = pickColsByTypeV2(evalin('base','dataset_type'), use_est); % get dataset_type from base
    pathCol = cols.path_total;

    for i = 1:height(t)
        % floors (2,3,4) from Z/100 + 2
        sp = parsePoint(t.StartPoint{i});
        gp = parsePoint(t.GoalPoint{i});
        startFloor = sp(3)/100 + 2;
        goalFloor  = gp(3)/100 + 2;

        if startFloor==goalFloor
            idx = startFloor - 1;     % 1->2nd-2nd, 2->3rd-3rd, 3->4th-4th
        else
            if startFloor==2 && goalFloor==3, idx=4;
            elseif startFloor==2 && goalFloor==4, idx=5;
            elseif startFloor==3 && goalFloor==2, idx=6;
            elseif startFloor==3 && goalFloor==4, idx=7;
            elseif startFloor==4 && goalFloor==2, idx=8;
            elseif startFloor==4 && goalFloor==3, idx=9;
            else, continue;
            end
        end

        val = t.(pathCol)(i);
        if isnan(val) || val<=0, continue; end
        sums(idx) = sums(idx) + val;
        cnts(idx) = cnts(idx) + 1;
    end

    nz = cnts>0;
    avg_lengths(nz) = sums(nz)./cnts(nz);
end

function plot3DBarAverageTrajectoryLengthV2(t_r05, t_r1, t_r5, t_r10, t_r30, floor_combinations, use_est)
    % use_est=false -> True lengths (default), true -> Estimated
    if nargin<7, use_est=false; end

    radii  = [0.5, 1, 5, 10, 30];
    tables = {t_r05, t_r1, t_r5, t_r10, t_r30};
    avg_lengths = zeros(length(radii), length(floor_combinations));

    for r = 1:length(radii)
        t = tables{r};
        avg_lengths(r,:) = calculateAverageLengthsV2(t, floor_combinations, use_est);
    end

    figure;
    h = bar3(avg_lengths);

    if use_est
        lab = 'Estimated';
    else
        lab = 'True';
    end

    xlabel('Floor Combinations','FontSize',18);
    ylabel('Vision Radius (m)','FontSize',18);
    zlabel(sprintf('Average %s Path Length (m)', lab),'FontSize',18);
    title(sprintf('Average %s Path Length by Floor Combination and Vision Radius', lab),'FontSize',18);

    xticklabels(floor_combinations); xtickangle(45);
    yticks(1:length(radii)); yticklabels(radii);
    ax = gca; ax.XAxis.FontSize=18; ax.YAxis.FontSize=18; ax.ZAxis.FontSize=18;
    view(19.5,25.7);

    % annotate bars
    for i = 1:size(avg_lengths,1)
        for j = 1:size(avg_lengths,2)
            if ~isnan(avg_lengths(i,j))
                text(j,i,avg_lengths(i,j), sprintf('%.1f',avg_lengths(i,j)), ...
                    'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',16);
            end
        end
    end

    % simple face colors
    cmap = lines(length(h));
    for i = 1:length(h), h(i).FaceColor = cmap(i,:); end

    % quick console summaries
    disp("-------------------------------")
    if use_est
        lab = '(Estimated)';
    else
        lab = '(True)';
    end
    disp("Average w/o in-floor trajectories " + lab)
    disp("r=0.5: " + num2str(mean(avg_lengths(1,4:end),'omitnan')));
    disp("r=1:   " + num2str(mean(avg_lengths(2,4:end),'omitnan')));
    disp("r=5:   " + num2str(mean(avg_lengths(3,4:end),'omitnan')));
    disp("r=10:  " + num2str(mean(avg_lengths(4,4:end),'omitnan')));
    disp("r=30:  " + num2str(mean(avg_lengths(5,4:end),'omitnan')));

    disp("-------------------------------")
    disp("Average with in-floor trajectories " + lab)
    for k=1:size(avg_lengths,1)
        fprintf('r=%g: %g\n', radii(k), mean(avg_lengths(k,:), 'omitnan'));
    end
end

function generateSummaryTableV2(t_r05, t_r1, t_r5, t_r10, t_r30, dataset_type)
    radii  = [0.5, 1, 5, 10, 30];
    tables = {t_r05, t_r1, t_r5, t_r10, t_r30};

    % arrays to fill
    SR_true  = zeros(numel(radii),1);   % success by ground truth goal flag
    SR_est   = zeros(numel(radii),1);   % success by estimated goal flag
    SR_est_5m = zeros(numel(radii),1);  % success where estimated final pos within 5 m
    time_avg = zeros(numel(radii),1);

    % path (True) and (Est)
    path_true_avg = nan(numel(radii),1);
    path_est_avg  = nan(numel(radii),1);
    path_mae      = nan(numel(radii),1);
    path_bias     = nan(numel(radii),1); % Est-True

    % per leg (only when columns exist)
    expl_true_avg = nan(numel(radii),1);
    expl_est_avg  = nan(numel(radii),1);
    goal_true_avg = nan(numel(radii),1);
    goal_est_avg  = nan(numel(radii),1);

    % explicit per-leg names (Emergency & Bug2), averages over >0 only
    Emergency_True_Avg_m_NoZeros = nan(numel(radii),1);
    Emergency_Est_Avg_m_NoZeros  = nan(numel(radii),1);
    Bug2_True_Avg_m_NoZeros      = nan(numel(radii),1);
    Bug2_Est_Avg_m_NoZeros       = nan(numel(radii),1);

    Emergency_True_N = zeros(numel(radii),1);
    Emergency_Est_N  = zeros(numel(radii),1);
    Bug2_True_N      = zeros(numel(radii),1);
    Bug2_Est_N       = zeros(numel(radii),1);

    % drift
    drift_m_med   = nan(numel(radii),1);
    drift_pct_med = nan(numel(radii),1);

    % confusion stats
    TP = zeros(numel(radii),1); TN=TP; FP=TP; FN=TP;

    for i=1:numel(radii)
        t = tables{i};
        colsT = pickColsByTypeV2(dataset_type,false);
        colsE = pickColsByTypeV2(dataset_type,true);

        % success flags
        gt   = logical(t.('GoalReached_True'));
        gest = logical(t.('GoalReached_Est'));
        if ismember('FinalPosWithin5m', t.Properties.VariableNames)
            g5m = logical(t.('FinalPosWithin5m'));
        else
            g5m = false(height(t),1); % fallback if column missing
        end

        SR_true(i)   = 100*mean(gt);
        SR_est(i)    = 100*mean(gest);
        SR_est_5m(i) = 100*mean(g5m);

        % exec time
        time_avg(i)= mean(t.ExecutionTime, 'omitnan');

        % total path
        pt = t.(colsT.path_total);
        pe = t.(colsE.path_total);
        path_true_avg(i) = mean(pt(pt>0), 'omitnan');
        path_est_avg(i)  = mean(pe(pe>0), 'omitnan');

        % errors (only where both valid)
        mask = pt>0 & pe>0 & ~isnan(pt) & ~isnan(pe);
        if any(mask)
            dif = pe(mask)-pt(mask);
            path_mae(i)  = mean(abs(dif));
            path_bias(i) = mean(dif);
        end

        % ---- Per-leg: Emergency (exploration) ----
        if ismember(colsT.exploration, t.Properties.VariableNames)
            exT_all = t.(colsT.exploration);
            exE_all = t.(colsE.exploration);
            exT = exT_all(exT_all>0 & ~isnan(exT_all));
            exE = exE_all(exE_all>0 & ~isnan(exE_all));

            expl_true_avg(i) = mean(exT,'omitnan');
            expl_est_avg(i)  = mean(exE,'omitnan');
            Emergency_True_Avg_m_NoZeros(i) = mean(exT,'omitnan');
            Emergency_Est_Avg_m_NoZeros(i)  = mean(exE,'omitnan');
            Emergency_True_N(i) = numel(exT);
            Emergency_Est_N(i)  = numel(exE);
        end

        % ---- Per-leg: Bug2 (goal planner) ----
        if ismember(colsT.goal, t.Properties.VariableNames)
            gT_all = t.(colsT.goal);
            gE_all = t.(colsE.goal);
            gT = gT_all(gT_all>0 & ~isnan(gT_all));
            gE = gE_all(gE_all>0 & ~isnan(gE_all));

            goal_true_avg(i) = mean(gT,'omitnan');
            goal_est_avg(i)  = mean(gE,'omitnan');
            Bug2_True_Avg_m_NoZeros(i) = mean(gT,'omitnan');
            Bug2_Est_Avg_m_NoZeros(i)  = mean(gE,'omitnan');
            Bug2_True_N(i) = numel(gT);
            Bug2_Est_N(i)  = numel(gE);
        end

        % drift stats
        if ismember(colsT.drift_m, t.Properties.VariableNames)
            drift_m_med(i)   = median(t.(colsT.drift_m), 'omitnan');
        end
        if ismember(colsT.drift_pct, t.Properties.VariableNames)
            drift_pct_med(i) = median(t.(colsT.drift_pct), 'omitnan');
        end

        % confusion (True vs Est)
        TP(i) = sum( gt &  gest);
        TN(i) = sum(~gt & ~gest);
        FP(i) = sum(~gt &  gest);
        FN(i) = sum( gt & ~gest);
    end

    % ---- Build summary table ----
    summary = table(radii', SR_true, SR_est, SR_est_5m, time_avg, ...
        path_true_avg, path_est_avg, path_bias, path_mae, ...
        expl_true_avg, expl_est_avg, goal_true_avg, goal_est_avg, ...
        Emergency_True_Avg_m_NoZeros, Emergency_Est_Avg_m_NoZeros, ...
        Bug2_True_Avg_m_NoZeros,     Bug2_Est_Avg_m_NoZeros, ...
        Emergency_True_N, Emergency_Est_N, Bug2_True_N, Bug2_Est_N, ...
        drift_m_med, drift_pct_med, TP, TN, FP, FN, ...
        'VariableNames', {'Radius','SR_True_%','SR_Est_%','SR_Est_Within5m_%','AvgTime_s', ...
        'AvgPath_True_m','AvgPath_Est_m','Bias_m','MAE_m', ...
        'Expl_True_m','Expl_Est_m','Goal_True_m','Goal_Est_m', ...
        'Emergency_True_Avg_m_NoZeros','Emergency_Est_Avg_m_NoZeros', ...
        'Bug2_True_Avg_m_NoZeros','Bug2_Est_Avg_m_NoZeros', ...
        'Emergency_True_N','Emergency_Est_N','Bug2_True_N','Bug2_Est_N', ...
        'MedianDrift_m','MedianDrift_%','TP','TN','FP','FN'});

    disp('========================================================================');
    disp(['SUMMARY TABLE v2 - Monte Carlo (' char(dataset_type) ')']);
    disp(summary);
    disp('========================================================================');

    % quick plots
    createSummaryPlotsV2(summary);
    plotConfusionByRadius(radii, TP, TN, FP, FN);
end

function createSummaryPlotsV2(T)
    figure('Position',[100,100,1300,860]);

    % ----- pull columns safely (names contain % so use string indexing) -----
    r   = T.('Radius');
    srT = T.('SR_True_%');
    srE = T.('SR_Est_%');
    apt = T.('AvgPath_True_m');
    ape = T.('AvgPath_Est_m');
    bia = T.('Bias_m');
    mae = T.('MAE_m');
    d_m = T.('MedianDrift_m');
    d_p = T.('MedianDrift_%');

    % Success (True vs Est)
    subplot(2,2,1);
    hold on;
    bar(r-0.15, srT, 0.3);
    bar(r+0.15, srE, 0.3);
    hold off; grid on;
    xlabel('Vision Radius (m)'); ylabel('Success Rate (%)');
    title('Success Rate (True vs Est)');
    legend('True','Est','Location','best');

    % Path (True vs Est)
    subplot(2,2,2);
    hold on;
    plot(r, apt, '-o','LineWidth',2,'MarkerSize',8);
    plot(r, ape, '-s','LineWidth',2,'MarkerSize',8);
    hold off; grid on;
    xlabel('Vision Radius (m)'); ylabel('Average Path (m)');
    title('Average Path Length'); legend('True','Est','Location','best');

    % Bias / MAE
    subplot(2,2,3);
    yyaxis left;  plot(r, bia,'-o','LineWidth',2,'MarkerSize',8); ylabel('Bias (m)');
    yyaxis right; plot(r, mae,'-s','LineWidth',2,'MarkerSize',8); ylabel('MAE (m)');
    grid on; xlabel('Vision Radius (m)'); title('Estimation Error (Est - True)');

    % Drift medians
    subplot(2,2,4);
    yyaxis left;  plot(r, d_m, '-o','LineWidth',2,'MarkerSize',8); ylabel('Median Drift (m)');
    yyaxis right; plot(r, d_p, '-s','LineWidth',2,'MarkerSize',8); ylabel('Median Drift (%)');
    grid on; xlabel('Vision Radius (m)'); title('Drift Error (Median)');
end

function plotConfusionByRadius(radii, TP, TN, FP, FN)
    figure('Position',[200,150,900,420]);
    data = [TP TN FP FN];
    bar(radii, data, 'stacked');
    grid on; xlabel('Vision Radius (m)'); ylabel('Count');
    legend('TP','TN','FP','FN','Location','best');
    title('GoalReached: Confusion Components by Radius');
end

function quickEstVsTrueAndDrift(t, dataset_type, label)
    if nargin<3, label=''; end
    C_T = pickColsByTypeV2(dataset_type,false);
    C_E = pickColsByTypeV2(dataset_type,true);

    figure('Position',[80,80,1400,500]);

    % Scatter: Total path Est vs True
    subplot(1,3,1);
    x = t.(C_T.path_total); y = t.(C_E.path_total);
    mask = x>0 & y>0 & ~isnan(x) & ~isnan(y);
    scatter(x(mask), y(mask), 12, 'filled'); grid on; axis equal;
    hold on; mx = max([x(mask); y(mask)]); plot([0 mx],[0 mx],'k--'); hold off;
    xlabel('Path True (m)'); ylabel('Path Est (m)');
    title(['Total Path: Est vs True ' label]);

    % Scatter: per-leg Est vs True (if exists)
    subplot(1,3,2);
    if ismember(C_T.exploration, t.Properties.VariableNames) && ismember(C_E.exploration, t.Properties.VariableNames)
        exT = t.(C_T.exploration); exE = t.(C_E.exploration);
        mk  = exT>0 & exE>0;
        scatter(exT(mk), exE(mk), 12, 'filled'); hold on;
        mg = max([exT(mk); exE(mk)]); plot([0 mg],[0 mg],'k--'); hold off; grid on; axis equal;
        xlabel([C_T.exploration ' (m)']); ylabel([C_E.exploration ' (m)']);
        title('Exploration Leg: Est vs True');
    else
        text(0.1,0.5,'(No exploration columns)','FontSize',12);
        axis off;
    end

    % Drift histogram
    subplot(1,3,3);
    if ismember(C_T.drift_m, t.Properties.VariableNames)
        dm = t.(C_T.drift_m);
        histogram(dm, 25); grid on;
        xlabel('Drift Error (m)'); ylabel('Count');
        title('Drift Error Distribution');
    else
        text(0.1,0.5,'(No drift columns)','FontSize',12);
        axis off;
    end
end

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
