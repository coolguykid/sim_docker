clc;
clear;
close all;

%% 基础安全距离 s_0 与主道速度-吞吐量关系
% 不考虑汇入影响，只看不同基础安全距离 s_0 下，主道速度 v_0 对基础吞吐量的影响。

%% ===================== 可调参数 =====================

s0List = [30 60 90];          % 基础安全距离，m
v0List = 10:2:50;             % 主道速度扫描范围，m/s

params = baseCapacityParams();

%% ===================== 容量估计 =====================

throughput = zeros(length(s0List),length(v0List));
safeGap = zeros(size(throughput));
mainSpacing = zeros(size(throughput));

for j = 1:length(s0List)
    for i = 1:length(v0List)
        v0 = v0List(i);
        s0 = s0List(j);

        safeGap(j,i) = speedDependentSafetyGap(v0,s0,params);
        mainSpacing(j,i) = params.mainSpacingFactor*safeGap(j,i);
        throughput(j,i) = v0/(params.Lveh + mainSpacing(j,i));
    end
end

for j = 1:length(s0List)
    [peakThroughput,peakIdx] = max(throughput(j,:));
    fprintf('s0 = %5.1f m: peak throughput = %.4f veh/s at v0 = %.1f m/s\n', ...
        s0List(j),peakThroughput,v0List(peakIdx));
end

%% ===================== 绘图和保存 =====================

outDir = fullfile('outputs','current');
if ~exist(outDir,'dir')
    mkdir(outDir);
end

colors = [0.000 0.447 0.741;
          0.850 0.325 0.098;
          0.466 0.674 0.188];
markers = {'o','s','^'};

fig = figure('Color','w','Position',[100 100 880 560]);
hold on;
for j = 1:length(s0List)
    plot(v0List,throughput(j,:), ...
        'Color',colors(j,:), ...
        'Marker',markers{j}, ...
        'LineWidth',2.4, ...
        'MarkerSize',7, ...
        'MarkerFaceColor','w', ...
        'DisplayName',sprintf('s_0 = %d m',s0List(j)));
end
grid on;
xlabel('Main-road speed v_0 (m/s)','FontSize',13);
ylabel('Expected throughput (veh/s)','FontSize',13);
title('Main-road Speed-Throughput Curves under Different s_0','FontSize',14);
legend('Location','best','FontSize',11);
set(gca,'FontSize',11);
exportgraphics(fig,fullfile(outDir,'s0_main_speed_throughput.png'),'Resolution',300);

summaryTable = table();
for j = 1:length(s0List)
    oneS0 = table( ...
        s0List(j)*ones(numel(v0List),1), ...
        v0List(:), ...
        safeGap(j,:)', ...
        mainSpacing(j,:)', ...
        throughput(j,:)', ...
        'VariableNames',{'s0','v0','safeGap','mainSpacing','throughputVehPerSec'});
    summaryTable = [summaryTable; oneS0]; %#ok<AGROW>
end

save(fullfile(outDir,'s0_main_speed_throughput.mat'), ...
    's0List','v0List','params','safeGap','mainSpacing','throughput','summaryTable');

fprintf('Saved figure: %s\n',fullfile(outDir,'s0_main_speed_throughput.png'));

%% ===================== 本脚本使用的函数 =====================

function params = baseCapacityParams()
%BASECAPACITYPARAMS 设置不考虑汇入时的基础容量参数。
    params = struct();
    params.Tgap = 1.2;              % 期望时间间隔，s
    params.Lveh = 5;                % 等效机长，m
    params.b = 2;                   % 舒适减速度，m/s^2
    params.brakingGapFactor = 0.6;  % 二次速度安全距离项系数
    params.mainSpacingFactor = 1.6; % 主道实际巡航间距相对安全距离的放大倍数
end

function gap = speedDependentSafetyGap(v,s0,params)
%SPEEDDEPENDENTSAFETYGAP 计算给定 s_0 和速度下的有效安全距离。
    gap = s0 + params.Tgap*v + ...
        params.brakingGapFactor*v^2/(2*params.b);
end
