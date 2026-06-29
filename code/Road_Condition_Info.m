%% 复杂综合道路生成器 (支持变速与变附着) - 终极版
% 功能：解析 {Type, Param1, Param2, TargetSpeed, Mu} 格式数据
% 生成 Simulink 查表用的 Map 和 CarSim 建模用的坐标表

clc; clear; close all;

% =========================================================================
% 1. 你的超级道路定义 (5个参数)
% =========================================================================
% 格式：{类型(1直/2弯), 参数1(长/半径), 参数2(0/角度), 目标车速(km/h), 附着系数(0.2-1.0)}
% 正半径 = 左转, 负半径 = 右转

Road_Segments_Raw = {
    % 类型  参数1(m)  参数2(deg)  速度(km/h)   附着(Mu)
    {1,     200,       0,         68,         0.85};   
    {2,     60,       180,        68,         0.85};  
    {1,     300,       0,         68,         0.60};   
    {2,    -80,       90,         68,         0.50};    
    {1,     200,      0,          68,          0.85};  
    {2,     100,      90,         68,         0.50};   
    {2,    -120,      180,        68,         0.30};   
    {1,     150,      0,          68,         0.60};   
    {2,     60,       90,         68,         0.65};    
    {2,    -80,       180,        68,         0.60};    
    {1,     200,       0,         68,         0.60};  
    {2,     70,       180,        68,         0.60};   
    {2,    -100,      90,         68,         0.50}; 
    {1,     380,      0,          68,         0.85}; 
};

% =========================================================================
% 2. 核心生成逻辑
% =========================================================================
ds = 0.5; % 离散步长 0.5m

% 初始化全局变量
Global_S = 0; 
Global_K = 0; 
Global_X = 0; 
Global_Y = 0; 
Global_Heading = 0; 
Global_Mu = 0.85; % 初始附着
Global_Vx =68/3.6; % 初始速度(m/s)

current_s = 0;
current_x = 0;
current_y = 0;
current_heading = 0;

fprintf('正在生成变速变附着道路...\n');

for i = 1:length(Road_Segments_Raw)
    seg = Road_Segments_Raw{i};
    type = seg{1};
    param1 = seg{2};
    param2 = seg{3};
    target_v_kmh = seg{4}; % 新增：目标车速
    target_mu    = seg{5}; % 新增：目标附着
    
    x_local = []; y_local = []; s_local = []; k_local = []; 
    mu_local = []; vx_local = [];
    
    target_v_ms = target_v_kmh / 3.6;
    
    if type == 1 
        % --- 直道 ---
        len = param1;
        seg_k = 0;
        s_vec = ds:ds:len;
        
        for j = 1:length(s_vec)
            dist = ds;
            current_x = current_x + dist * cos(current_heading);
            current_y = current_y + dist * sin(current_heading);
            x_local = [x_local; current_x];
            y_local = [y_local; current_y];
        end
        s_local = s_vec' + current_s;
        k_local = zeros(length(s_vec), 1);
        current_s = current_s + len;
        
    elseif type == 2
        % --- 弯道 ---
        R = param1;
        Angle_deg = param2;
        Angle_rad = deg2rad(Angle_deg);
        len = abs(R) * Angle_rad; 
        seg_k = 1.0 / R;
        s_vec = ds:ds:len;
        
        for j = 1:length(s_vec)
            dist = ds;
            d_theta = dist * seg_k; 
            current_heading = current_heading + d_theta;
            current_x = current_x + dist * cos(current_heading);
            current_y = current_y + dist * sin(current_heading);
            x_local = [x_local; current_x];
            y_local = [y_local; current_y];
        end
        s_local = s_vec' + current_s;
        k_local = seg_k * ones(length(s_vec), 1);
        current_s = current_s + len;
    end
    
    % 生成本段的 Mu 和 Vx 序列 (阶跃变化，Lookup Table 会自动插值)
    mu_local = target_mu * ones(length(s_local), 1);
    vx_local = target_v_ms * ones(length(s_local), 1);
    
    % 拼接到全局
    Global_S = [Global_S; s_local];
    Global_K = [Global_K; k_local];
    Global_X = [Global_X; x_local];
    Global_Y = [Global_Y; y_local];
    Global_Mu = [Global_Mu; mu_local];
    Global_Vx = [Global_Vx; vx_local];
end

% 缓冲尾段
Global_S = [Global_S; current_s + 200];
Global_K = [Global_K; 0];
Global_X = [Global_X; current_x + 200 * cos(current_heading)];
Global_Y = [Global_Y; current_y + 200 * sin(current_heading)];
Global_Mu = [Global_Mu; Global_Mu(end)];
Global_Vx = [Global_Vx; Global_Vx(end)];

% =========================================================================
% 3. 数据清洗 (防止 Simulink 查表报错)
% =========================================================================
% 确保 S 严格单调递增，剔除极小重复点
[unique_S, idx] = unique(Global_S);
Global_S = Global_S(idx);
Global_K = Global_K(idx);
Global_Mu = Global_Mu(idx);
Global_Vx = Global_Vx(idx);
% X,Y 不需要查表，不用严格对齐

% =========================================================================
% 4. 导出到 Workspace
% =========================================================================

% A. 曲率 Map (给 Simulink 查表: S -> K)
Road_Curvature_Map = [Global_S, Global_K];
assignin('base', 'Road_Curvature_Map', Road_Curvature_Map);

% B. 附着系数 Map (给 Simulink 查表: S -> Mu)
Road_Mu_Map = [Global_S, Global_Mu];
assignin('base', 'Road_Mu_Map', Road_Mu_Map);

% C. 目标车速 Map (给 Simulink 查表: S -> Vx_ref)
% 注意：这里是 m/s，如果你 Simulink 里要 km/h，请在 Simulink 里乘 3.6
Target_Speed_Map = [Global_S, Global_Vx];
assignin('base', 'Target_Speed_Map', Target_Speed_Map);

% D. 初始变量 (防止 Simulink 初始报错)
assignin('base', 'Sim_Vx_Set', Global_Vx(1)); % 初始速度
assignin('base', 'Sim_Mu', Global_Mu(1));     % 初始附着

% E. CarSim 坐标表
XY_Data_For_CarSim = [Global_X(idx), Global_Y(idx)]; % 保持长度一致
assignin('base', 'XY_Data_For_CarSim', XY_Data_For_CarSim);

% =========================================================================
% 5. 绘图预览
% =========================================================================
figure(1);
subplot(3,1,1);
plot(Global_X, Global_Y, 'b-', 'LineWidth', 2); axis equal; grid on;
title('道路轨迹 (X-Y)'); xlabel('X [m]'); ylabel('Y [m]');

subplot(3,1,2);
plot(Global_S, Global_Vx*3.6, 'r-', 'LineWidth', 1.5); grid on;
title('目标车速分布'); xlabel('S [m]'); ylabel('Speed [km/h]');

subplot(3,1,3);
plot(Global_S, Global_Mu, 'k-', 'LineWidth', 1.5); grid on;
title('路面附着系数分布'); xlabel('S [m]'); ylabel('Mu');
ylim([0, 1.0]);

fprintf('\n-------------------------------------------------------\n');
fprintf('★ 生成成功！★\n');
fprintf('1. 请把 XY_Data_For_CarSim 粘贴到 CarSim 坐标表。\n');
fprintf('2. 新增了 Target_Speed_Map，你可以在 Simulink 里加个 Lookup Table 查这个表，作为期望车速输入给 MPC。\n');
fprintf('-------------------------------------------------------\n');
openvar('XY_Data_For_CarSim');