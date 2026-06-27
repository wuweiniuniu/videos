function BYS_Full_Robust_Final()
% =========================================================================
% 贝叶斯优化脚本 v33.0 (弯道全维寻优 + 单轮Jerk + Danger采集 + 强制加速仿真)
% =========================================================================
clc; warning off;

% --- 1. 环境清理 ---
clearvars -except drive_* brake_* MotorData;
ModelName = 'siWDqudong_2'; 
if ~exist('drive_eff_table','var') && exist('setup_motor_efficiency_lookup_3.m', 'file')
    run('setup_motor_efficiency_lookup_3.m');
end
if ~exist('T_motor_max', 'var'), T_motor_max = 350; end
assignin('base', 'T_motor_max', T_motor_max);
if exist('MotorData', 'var'), assignin('base', 'MotorData', MotorData); end

% --- 2. 工况与路径配置 (已为您专门设置盲区补点参数) ---
Speed_List = 68; 
Mu_List = [0.3,0.35]; 
R_List = [120]; 
Max_Evals = 30; 

% [保存路径设置]
Manual_Path = 'D:\Carsim2019\DATE\Opt_Result_v17_Final_103';
if ~isempty(Manual_Path)
    Save_Dir = Manual_Path;
    if ~exist(Save_Dir, 'dir'), mkdir(Save_Dir); end
    fprintf('>>> [Init] 锁定工作目录: %s\n', Save_Dir);
else
    timestamp = datestr(now, 'mm-dd_HH-MM');
    Save_Dir = fullfile('D:\Carsim2019\DATE\', ['Opt_Result_' timestamp]);
    if ~exist(Save_Dir, 'dir'), mkdir(Save_Dir); end
end
Global_Log_File = fullfile(Save_Dir, 'Optimization_History_All_Steps.csv');

% =====================================================================
% ★ CSV 表头初始化，增加 Danger 列 (共 21 列) ★
% =====================================================================
if ~exist(Global_Log_File, 'file')
    fid = fopen(Global_Log_File, 'w');
    if fid > 0
        fprintf(fid, 'Iter,V_set,Mu,R_path,log_Q_beta,Real_Q_beta,log_Q_yaw,Real_Q_yaw,log_R_track,Real_R_track,log_R_rate,Real_R_rate,log_W_slip,Real_W_slip,Cost,SI_max,LatErr,Energy,SpdErr,Jerk,Danger\n');
        fclose(fid);
    end
end

% [中间结果汇总表]
Summary_File_Path = fullfile(Save_Dir, 'Intermediate_Results.mat');
if exist(Summary_File_Path, 'file')
    try 
        load(Summary_File_Path, 'Summary_Table'); 
        if ~ismember('R_path', Summary_Table.Properties.VariableNames)
            Summary_Table = table();
        end
    catch
        Summary_Table = table(); 
    end
else
    Summary_Table = table();
end

% --- 3. 初始种子 ---
DefaultInit.log_Q_beta = 3.3; 
DefaultInit.log_Q_yaw = 3.3; 
DefaultInit.log_R_track = 3.3; 
DefaultInit.log_R_rate = 3.47; 
DefaultInit.log_W_slip = 4.3; 
Inherited_Best_Log = DefaultInit;

% =====================================================================
% 主循环
% =====================================================================
for i = 1:length(Speed_List)
    Vx_kmh = Speed_List(i);
    for j = 1:length(Mu_List)
        Mu_val = Mu_List(j);
        for k = 1:length(R_List)
            R_path = R_List(k); 
            fprintf('\n==================================================\n');
            fprintf('>>> [Chain] 弯道全维寻优启动: V=%d km/h, Mu=%.2f, R=%d m\n', Vx_kmh, Mu_val, R_path);
            fprintf('==================================================\n');
            
            assignin('base', 'Sim_Vx_Set', Vx_kmh/3.6);
            assignin('base', 'Sim_Mu_Set', Mu_val);
            assignin('base', 'Sim_Road_Type', 1);
            assignin('base', 'R_path', R_path);
            
            % ★ 动态缓和曲线平滑生成
            Ts = 0.01; T_end = 30; Time = (0:Ts:T_end)';
            Vel = Vx_kmh / 3.6; Dist = Vel * Time; 
            Straight_Len = 20; 
            Transition_Len = Vel * 3.0; 
            
            X_ref = zeros(size(Time)); Y_ref = zeros(size(Time)); Phi_ref = zeros(size(Time));
            Curv_ref = zeros(size(Time)); 
            Target_Curv = 1e-4;
            if R_path <= 5000, Target_Curv = 1 / R_path; end
            
            for t_idx = 1:length(Time)
                d = Dist(t_idx);
                if d <= Straight_Len
                    Curv_ref(t_idx) = 1e-4; 
                elseif d > Straight_Len && d <= (Straight_Len + Transition_Len)
                    ratio = (d - Straight_Len) / Transition_Len;
                    smooth_ratio = 0.5 * (1 - cos(pi * ratio));
                    Curv_ref(t_idx) = 1e-4 + (Target_Curv - 1e-4) * smooth_ratio;
                else
                    Curv_ref(t_idx) = Target_Curv;
                end
                if t_idx > 1
                    ds = Dist(t_idx) - Dist(t_idx-1);
                    Phi_ref(t_idx) = Phi_ref(t_idx-1) + Curv_ref(t_idx) * ds;
                    X_ref(t_idx) = X_ref(t_idx-1) + cos(Phi_ref(t_idx)) * ds;
                    Y_ref(t_idx) = Y_ref(t_idx-1) + sin(Phi_ref(t_idx)) * ds;
                end
            end
            
            assignin('base', 'X_ref', X_ref);
            assignin('base', 'Y_ref', Y_ref);
            assignin('base', 'Phi_ref', Phi_ref);
            Sim_Curv_Array = [Time, Curv_ref];
            assignin('base', 'Sim_Curv_Array', Sim_Curv_Array);
            
            if R_path > 5000, assignin('base', 'Sim_Curv_Set', 1e-4); else, assignin('base', 'Sim_Curv_Set', 1 / R_path); end
            if i > 1 || k > 1 || j > 1, Current_Init = Inherited_Best_Log; else, Current_Init = DefaultInit; end
            
            vars = [
                optimizableVariable('log_Q_beta', [2.0, 4.5], 'Type','real');
                optimizableVariable('log_Q_yaw',  [2.0, 4.5], 'Type','real');
                optimizableVariable('log_R_track',[1.0, 5.0], 'Type','real'); 
                optimizableVariable('log_R_rate', [3.0, 4.5], 'Type','real'); 
                optimizableVariable('log_W_slip', [1.0, 5.0], 'Type','real');
            ];
            
            ObjFcn = @(p) Wrapper_ObjFcn(p, ModelName, T_end, Global_Log_File, Vx_kmh, Mu_val, R_path);
            InitialX = table(Current_Init.log_Q_beta, Current_Init.log_Q_yaw, Current_Init.log_R_track, Current_Init.log_R_rate, Current_Init.log_W_slip, 'VariableNames', {'log_Q_beta','log_Q_yaw','log_R_track','log_R_rate','log_W_slip'});
                
            results = bayesopt(ObjFcn, vars, ...
                'IsObjectiveDeterministic', true, ...
                'AcquisitionFunctionName', 'expected-improvement-plus', ...
                'MaxObjectiveEvaluations', Max_Evals, ...
                'InitialX', InitialX, ...
                'UseParallel', false, ...
                'PlotFcn', {@plotMinObjective});
                
            best_p = results.XAtMinObjective;
            Inherited_Best_Log = table2struct(best_p);
            
            [~, Info] = Run_Simulink_Cost_Robust(best_p, ModelName, T_end, Vx_kmh, Mu_val, R_path);
            Real_vals_Final = Calculate_Real_Weights(double(best_p.log_Q_beta), double(best_p.log_Q_yaw), double(best_p.log_R_track), double(best_p.log_R_rate), double(best_p.log_W_slip));
            new_row = table(Vx_kmh, Mu_val, R_path, Real_vals_Final(4), results.MinObjective, Info.SI_max, Info.LatErr_max, Info.Energy_kWh, 'VariableNames', {'V','Mu','R_path','R_rate','Cost','SI','Lat','Energy'});
                
            if isempty(Summary_Table)
                Summary_Table = new_row;
            else
                if ~isequal(Summary_Table.Properties.VariableNames, new_row.Properties.VariableNames), Summary_Table = new_row; else, Summary_Table = [Summary_Table; new_row]; end
            end
            save(Summary_File_Path, 'Summary_Table');
            FileName = sprintf('Result_Obj_%dkmh_Mu%.1f_R%d.mat', Vx_kmh, Mu_val, R_path);
            save(fullfile(Save_Dir, FileName), 'results');
            fprintf(' >>> 结果已保存: %s\n', FileName);
        end
    end
end
fprintf('\n>>> 所有优化完成。数据已保存至: %s\n', Save_Dir);
end
 
function Real_Weights = Calculate_Real_Weights(log_Q_beta, log_Q_yaw, log_R_track, log_R_rate, log_W_slip)
    Real_Weights = [10^log_Q_beta, 10^log_Q_yaw, 10^log_R_track, 10^log_R_rate, 10^log_W_slip];
end
 
function Cost = Wrapper_ObjFcn(p, ModelName, Target_Time, LogFile, Vx, Mu, R_val)
    [Cost, Info] = Run_Simulink_Cost_Robust(p, ModelName, Target_Time, Vx, Mu, R_val);
    Real_Vals = Calculate_Real_Weights(double(p.log_Q_beta), double(p.log_Q_yaw), double(p.log_R_track), double(p.log_R_rate), double(p.log_W_slip));
    
    % ★ 严格按照 21 列写入，包含 Info.Danger ★
    DataRow = [0, Vx, Mu, R_val, double(p.log_Q_beta), Real_Vals(1), double(p.log_Q_yaw), Real_Vals(2), double(p.log_R_track), Real_Vals(3), double(p.log_R_rate), Real_Vals(4), double(p.log_W_slip), Real_Vals(5), Cost, Info.SI_max, Info.LatErr_max, Info.Energy_kWh, Info.Speed_Err_Mean, Info.Jerk, Info.Danger];
    
    try 
        fid = fopen(LogFile, 'a');
        if fid > 0
            fprintf(fid, '%d,%d,%.2f,%d, %.4f,%.4e, %.4f,%.4e, %.4f,%.4e, %.4f,%.4e, %.4f,%.4e, %.4e,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', DataRow);
            fclose(fid);
        end
    catch; end
end
 
function [Cost, Info] = Run_Simulink_Cost_Robust(p, ModelName, Target_Time, Vx, Mu, R_path)
    assignin('base', 'Q_beta_tune', 10^double(p.log_Q_beta));
    assignin('base', 'Q_yaw_tune', 10^double(p.log_Q_yaw));
    assignin('base', 'R_track_tune', 10^double(p.log_R_track));
    assignin('base', 'R_rate_tune', 10^double(p.log_R_rate));
    assignin('base', 'W_slip_tune', 10^double(p.log_W_slip));
    
    % 结构体中初始化 Danger 字段
    Info = struct('SI_max',999, 'LatErr_max',999, 'Energy_kWh',0, 'Speed_Err_Mean',99, 'Jerk',0, 'Danger',0);
    Cost = 1e12;
    
    % =====================================================================
    % ★ 强制加速模式 (Accelerator) 彻底回归 ★
    % =====================================================================
    try
        set_param(ModelName, 'SignalLogging', 'on');
        set_param(ModelName, 'SimCompilerOptimization', 'on'); 
        warning('off', 'Simulink:Engine:Uv2wksNoSignalId');
        
        simOut = sim(ModelName, ...
            'StopTime', numParam(Target_Time), ...
            'SimulationMode', 'accelerator', ... 
            'ReturnWorkspaceOutputs', 'on');
    catch ME
        fprintf(' [SimError] %s\n', ME.message); return; 
    end
    
    v_time = get_time_axis(simOut);
    if isempty(v_time) || v_time(end) < (Target_Time - 1.0)
        fprintf(' [CRASH] Early stop.\n'); return; 
    end
    
    v_si = get_data_simple(simOut, 'SI_Out'); 
    v_lat = get_data_simple(simOut, 'Lat_Err_True'); 
    if isempty(v_lat), v_lat = get_data_simple(simOut, 'Lat_Err'); end
    v_E = get_data_simple(simOut, 'E_total_kWh'); 
    v_vx = get_data_simple(simOut, 'Vx_Out'); 
    
    % =====================================================================
    % ★ 退回只提取左前轮(T_fl)用于 Jerk 计算 ★
    % =====================================================================
    v_Tfl = get_data_simple(simOut, 'Motor_Torque_FL'); 
    
    len_min = min([length(v_time), length(v_si), length(v_lat), length(v_vx), length(v_Tfl)]);
    if len_min == 0, return; end
    v_time = v_time(1:len_min); v_si = v_si(1:len_min); v_lat = v_lat(1:len_min); v_vx = v_vx(1:len_min); v_Tfl = v_Tfl(1:len_min);
    
    Target_Spd_mps = Vx / 3.6; 
    if mean(v_vx) > 30, v_vx_mps = v_vx / 3.6; else, v_vx_mps = v_vx; end
    
    % =====================================================================
    % ★ 稳态评估与【单轮平顺性 Jerk】退回
    % =====================================================================
    steady_time = 2.0; 
    if v_time(end) > steady_time
        steady_idx = v_time >= steady_time; 
        SI_max = max(abs(v_si(steady_idx))); 
        LatErr_max = max(abs(v_lat(steady_idx)));
        Speed_Err_Mean = mean(abs(Target_Spd_mps - v_vx_mps(steady_idx))); 
        
        % 退回：只计算左前轮控制增量的二范数平方
        jerk_metric = mean(diff(v_Tfl(steady_idx)).^2);
    else
        SI_max = max(abs(v_si)); 
        LatErr_max = max(abs(v_lat));
        Speed_Err_Mean = mean(abs(Target_Spd_mps - v_vx_mps)); 
        if length(v_Tfl) > 1, jerk_metric = mean(diff(v_Tfl).^2); else, jerk_metric = 0; end
    end
    
    if isempty(v_E), final_energy = 0; else, final_energy = v_E(end) - v_E(1); end
    Info.SI_max = SI_max; Info.LatErr_max = LatErr_max; 
    Info.Energy_kWh = final_energy; Info.Speed_Err_Mean = Speed_Err_Mean; Info.Jerk = jerk_metric;
    
    if final_energy < 0.001, Cost = 1e11; return; end
    
    % =====================================================================
    % ★ 计算物理先验 Danger，并存入 Info 结构体以便外部记录 ★
    % =====================================================================
    v_mps = Vx / 3.6; 
    if R_path > 10000
        kappa_val = 0; 
        danger_idx = 0;
    else
        a_y_req = (v_mps^2) / R_path; 
        a_y_max = Mu * 9.81; 
        kappa_val = a_y_req / a_y_max; 
        danger_idx = kappa_val;
    end
    
    kappa_for_cost = min(kappa_val, 1.2); 
    Info.Kappa = kappa_for_cost;
    Info.Danger = danger_idx;  % 显式抛出真实的 Danger

    % =====================================================================
    % 多维异构帕累托代价拓扑
    % =====================================================================
    W_Energy_Base = 5.0e8; 
    W_Track_Base  = 2.0e7;
    W_Stab_Base   = 2.0e7;
    W_Speed_Base  = 2.0e7;
    W_Jerk_Base   = 1000;  
    
    kappa_center = 0.74; steepness = 25.0;
    S_factor = 1.0 / (1.0 + exp(-steepness * (kappa_for_cost - kappa_center)));
    
    W_Energy = W_Energy_Base * (1.0 - 0.95 * S_factor);
    W_Speed  = W_Speed_Base  * (1.0 - 0.95 * S_factor);
    W_Stab   = W_Stab_Base * exp(3 * (kappa_for_cost^2));      
    W_Track  = W_Track_Base;                        
    W_Jerk   = W_Jerk_Base;                         
 
    SI_Norm = max(0, SI_max - 0.60) / (1.0 - 0.60);            
    Lat_Norm = max(0, LatErr_max - 0.12) / (0.52 - 0.12);       
    Spd_Norm = max(0, Speed_Err_Mean - 0.20) / (1.2 - 0.20);    
    
    J_speed = W_Speed * (Spd_Norm^2 + 5 * Spd_Norm^4);
    J_track = W_Track * (Lat_Norm^2 + 5 * Lat_Norm^4);
    J_stab  = W_Stab  * (SI_Norm^2  + 5 * SI_Norm^4);
    J_jerk  = W_Jerk * jerk_metric; 
    J_energy = W_Energy * final_energy;
    Cost = J_speed + J_track + J_stab + J_jerk + J_energy;
    
    if isnan(Cost) || isinf(Cost) || Cost > 1e12
        Cost = 1e12;
    end
    
    % 输出时带上 Danger
    fprintf(' Cost=%.2e | Lat=%.3f | SI=%.4f | E=%.4f | SpdErr=%.3fm/s | Danger=%.2f\n', ...
        Cost, LatErr_max, SI_max, final_energy, Speed_Err_Mean, danger_idx);
end

function val = get_data_simple(simOut, varName)
    val = [];
    if isprop(simOut, 'logsout') && ~isempty(simOut.logsout)
        try el = simOut.logsout.getElement(varName); val = double(el.Values.Data(:)); return; catch; end
    end
    if isprop(simOut, varName)
        try val = double(simOut.(varName).Data(:)); return; catch; end
    end
    try if evalin('base', sprintf('exist(''%s'',''var'')', varName)), val = double(evalin('base', varName).Data(:)); end; catch; end
end

function v_time = get_time_axis(simOut)
    try, v_time = simOut.tout; catch, v_time=[]; end
    if isempty(v_time), try, v_time = simOut.logsout{1}.Values.Time; catch; end; end
end

function out = numParam(val)
    if isnumeric(val), out = num2str(val); else, out = val; end
end