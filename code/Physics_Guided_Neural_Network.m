function Train_BPNN_Ultimate_Residual()
% =========================================================================
% 终极残差非对称 BPNN 训练代码 —— 放宽极端工况版 (Data-Driven 终极形态)
% =========================================================================
    clc; warning off; rng(42, 'twister');
    
    % 1. 读取数据 (请确保路径正确)
    csvFile = 'D:\Carsim2019\DATE\Opt_Result_v17_Final_100\Optimization_History_All_Steps_with_Danger.csv';
    fprintf('>>> 正在读取数据: %s\n', csvFile);
    data = readtable(csvFile);
    
    % 2. ★★★ 核心修复：极度放宽数据清洗边界 ★★★
    % 绝不能用 1.49 切断！极端冰雪工况被救回来的 SI_max 可能高达 2.5~3.5
    % 只要 Cost 没有爆表到 1e10(撞墙)，我们就认为 BO 找到的参数是有价值的！
    data.Kappa = 1.0 ./ max(abs(data.R_path), 1e-1); 
    valid_idx = data.SI_max < 4.0 & data.Cost < 1e10; 
    data = data(valid_idx, :);
    
    % 3. 提取各个工况下的全局最优解
    [G, ~, ~, ~] = findgroups(data.V_set, data.Mu, data.Kappa);
    num_conds = max(G);
    fprintf('>>> 提取有效独立工况数: %d\n', num_conds);
    
    X_train = zeros(4, num_conds);
    Y_train = zeros(5, num_conds);
    
    for i = 1:num_conds
        sub_data = data(G == i, :);
        % 在该工况的所有尝试中，找到 Cost 最低的那个作为金标准
        [~, best_idx] = min(sub_data.Cost);
        best_label = sub_data(best_idx, :);
        
        % 计算危险度
        danger = min(max(mean(sub_data.Danger), 0), 1);
        
        X_train(:, i) = [mean(sub_data.V_set); mean(sub_data.Mu); mean(sub_data.Kappa); danger];
        
        % 物理基线设计 (完全对齐你 Simulink 里的推导)
        base_R = 4.5 - 3.5 * danger;
        base_W = 1.0 + 3.5 * danger;
        
        % 严格对齐 MPC 顺序: 1:Q_beta, 2:Q_yaw, 3:R_alloc, 4:R_rate, 5:W_slip
        % W_slip 和 R_alloc 训练残差，其余训练绝对值
        Y_train(:, i) = [best_label.log_Q_beta; 
                         best_label.log_Q_yaw; 
                         best_label.log_R_track - base_R; 
                         best_label.log_R_rate; 
                         best_label.log_W_slip - base_W];
    end
    
    % 4. 建立并配置神经网络
    % 稍微加宽网络容量 [16, 16]，确保它能记住新加入的那些极其刁钻的极端工况
    net = fitnet([16, 16], 'trainlm');
    net.trainParam.showWindow = true;
    net.trainParam.epochs = 1500;
    net.trainParam.goal = 1e-5;
    net.trainParam.min_grad = 1e-7;
    net.trainParam.max_fail = 20; 
    
    % 5. 开始训练
    fprintf('>>> 开始训练包含极限救车数据的神经网络...\n');
    net = train(net, X_train, Y_train);
    
    % 6. 生成 Simulink 可用的代码
    genFunction(net, 'BPNN_Core_Ultimate_Res', 'MatrixOnly', 'yes');
    fprintf('>>> ★ BPNN_Core_Ultimate_Res.m 已生成完毕！★\n');
end