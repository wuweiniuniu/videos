                                                                                                                                                                                                                                                                                                                                                                                %% 电机数据精准缝合脚本 (适配第83行空行分割)
%clc; clear;

% 1. 读取原始文件
filename = 'C:\Users\zlm22\Desktop\制动和驱动效率.xlsx'; 
fprintf('>>> 正在读取文件: %s ...\n', filename);

try
    raw_data = readmatrix(filename);
catch
    error('无法读取文件！请检查路径。');
end

% 2. 分离轴和数据体
% Excel结构: Row1=Speed, Col1=Torque
raw_speed_row = raw_data(1, 2:end);   % 第一行 (Speed RPM)
raw_torque_col = raw_data(2:end, 1);  % 第一列 (Torque Nm)
raw_map_body   = raw_data(2:end, 2:end); % 中间的数据体

% 3. 【核心步骤】剔除空行 (第83行) 并缝合数据
% 逻辑：只要转矩那一列不是 NaN，就是有效行（不管是制动还是驱动）
valid_row_idx = ~isnan(raw_torque_col);

% 提取清洗后的转矩列 (包含负数和正数)
clean_torque_nm = raw_torque_col(valid_row_idx);
% 提取清洗后的 Map (对应行)
clean_map_body  = raw_map_body(valid_row_idx, :);

% 处理转速轴 (去掉末尾可能的 NaN)
clean_speed_rpm = raw_speed_row(~isnan(raw_speed_row));
% 截取 Map 的有效列
clean_map_body = clean_map_body(:, 1:length(clean_speed_rpm));

% 4. 排序 (Sort) - 关键！
% 原始数据可能是：[-800... -10] (空行) [0 ... 800]
% 我们需要确保转矩是从小到大单调递增的 (-800 -> 800)
[final_torque_nm, sort_idx] = sort(clean_torque_nm, 'ascend');
final_map_body = clean_map_body(sort_idx, :);

% 5. 单位换算 (RPM -> rad/s, % -> 0-1)
% A. 转速换算
speed_axis_rad = clean_speed_rpm * (2 * pi / 60);

% B. 效率换算
% 如果最大值大于 1，说明是百分比
if max(final_map_body(:)) > 1.0
    fprintf('  [单位修正] 效率为 0-100%%，正在转换为 0-1...\n');
    map_norm = final_map_body / 100.0;
else
    map_norm = final_map_body;
end

% C. 异常值清洗 (NaN/0 -> 0.01)
map_norm(isnan(map_norm)) = 0.01;
map_norm(map_norm < 0.01) = 0.01;
map_norm(map_norm > 0.98) = 0.98;

% 6. 维度转置 (适配 Simulink)
% 当前 map_norm 是 [Torque(行) x Speed(列)]
% Simulink 2D Lookup Table 通常期望 [Speed(行) x Torque(列)] (如果第一个输入是Speed)
% 所以我们【转置】它
map_for_simulink = map_norm';

% 7. 封装并保存
MotorData.Speed_Axis  = reshape(speed_axis_rad, 1, []);   % 行向量
MotorData.Torque_Axis = reshape(final_torque_nm, 1, []);  % 行向量
MotorData.Eff_Map     = map_for_simulink; % [Speed x Torque]

save('MotorData_Final.mat', 'MotorData');

%% 8. 结果验证
fprintf('>>> 处理完成！\n');
fprintf('  转速范围: %.1f ~ %.1f rad/s (原 %.0f~%.0f RPM)\n', ...
    min(speed_axis_rad), max(speed_axis_rad), min(clean_speed_rpm), max(clean_speed_rpm));
fprintf('  转矩范围: %.1f ~ %.1f Nm\n', min(final_torque_nm), max(final_torque_nm));
fprintf('  Map 尺寸: %d(Speed) x %d(Torque)\n', size(map_for_simulink));

% 检查是否包含正负
if min(final_torque_nm) < -10 && max(final_torque_nm) > 10
    fprintf('  [完美] 数据已成功缝合制动(负)和驱动(正)区间！\n');
else
    error('  [警告] 数据缺失！请检查第83行是否真的分割了数据。');
end