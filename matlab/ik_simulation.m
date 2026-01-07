%% ik_simulation.m
% 逆运动学计算流程示例
% 说明：脚本按公式(1)-(9)组织计算步骤，变量命名与题目描述保持一致。

clear; clc; close all;

%% 1. 定义参数
% 固定/动平台半径
 g = 0.35;     % 固定平台半径
 h = 0.18;     % 动平台半径

% 臂附着角 gamma_i (固定平台与动平台按相同顺序布置)
% 这里给出常用对称六点布置
Gamma = deg2rad([10, 110, 130, 230, 250, 350]);

% 欧拉角 (ZYX: yaw-pitch-roll)
alpha = deg2rad(5);   % yaw
beta  = deg2rad(3);   % pitch
gamma = deg2rad(-2);  % roll

% 平台位置 p = [x y z]^T
p = [0.02; -0.01; 0.45];

% 坐标系定义：
% {A} 固定平台坐标系，{B} 动平台坐标系。
% 设定各附着点位于各自平台的 XY 平面，Z=0。

%% 2. 按公式(1)构造从 {B} 到 {A} 的旋转矩阵 A_BR
A_BR = rotz(alpha) * roty(beta) * rotx(gamma);

%% 3. 计算 a_i 与 b_i，并按公式(2)-(4)得到 d_i 与 x_i,y_i,z_i
nLegs = numel(Gamma);
a = zeros(3, nLegs);
b = zeros(3, nLegs);

for i = 1:nLegs
    % 固定平台点 a_i (在 {A} 坐标系)
    a(:, i) = [g * cos(Gamma(i)); g * sin(Gamma(i)); 0];
    % 动平台点 b_i (在 {B} 坐标系)
    b(:, i) = [h * cos(Gamma(i)); h * sin(Gamma(i)); 0];
end

% 按公式(2)-(4)计算 d_i 和分量
% d_i = p + A_BR * b_i - a_i
D = zeros(3, nLegs);
for i = 1:nLegs
    D(:, i) = p + A_BR * b(:, i) - a(:, i);
end
x_i = D(1, :);
y_i = D(2, :);
z_i = D(3, :);

%% 4. 计算执行器长度 d_i (公式(5))
leg_length = sqrt(x_i.^2 + y_i.^2 + z_i.^2);

%% 5. 构造 A_CiR (公式(8))，按公式(9)求解主动关节角 theta_i
% 假设每条执行器的局部坐标系 {Ci}：
% x 轴沿着固定平台圆周切向旋转 gamma_i，z 轴与 {A} 同向。
% A_CiR 将 {A} 坐标系向 {Ci} 旋转
A_CiR = zeros(3, 3, nLegs);
theta = zeros(1, nLegs);

for i = 1:nLegs
    A_CiR(:, :, i) = rotz(Gamma(i));
    d_ci = A_CiR(:, :, i).' * D(:, i); % {A} -> {Ci}
    % 主动关节角 theta_i (绕 {Ci} 的 y 轴转动示例)
    theta(i) = atan2(d_ci(3), d_ci(1));
end

%% 6. 示例：打印结果并绘制关节角随平台位姿变化曲线
fprintf('=== 执行器长度 d_i ===\n');
disp(leg_length.');

fprintf('=== 主动关节角 theta_i (rad) ===\n');
disp(theta.');

% 令平台沿 Z 方向平移，绘制关节角变化
z_list = linspace(0.35, 0.55, 30);
theta_z = zeros(nLegs, numel(z_list));

for k = 1:numel(z_list)
    p_k = [p(1); p(2); z_list(k)];
    for i = 1:nLegs
        D_k = p_k + A_BR * b(:, i) - a(:, i);
        d_ci = A_CiR(:, :, i).' * D_k;
        theta_z(i, k) = atan2(d_ci(3), d_ci(1));
    end
end

figure('Name', 'Joint Angles vs Platform Z');
plot(z_list, theta_z.');
grid on;
xlabel('Platform z (m)');
ylabel('\theta_i (rad)');
legend(arrayfun(@(i) sprintf('\\theta_%d', i), 1:nLegs, 'UniformOutput', false));

title('主动关节角随平台 Z 位移变化');

%% 辅助函数
function R = rotx(phi)
R = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
end

function R = roty(theta)
R = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
end

function R = rotz(psi)
R = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
end
