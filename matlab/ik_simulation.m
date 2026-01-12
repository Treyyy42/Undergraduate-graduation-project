%% ik_simulation.m
% 逆运动学计算流程示例
% 说明：脚本按公式(1)-(9)组织计算步骤，变量命名与题目描述保持一致。

clear; clc; close all;

%% 1. 定义参数
% 固定/动平台半径 (单位: m)
g = 0.10;     % 固定平台半径
h = 0.075;    % 动平台半径

% 臂附着角 gamma_i (固定平台与动平台按相同顺序布置)
% 这里给出对称三点布置 (0°, 120°, 240°)
Gamma = deg2rad([0, 120, 240]);

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
ca = cos(alpha); sa = sin(alpha);
cb = cos(beta); sb = sin(beta);
cg = cos(gamma); sg = sin(gamma);

A_BR = [ca * cb, ca * sb * sg - sa * cg, ca * sb * cg + sa * sg; ...
        sa * cb, sa * sb * sg + ca * cg, sa * sb * cg - ca * sg; ...
        -sb,     cb * sg,              cb * cg];

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
    cgi = cos(Gamma(i));
    sgi = sin(Gamma(i));
    A_CiR(:, :, i) = [cgi, -sgi, 0; sgi, cgi, 0; 0, 0, 1];
    d_ci = A_CiR(:, :, i).' * D(:, i); % {A} -> {Ci}
    % 主动关节角 theta_i (绕 {Ci} 的 y 轴转动示例)
    theta(i) = atan2(d_ci(3), d_ci(1));
end

%% 6. 示例：打印结果
fprintf('=== 执行器长度 d_i ===\n');
disp(leg_length.');

fprintf('=== 主动关节角 theta_i (rad) ===\n');
disp(theta.');

%% 7. 绘制机构简图（固定/动平台与三条支链）
figure('Name', '3-RRPS Mechanism Schematic');
hold on; grid on; axis equal;

% 固定平台与动平台点（在 {A} 坐标系下）
a_pts = a;
b_pts_A = p + A_BR * b;

% 关闭多边形以绘制平台轮廓
a_poly = [a_pts, a_pts(:, 1)];
b_poly = [b_pts_A, b_pts_A(:, 1)];

h_fixed = plot3(a_poly(1, :), a_poly(2, :), a_poly(3, :), 'k-', 'LineWidth', 1.5);
h_moving = plot3(b_poly(1, :), b_poly(2, :), b_poly(3, :), 'b-', 'LineWidth', 1.5);

% 绘制支链连线
for i = 1:nLegs
    h_leg_current = plot3([a_pts(1, i), b_pts_A(1, i)], ...
                          [a_pts(2, i), b_pts_A(2, i)], ...
                          [a_pts(3, i), b_pts_A(3, i)], 'r--', 'LineWidth', 1.2);
    if i == 1
        h_leg = h_leg_current;
    end
end

% 绘制平台中心
h_base = plot3(0, 0, 0, 'ko', 'MarkerFaceColor', 'k');
h_platform = plot3(p(1), p(2), p(3), 'bo', 'MarkerFaceColor', 'b');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend([h_fixed, h_moving, h_leg, h_base, h_platform], ...
       {'Fixed platform', 'Moving platform', 'Legs', 'Base center', 'Platform center'}, ...
       'Location', 'bestoutside');
title('3-RRPS 机构简图');
