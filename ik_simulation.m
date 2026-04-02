%% ik_simulation.m
% 3-RRPS 逆运动学（基于题图公式(1)-(9)）
% 目标：
% 1) 计算三条支链长度 d_i
% 2) 计算主动转角 theta_i
% 3) 画出一个简易空间模型
% 4) 通过随机采样绘制末端可达工作空间点云

clear; clc; close all;

%% 1) 参数设置
% 平台半径（可按实际机构修改）
g = 0.10;   % fixed platform radius (m)
h = 0.075;  % moving platform radius (m)

% 固定采用图5中的对称 3-RRPS 附着角（不做模式切换）
gamma_deg = [-30, 90, 210];
gamma_i = deg2rad(gamma_deg);

% 末端期望位姿 p=[x y z]^T 与欧拉角(alpha,beta,gamma)
% 采用文中 x-y-z 顺序（对应公式(1)）
p = [0.02; -0.01; 0.45];   % m
alpha = deg2rad(5);        % rad
beta  = deg2rad(3);        % rad
gamma = deg2rad(-2);       % rad

%% 2) 公式(1)：^A_B R
ca = cos(alpha); sa = sin(alpha);
cb = cos(beta);  sb = sin(beta);
cg = cos(gamma); sg = sin(gamma);

A_BR = [ cb*cg,              -cb*sg,            sb; ...
         cg*sb*sa + sg*ca,  -sg*sb*sa + cg*ca, -cb*sa; ...
        -cg*sb*ca + sg*sa,   sg*sb*ca + cg*sa,  cb*ca];

%% 3) 公式(2)-(4)：构造 a_i, b_i, d_i 向量与中间坐标 x_i,y_i,z_i
nLegs = numel(gamma_i);
a = zeros(3,nLegs);   % fixed platform attachment points in {A}
b = zeros(3,nLegs);   % moving platform attachment points in {B}
D = zeros(3,nLegs);   % d_i vectors in {A}

for i = 1:nLegs
    gi = gamma_i(i);
    a(:,i) = [g*cos(gi); g*sin(gi); 0];
    b(:,i) = [h*cos(gi); h*sin(gi); 0];

    % d_i = p + ^A_B R * b_i - a_i
    D(:,i) = p + A_BR*b(:,i) - a(:,i);
end

% 与公式(3)(4)符号一致：d_i = [x-x_i, y-y_i, z-z_i]^T
x_minus_xi = D(1,:);
y_minus_yi = D(2,:);
z_minus_zi = D(3,:);

%% 4) 公式(5)：支链长度
leg_len = sqrt(x_minus_xi.^2 + y_minus_yi.^2 + z_minus_zi.^2);

%% 5) 公式(8)(7) + 几何关系：求主动转角 theta_i
% ^A_Ci R（文中公式(8)）
% d_Ci = (^A_Ci R)^T * d_A
% 文中公式(6)等价写法：d_Ci = [d_i*sin(phi_i); -d_i*sin(theta_i)*cos(phi_i); d_i*cos(theta_i)*cos(phi_i)]
% 因此可消去 phi_i，得到 theta_i = atan2(-d_Ci(2), d_Ci(3))

theta = zeros(1,nLegs);
d_ci_all = zeros(3,nLegs);
for i = 1:nLegs
    gi = gamma_i(i);
    A_CiR = [cos(gi), -sin(gi), 0; ...
             sin(gi),  cos(gi), 0; ...
             0,        0,       1];

    d_ci = A_CiR.' * D(:,i);  % {A}->{Ci}
    d_ci_all(:,i) = d_ci;

    theta(i) = atan2(-d_ci(2), d_ci(3));
end

%% 6) 输出结果
fprintf('\n=== 3-RRPS IK 结果（symmetric 3-RRPS）===\n');
fprintf('附着角 gamma_i (deg): ');
fprintf('%8.2f', gamma_deg);
fprintf('\n\n支链长度 d_i (m):\n');
disp(leg_len.');

fprintf('主动转角 theta_i (deg):\n');
disp(rad2deg(theta).');

%% 7) 简易模型绘图
figure('Name','3-RRPS Simple Model','Color','w');
hold on; grid on; axis equal; view(3);

% 固定平台/动平台附着点（动平台需变换到{A}）
b_A = p + A_BR*b;

% 平台轮廓（闭合）
a_poly = [a, a(:,1)];
b_poly = [b_A, b_A(:,1)];

h1 = plot3(a_poly(1,:), a_poly(2,:), a_poly(3,:), 'k-', 'LineWidth',1.8);
h2 = plot3(b_poly(1,:), b_poly(2,:), b_poly(3,:), 'b-', 'LineWidth',1.8);

% 三条支链
for i = 1:nLegs
    hi = plot3([a(1,i), b_A(1,i)], [a(2,i), b_A(2,i)], [a(3,i), b_A(3,i)], ...
               'r--', 'LineWidth',1.5);
    if i==1
        hLeg = hi;
    end

    % 标号
    % 若工作目录存在同名脚本 text.m，会屏蔽 MATLAB 内置 text 函数；
    % 使用 builtin 可强制调用内置函数，避免 “不支持将脚本 text 作为函数执行”。
    builtin('text', a(1,i),   a(2,i),   a(3,i),   sprintf('  A_%d',i), 'Color','k');
    builtin('text', b_A(1,i), b_A(2,i), b_A(3,i), sprintf('  B_%d',i), 'Color','b');
end

% 中心点
h3 = plot3(0,0,0,'ko','MarkerFaceColor','k');
h4 = plot3(p(1),p(2),p(3),'bo','MarkerFaceColor','b');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend([h1 h2 hLeg h3 h4], ...
       {'Fixed platform','Moving platform','Legs','Base center','Moving center'}, ...
       'Location','bestoutside');
title('3-RRPS IK Simple Model (symmetric)');

%% 8) 工作空间随机采样绘制（位置工作空间）
% 说明：
% - 随机采样末端位姿 (x,y,z,alpha,beta,gamma)
% - 通过逆运动学计算 d_i 与 theta_i
% - 满足约束则记录该位置点 p 作为可达点

N = 30000;  % 采样数，可按电脑性能调整

% 采样范围（可根据机构尺寸再调）
x_rng = [-0.12, 0.12];
y_rng = [-0.12, 0.12];
z_rng = [0.22, 0.62];
ang_rng_deg = [-15, 15];    % alpha,beta,gamma 共用角度范围

% 约束（示例）
d_min = 0.20; d_max = 0.75;          % 支链长度约束 (m)
theta_lim = deg2rad(60);             % 主动转角约束 |theta_i| <= 60°

ws_points = zeros(3, N);
count = 0;

for k = 1:N
    % 1) 随机位姿
    p_k = [x_rng(1) + (x_rng(2)-x_rng(1))*rand; ...
           y_rng(1) + (y_rng(2)-y_rng(1))*rand; ...
           z_rng(1) + (z_rng(2)-z_rng(1))*rand];
    alpha_k = deg2rad(ang_rng_deg(1) + (ang_rng_deg(2)-ang_rng_deg(1))*rand);
    beta_k  = deg2rad(ang_rng_deg(1) + (ang_rng_deg(2)-ang_rng_deg(1))*rand);
    gamma_k = deg2rad(ang_rng_deg(1) + (ang_rng_deg(2)-ang_rng_deg(1))*rand);

    % 2) 构造旋转矩阵 ^A_B R（同公式(1)）
    ca = cos(alpha_k); sa = sin(alpha_k);
    cb = cos(beta_k);  sb = sin(beta_k);
    cg = cos(gamma_k); sg = sin(gamma_k);
    A_BR_k = [ cb*cg,              -cb*sg,            sb; ...
               cg*sb*sa + sg*ca,  -sg*sb*sa + cg*ca, -cb*sa; ...
              -cg*sb*ca + sg*sa,   sg*sb*ca + cg*sa,  cb*ca];

    % 3) IK 计算 d_i 与 theta_i
    D_k = zeros(3,nLegs);
    theta_k = zeros(1,nLegs);
    for i = 1:nLegs
        gi = gamma_i(i);
        D_k(:,i) = p_k + A_BR_k*b(:,i) - a(:,i);

        A_CiR = [cos(gi), -sin(gi), 0; ...
                 sin(gi),  cos(gi), 0; ...
                 0,        0,       1];
        d_ci = A_CiR.' * D_k(:,i);
        theta_k(i) = atan2(-d_ci(2), d_ci(3));
    end
    d_k = vecnorm(D_k, 2, 1);

    % 4) 约束判断
    len_ok = all(d_k >= d_min & d_k <= d_max);
    theta_ok = all(abs(theta_k) <= theta_lim);

    if len_ok && theta_ok
        count = count + 1;
        ws_points(:,count) = p_k;
    end
end

ws_points = ws_points(:,1:count);
fprintf('\n随机采样完成：有效点 %d / %d\n', count, N);

figure('Name','3-RRPS Workspace (Random Sampling)','Color','w');
hold on; grid on; axis equal; view(3);
if count > 0
    scatter3(ws_points(1,:), ws_points(2,:), ws_points(3,:), 8, ws_points(3,:), 'filled');
    colormap(turbo); colorbar;
else
    warning('未采到满足约束的点，请放宽约束或增大采样范围。');
end

% 叠加固定平台轮廓作为参考
plot3(a_poly(1,:), a_poly(2,:), a_poly(3,:), 'k-', 'LineWidth', 1.5);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3-RRPS Workspace via Random Sampling');
