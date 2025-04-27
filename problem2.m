
%% Problem 2: Minimum-Time Control
clear; clc; close all;

%% (1) Solve system equations for u = -1 and u = 1
A = [0 1; 0 -2];
B = [0; 2];

% Time vector
t = linspace(0, 5, 500);

% Grid of initial conditions
[x10, x20] = meshgrid(-10:2:10, -10:2:10);

%% (2) Sketch phase plane trajectories for u = ±1
figure(1); clf; hold on;

% u = +1 trajectories (blue)
for i = 1:numel(x10)
    [~, x] = ode45(@(t,x) A*x + B*1, t, [x10(i); x20(i)]);
    plot(x(:,1), x(:,2), 'b');
end

% u = -1 trajectories (red)
for i = 1:numel(x10)
    [~, x] = ode45(@(t,x) A*x + B*(-1), t, [x10(i); x20(i)]);
    plot(x(:,1), x(:,2), 'r');
end

xlabel('$x_1$', 'Interpreter', 'latex');
ylabel('$x_2$', 'Interpreter', 'latex');
title('Phase Plane Trajectories for $u = \pm1$', 'Interpreter', 'latex');
legend('$u = +1$', '$u = -1$', 'Interpreter', 'latex');
grid on;
axis equal;

%% (3) Use Pontryagin's Minimum Principle
disp('Optimal Control Law by PMP: u*(t) = -sign(p2(t))');

%% (4) Find the switching curve
t_switch = linspace(0,5,500);

% Positive branch (u = -1)
x1_switch_pos = (exp(-2*t_switch) + 2*t_switch - 1)/2;
x2_switch_pos = 1 - exp(-2*t_switch);

% Negative branch (u = +1)
x1_switch_neg = (-exp(-2*t_switch) - 2*t_switch + 1)/2;
x2_switch_neg = -1 + exp(-2*t_switch);

figure(2); clf;
plot(x1_switch_pos, x2_switch_pos, 'k-', 'LineWidth', 2); hold on;
plot(x1_switch_neg, x2_switch_neg, 'k-', 'LineWidth', 2);
xlabel('$x_1$', 'Interpreter', 'latex');
ylabel('$x_2$', 'Interpreter', 'latex');
title('Switching Curve', 'Interpreter', 'latex');
grid on;
axis equal;

disp('Switching curve plotted successfully.');

%% (5) For x(0) = [10; 0]
x0 = [10; 0];

% (5a) Derive optimal control u(t)
[t1, x1] = ode45(@(t,x) A*x + B*(-1), [0 5], x0);

% (修正后的) 找到最近的 switching curve
dist_pos = vecnorm([x1(:,1) - interp1(t_switch, x1_switch_pos, t1, 'linear', 'extrap'), ...
                    x1(:,2) - interp1(t_switch, x2_switch_pos, t1, 'linear', 'extrap')], 2, 2);
dist_neg = vecnorm([x1(:,1) - interp1(t_switch, x1_switch_neg, t1, 'linear', 'extrap'), ...
                    x1(:,2) - interp1(t_switch, x2_switch_neg, t1, 'linear', 'extrap')], 2, 2);
[~, idx_switch] = min(min(dist_pos, dist_neg));
t_switch_point = t1(idx_switch);

% 判断是否需要第二段积分
if abs(t1(idx_switch) - 5) < 1e-6
    % Already at final time
    t_opt = t1(1:idx_switch);
    x_opt = x1(1:idx_switch,:);
    u_opt = -ones(idx_switch,1); % all u = -1
else
    % Need second phase
    [t2, x2] = ode45(@(t,x) A*x + B*(1), [t1(idx_switch) 5], x1(idx_switch,:)');
    t_opt = [t1(1:idx_switch); t2];
    x_opt = [x1(1:idx_switch,:); x2];
    u_opt = [-ones(idx_switch,1); ones(length(t2),1)];
end

fprintf('Switching occurs at t = %.4f seconds.\n', t_switch_point);
fprintf('Total time to reach origin: %.4f seconds.\n', t_opt(end));

%% (5b) Plot optimal state trajectory on phase plane
figure(3); clf; hold on;
plot(x1_switch_pos, x2_switch_pos, 'k--', 'LineWidth', 1.5);
plot(x1_switch_neg, x2_switch_neg, 'k--', 'LineWidth', 1.5);
plot(x_opt(:,1), x_opt(:,2), 'b-', 'LineWidth', 2);
plot(x0(1), x0(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(0, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('$x_1$', 'Interpreter', 'latex');
ylabel('$x_2$', 'Interpreter', 'latex');
title('Optimal State Trajectory (Phase Plane)', 'Interpreter', 'latex');
legend('Switching Curve', 'Optimal Trajectory', 'Initial State', 'Origin', 'Location', 'best');
grid on;
axis equal;

%% (5c) Plot time trajectories of x1(t), x2(t), u(t)
figure(4); clf;
subplot(3,1,1);
plot(t_opt, x_opt(:,1), 'b', 'LineWidth', 1.5);
ylabel('$x_1(t)$', 'Interpreter', 'latex');
grid on;
title('Optimal State and Control Trajectories');

subplot(3,1,2);
plot(t_opt, x_opt(:,2), 'r', 'LineWidth', 1.5);
ylabel('$x_2(t)$', 'Interpreter', 'latex');
grid on;

subplot(3,1,3);
stairs(t_opt, u_opt, 'k', 'LineWidth', 1.5);
ylabel('$u(t)$', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');
ylim([-1.5 1.5]);
grid on;