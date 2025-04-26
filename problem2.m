%% Problem 2: Minimum-Time Control Implementation

clc;
clear;
close all;

%% System Definition
% Define a double integrator system: x1 = position, x2 = velocity
% State-space representation: 
%   dx1/dt = x2
%   dx2/dt = u
% where u is constrained by |u| <= u_max

A = [0 1; 0 0];    % System matrix
B = [0; 1];         % Input matrix
x0 = [5; 0];        % Initial state [position; velocity]
x_target = [0; 0];  % Target state (origin)
u_max = 1;          % Maximum control effort (bang-bang control)

%% Simulation Parameters
tspan = 0:0.01:10;  % Time vector for simulation
switch_tol = 1e-3;  % Tolerance for detecting switching time

%% Minimum-Time Control Law
% The control law follows bang-bang principle:
% u = +u_max when below switching curve
% u = -u_max when above switching curve
% Switching occurs on the parabolic switching curve:
% x1 + 0.5*x2*|x2|/u_max = 0

% Preallocate storage
x = zeros(length(tspan), 2);  % State trajectory [x1, x2]
u = zeros(length(tspan), 1);  % Control history
x(1,:) = x0;                  % Set initial state

% Simulate using Euler integration
for k = 1:length(tspan)-1
    current_x = x(k,:)';
    
    % Compute switching function
    switching_func = current_x(1) + 0.5*current_x(2)*abs(current_x(2))/u_max;
    
    % Bang-bang control law
    if switching_func > 0
        u(k) = -u_max;  % Above switching curve -> negative max control
    else
        u(k) = u_max;   % Below switching curve -> positive max control
    end
    
    % State update (Euler integration)
    x(k+1,:) = x(k,:) + (A*x(k,:)' + B*u(k))'*(tspan(2)-tspan(1));
end

%% Plot Results

% State Trajectories
figure;
subplot(2,1,1);
plot(tspan, x(:,1), 'LineWidth', 2);
hold on;
plot(tspan, x(:,2), 'LineWidth', 2);
title('State Trajectories for Minimum-Time Control');
xlabel('Time (seconds)');
ylabel('State Values');
legend('Position (x1)', 'Velocity (x2)');
grid on;

% Control Input
subplot(2,1,2);
stairs(tspan, u, 'LineWidth', 2, 'Color', [0.8 0.2 0.2]);
title('Control Input');
xlabel('Time (seconds)');
ylabel('Control Effort (u)');
ylim([-1.2*u_max 1.2*u_max]);
grid on;

%% Phase Portrait Analysis
% Plot the state trajectory in phase space (x1 vs x2)
figure;
plot(x(:,1), x(:,2), 'LineWidth', 2);
hold on;

% Plot switching curve
x2_vals = linspace(min(x(:,2)), max(x(:,2)), 100);
switching_curve = -0.5*x2_vals.*abs(x2_vals)/u_max;
plot(switching_curve, x2_vals, '--', 'LineWidth', 1.5);

title('Phase Portrait with Switching Curve');
xlabel('Position (x1)');
ylabel('Velocity (x2)');
legend('State Trajectory', 'Switching Curve');
grid on;

%% Performance Metrics
% Calculate time to reach target (within 1% tolerance)
reached_target = find(sqrt(x(:,1).^2 + x(:,2).^2) < 0.01*sqrt(x0'*x0), 1);
if ~isempty(reached_target)
    min_time = tspan(reached_target);
    fprintf('Minimum time to reach target: %.3f seconds\n', min_time);
else
    fprintf('Target not reached within simulation time\n');
end

%% Verification
% Compare with theoretical minimum time
% For double integrator: t_min = (|x2(0)| + sqrt(x2(0)^2 + 2*u_max*|x1(0)|))/u_max
theoretical_min_time = (abs(x0(2)) + sqrt(x0(2)^2 + 2*u_max*abs(x0(1))))/u_max;
fprintf('Theoretical minimum time: %.3f seconds\n', theoretical_min_time);