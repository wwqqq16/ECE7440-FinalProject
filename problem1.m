% Problem 1: Infinite-Horizon LQR Design (Corrected Version)

% Step 1: Define the CORRECT system matrices A and B

A = [0 1 0;
     0 0 1;
     10 -2 1];  % Corrected system dynamics matrix

B = [0; 0; 3];   % Note the 3 in B(3) from the problem equation

% --------------------------------------------------
% Step 2: Define the cost matrices Q and R
% --------------------------------------------------
Q = diag([2, 4, 1]);  % State weighting matrix
R = 2;                % Input weighting scalar

% --------------------------------------------------
% Step 3: Check controllability
% --------------------------------------------------
C = ctrb(A, B);
rank_C = rank(C);
disp(['Controllability Matrix Rank: ', num2str(rank_C)]);

% --------------------------------------------------
% Step 4: Solve Algebraic Riccati Equation
% --------------------------------------------------
[P, ~, K] = care(A, B, Q, R);   % Solve ARE to get P and K directly
disp('Optimal feedback gain K =');
disp(K);

% Compute closed-loop system matrix
Acl = A - B * K;
eigVals = eig(Acl);
disp('Closed-loop system eigenvalues =');
disp(eigVals);

% --------------------------------------------------
% Step 5: Define CORRECT initial condition
% --------------------------------------------------
x0 = [4; 0; 5];    % Correct initial state

% --------------------------------------------------
% Step 6: Simulate the closed-loop system
% --------------------------------------------------
tspan = 0:0.01:10;           % Time span

% Simulate using ode45
[t, x] = ode45(@(t,x) Acl * x, tspan, x0);

% Compute control input u(t)
u = -x * K';  % Note: K' is 3x1, x is Nx3

% --------------------------------------------------
% Step 7: Plot state trajectories and control input
% --------------------------------------------------
figure;
plot(t, x);
title('State Trajectories (LQR)');
xlabel('Time (s)');
ylabel('State Values');
legend('x1', 'x2', 'x3');
grid on;

figure;
plot(t, u);
title('Control Input u(t) (LQR)');
xlabel('Time (s)');
ylabel('Control Input');
grid on;

% --------------------------------------------------
% Step 8: Compute the cost J numerically
% --------------------------------------------------
J = 0;
dt = t(2) - t(1);
for i = 1:length(t)
    J = J + 0.5 * (x(i,:) * Q * x(i,:)' + R * u(i)^2) * dt;
end
disp(['Optimal cost J = ', num2str(J)]);

%% ------------------------------------------
%  Part 2: Pole Placement Controller
%% ------------------------------------------

disp('--- Pole Placement Controller ---')

% Step 1: Use LQR closed-loop poles as desired poles
desired_poles = eigVals'; % Use the LQR poles

% Step 2: Compute state feedback gain using place()
K_pp = place(A, B, desired_poles);

% Step 3: Define closed-loop system matrix
Acl_pp = A - B * K_pp;

% Step 4: Simulate the closed-loop system with the same initial condition
[t_pp, x_pp] = ode45(@(t,x) Acl_pp * x, tspan, x0);

% Step 5: Compute control input u = -Kx
u_pp = -x_pp * K_pp';

% Step 6: Plot state trajectories
figure;
plot(t_pp, x_pp);
title('State Trajectories (Pole Placement)');
xlabel('Time (s)');
ylabel('State Values');
legend('x1', 'x2', 'x3');
grid on;

% Step 7: Plot control input
figure;
plot(t_pp, u_pp);
title('Control Input u(t) (Pole Placement)');
xlabel('Time (s)');
ylabel('Control Input');
grid on;

% Step 8: Compute cost numerically
J_pp = 0;
dt_pp = t_pp(2) - t_pp(1);
for i = 1:length(t_pp)
    J_pp = J_pp + 0.5 * (x_pp(i,:) * Q * x_pp(i,:)' + R * u_pp(i)^2) * dt_pp;
end
disp(['Pole Placement Controller Cost J = ', num2str(J_pp)]);

