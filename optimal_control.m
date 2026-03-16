function main_lqr_control()
m = 0.009;   % 9g in kg
M = 0.020;   % 20g in kg
g = 9.81;
l = 0.25;
    % 1. System Definition: dx/dt = [0 1 0 0; 0 0 -(m.*g)/M 0; 0 0 0 1;0 0 ((M+m).*g)/(l.*M) 0]x + [0; 1/M;0; -1/(l.*M)]u
    A = [0 1 0 0; 
         0 0 -(m.*g)/M 0; 
         0 0 0 1;
         0 0 ((M+m).*g)/(l.*M) 0];
    B = [0; 1/M;0; -1/(l.*M)];

    % 2. Weighting Matrices (Cost Function)
    % min integral(x'Wx + u'Uu)
    W = diag([10, 1, 10, 1]);  % Penalize position & angle more    
    U = 10;         % Weight on control effort

    % 3. Calculate LQR Gain
    % K is the optimal gain matrix such that u = -Kx
    [K, ~, ~] = lqr(A, B, W, U)

    % 4. Simulation Settings
    xinit = [1; 2; 3;4];           % Initial conditions
    tspan = [0 5];              % Time range [start end]
    
    % 5. Solve the Closed-Loop System: dx/dt = A*x + B*(-K*x)
    % Using an anonymous function @(t,x) instead of 'global'
    [t, X] = ode45(@(t, x) (A - B*K)*x, tspan, xinit);

    % 6. Plotting Results
    figure;
    plot(t, X(:,1), 'LineWidth',1.5)
    hold on
    plot(t, X(:,2), 'LineWidth',1.5)
    hold on 
    plot(t, X(:,3), 'LineWidth',1.5)
    hold on 
    plot(t, X(:,4), 'LineWidth',1.5)
    grid on;
    legend('x_1 (cart pos)', 'x_2 (cart vel)', 'x_3 (angle)', 'x_4 (ang vel)');
    xlabel('Time (s)');
    ylabel('State x');
    title('LQR Control Response: State x');
end
