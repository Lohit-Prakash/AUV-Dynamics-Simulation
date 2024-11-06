% Define parameters
M = diag([1500, 1500, 1500, 145, 4000, 4200]);
M(3,6) = -35;
M(6,3) = 35;

tau = [150; 50; 0; 0; 0; 0];

% Initial Conditions
initial_eta = [0; 0; 100; 0; 0; 0];
initial_v = zeros(6,1);
tspan = [0 150];

dynamics = @(t, state) auv_dynamics(t, state, M, tau);
[t, state] = ode45(dynamics, tspan, [initial_v; initial_eta]);
v = state(:, 1:6);
eta = state(:, 7:12);

%% Plot 
figure;
plot3(eta(:,1), eta(:,2), eta(:,3));
xlabel('X Position'); ylabel('Y Position'); zlabel('Z Position');
title('3D Trajectory of the AUV');

figure;
subplot(3,1,1); plot(t, v(:,1)); title('Linear Velocity in X'); xlabel('Time (s)'); ylabel('Velocity (m/s)');
subplot(3,1,2); plot(t, v(:,2)); title('Linear Velocity in Y'); xlabel('Time (s)'); ylabel('Velocity (m/s)');
subplot(3,1,3); plot(t, v(:,3)); title('Linear Velocity in Z'); xlabel('Time (s)'); ylabel('Velocity (m/s)');

%% Function
function dstate = auv_dynamics(t, state, M, tau)
    v = state(1:6);
    eta = state(7:12);
    
    % Define matrices (based on provided values)
    C = zeros(6,6);  % Assuming C is zero for simplicity
    D = [42.5 -19.8 31.1 0 0 0; 0 92 0 0 0 -1450; 0 0 1500 0 2100 0; 
         0 0 0 0 0 0; 0 0 0 -3200 6700 0; 0 380 0 0 0 1250];
    g_eta = [-205*sin(eta(2));
             205*cos(eta(2))*sin(eta(1));
             205*cos(eta(2))*cos(eta(1));
             220*cos(eta(2))*sin(eta(1)) - 1.1*cos(eta(2))*cos(eta(1));
             22*sin(eta(2)) - 1.1*cos(eta(2))*sin(eta(1));
             1.1*cos(eta(2))*sin(eta(1))];
    
    % Calculate dynamics
    dv = M \ (tau - (C*v + D*v + g_eta));
    deta = v;
    
    % Return derivatives
    dstate = [dv; deta];
end

