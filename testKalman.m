% @fileName testKalman.m
% @author Melih Altun @2023

num_pts = 1000;  % number of points
r = 1;  % Radius of the spiral
p = 0.1;  % Pitch of the spiral
t = (linspace(0, 10*pi, num_pts))';  % Time vector
f = 0.5;   % frequency of points

% Spiral coordinates
x = r*cos(f*t);
y = r*sin(f*t);
z = p*t;

% add noise to observations
x_n = x + 0.04*randn(num_pts,1);
y_n = y + 0.04*randn(num_pts,1);
z_n = z + 0.04*randn(num_pts,1);

% time difference between observations
dt = 0.1;

% allocate filtered results
x_f = zeros(num_pts,1);
y_f = zeros(num_pts,1);
z_f = zeros(num_pts,1);

% initialize kalman
[k_H, k_R, k_P] = KalmanInit();

% for each observation
for k = 1:num_pts
    % observation vector
    k_z = [x_n(k); y_n(k); z_n(k)];
    if k == 1
        %initial velocities are 0, initial prediction is the observed state
        dx_k = 0;
        dy_k = 0;
        dz_k = 0;
        k_x = [k_z; dx_k; dy_k; dz_k];
    end
    
    % call the filter
    [ k_x, k_P ] = KalmanPredict( k_x, k_P, dt);
    [ k_x, k_P ] = KalmanUpdate( k_z, k_x, k_P, k_R, k_H);
    % save predicted positions
    x_f(k) = k_x(1);
    y_f(k) = k_x(2);
    z_f(k) = k_x(3);
end

% Plot the spiral trajectory
figure(1)
plot3(x_n, y_n, z_n,'r', 'LineWidth', 1.5)
hold on;
plot3(x_f, y_f, z_f,'g','LineWidth', 2)
hold off
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Trajectory')
legend('Noisy Observations','Filtered Coordionates')
grid on

err_observations = sqrt((x_n - x).^2 + (y_n - y).^2 + (z_n - z).^2);
err_filtered = sqrt((x_f - x).^2 + (y_f - y).^2 + (z_f - z).^2);
fprintf('MSE Noisy Observations = %f\n', sum(err_observations.^2)/length(err_observations));
fprintf('MSE Filtered Observations = %f\n', sum(err_filtered.^2)/length(err_observations));
