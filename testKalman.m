
num_pts = 100;  % numver of points
r = 1;  % Radius of the spiral
p = 0.1;  % Pitch of the spiral
t = (linspace(0, 10*pi, num_pts))';  % Time vector
f = 0.5;   % frequency of points

% Spiral coordinates
x = r*cos(f*t);
y = r*sin(f*t);
z = p*t;

% add noise to observations
x = x + 0.03*randn(num_pts,1);
y = y + 0.03*randn(num_pts,1);
z = z + 0.03*randn(num_pts,1);

% Plot the spiral
figure(1)
plot3(x, y, z)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Trajectory')

% time and positions differences
dt = 1;
dx = diff(x);
dy = diff(y);
dz = diff(z);

% allocate filtered results
x_f = zeros(num_pts,1);
y_f = zeros(num_pts,1);
z_f = zeros(num_pts,1);


% initialize filters
[k_H, k_R, k_P] = initKalman();

% filter observations
for k = 1:num_pts
    %input vectors    
    if k == 1
        %initial velocities are 0
        dx_k = 0;
        dy_k = 0;
        dz_k = 0;
    else
        dx_k = dx(k-1);
        dy_k = dy(k-1);
        dz_k = dz(k-1);        
    end
    % observation vector
    k_z = [x(k); y(k); z(k)];
    k_x = [k_z; dx_k; dy_k; dz_k];
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
plot3(x_f, y_f, z_f)
hold on;
plot3(x, y, z)
hold off
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Trajectory')
legend('Noisy Observations', 'Filtered Coordionates')

