%% Gaussian S-Curve Multi-Waypoint Trajectory Planner
clear; clc; close all;
fprintf('=== Gaussian S-Curve Multi-Waypoint Trajectory ===\n\n');

%% STEP 1: Define Cartesian Waypoints & Orientation (From 6-7-6 Code)
% Facing Down 
target_rpy = [pi, 0, 0]; 

% Uncomment path to select it
% 4 Cartesian Waypoints (Path 5)
% p_start   = [180, 0, 50];
% p_lift    = [180, 0, 250];  
% p_descent = [180, 120, 200];
% p_end     = [180, 120, 150];
% 
% Path 4
% p_start=[50,150, 80];
% p_lift=[100,100, 220];  
% p_descent=[180, 0,180];
% p_end=[220,-100, 80];
% 
% Path 3
% p_start=[100, -200, 150];
% p_lift=[100,200, 250];  
% p_descent=[50, 180,200];
% p_end=[0,150, 150];

% Path 2
% p_start=[150, -100, 120];
% p_lift=[150,-100, 200];  
% p_descent=[200, -50,160];
% p_end=[100,150, 100];

% Path 1
p_start=[200,100, 100];  
p_lift=[200,100, 180];  
p_descent= [150, 120, 140];  
p_end=[100, 150, 100];


waypoints_cart = [p_start; p_lift; p_descent; p_end];
fprintf('Cartesian Waypoints:\n');
fprintf('  Start:   [%.1f, %.1f, %.1f]\n', p_start);
fprintf('  Lift:    [%.1f, %.1f, %.1f]\n', p_lift);
fprintf('  Descent: [%.1f, %.1f, %.1f]\n', p_descent);
fprintf('  End:     [%.1f, %.1f, %.1f]\n\n', p_end);

%% STEP 2: Solve IK for All 4 Waypoints (Warm-Start Chaining)
fprintf('Solving Inverse Kinematics for 4 waypoints...\n');
% first point
q_seed = zeros(1, 6); 

[theta1, ~] = ilink6dofelephant_hybrid(p_start, target_rpy, q_seed);
[theta2, ~] = ilink6dofelephant_hybrid(p_lift, target_rpy, theta1);
[theta3, ~] = ilink6dofelephant_hybrid(p_descent, target_rpy, theta2);
[theta4, ~] = ilink6dofelephant_hybrid(p_end, target_rpy, theta3);

% Assemble joint waypoints
theta_waypoints = [theta1; theta2; theta3; theta4];  % 4x6 matrix

% Calculate total joint travel
travel_cost = 0;
for i = 1:3
    diff = abs(theta_waypoints(i+1,:) - theta_waypoints(i,:));
    travel_cost = travel_cost + sum(diff);
end
fprintf('Total Joint Travel: %.4f rad (%.2f deg)\n', travel_cost, rad2deg(travel_cost));
fprintf('Joint Angles at Waypoints (deg):\n');
disp(rad2deg(theta_waypoints));

%% STEP 3: PSO Optimization for Each Segment (3 Segments)
dt = 0.002;
sigma = 0.1;
H = 30;        % Swarm size
N_iter = 100;  % PSO iterations
lb = [0, 0];   % [a,b] lower bounds
ub = [1, 1];   % [a,b] upper bounds
% Joint limits
q_min = deg2rad([-165, -165, -165, -165, -165, -165]);
q_max = deg2rad([165, 165, 165, 165, 165, 165]);
% Velocity and acceleration limits (deg/s and deg/s²)
vel_limit = 160;
acc_limit = 300;
jerk_limit = 200;
% Storage for optimized parameters
segment_params = zeros(3, 2);  % [a, b] for each segment
segment_pairs = [1,2; 2,3; 3,4];
for seg = 1:3
    q0 = rad2deg(theta_waypoints(segment_pairs(seg,1), :));
    qf = rad2deg(theta_waypoints(segment_pairs(seg,2), :));
    
    fprintf('\nSegment %d: Waypoint %d → %d\n', seg, segment_pairs(seg,1), segment_pairs(seg,2));
    % Initialize PSO swarm
    B = rand(H, 2) .* (ub - lb) + lb;
    V = zeros(H, 2);
    P = B; Pcost = inf(H, 1);
    G = B(1,:); Gcost = inf;
    for it = 1:N_iter
        for i = 1:H
            a = B(i,1); b = B(i,2);
            [t, q, qd, qdd, qddd] = planner_erf(q0, qf, dt, a, b, sigma);
            % Hard constraints
            if any(abs(qd(:)) > vel_limit) || any(abs(qdd(:)) > acc_limit)
                cost = inf;
            elseif any(any(q < rad2deg(q_min)')) || any(any(q > rad2deg(q_max)'))
                cost = inf;
            else
                % Cost function: minimize time + penalties
                cost = t(end) ...
                     + sum(max(0, abs(qd) - vel_limit).^2, 'all') ...
                     + sum(max(0, abs(qdd) - acc_limit).^2, 'all') ...
                     + sum(max(0, abs(qddd) - jerk_limit).^2, 'all');
            end
            
            if cost < Pcost(i)
                P(i,:) = B(i,:); Pcost(i) = cost;
            end
            if cost < Gcost
                G = B(i,:); Gcost = cost;
            end
        end
        
        % Update velocities and positions
        w = 0.7; c1 = 1.5; c2 = 1.5;
        for i = 1:H
            r1 = rand; r2 = rand;
            V(i,:) = w*V(i,:) + c1*r1*(P(i,:) - B(i,:)) + c2*r2*(G - B(i,:));
            B(i,:) = min(max(B(i,:) + V(i,:), lb), ub);
        end
    end
    segment_params(seg, :) = G;
    fprintf('  Optimized: a=%.4f, b=%.4f, Duration=%.3fs\n', G(1), G(2), Gcost);
end

%% STEP 4: Generate Complete Trajectory by Stitching Segments
t_complete = []; q_complete = []; qd_complete = []; qdd_complete = []; qddd_complete = [];
time_offset = 0;  

for seg = 1:3
    q0 = rad2deg(theta_waypoints(segment_pairs(seg,1), :));
    qf = rad2deg(theta_waypoints(segment_pairs(seg,2), :));
    a_opt = segment_params(seg, 1);
    b_opt = segment_params(seg, 2);
    
    % Generating trajectory 
    [t_seg, q_seg, qd_seg, qdd_seg, qddd_seg] = planner_erf(q0, qf, dt, a_opt, b_opt, sigma);
    t_seg = t_seg + time_offset;
    
    if seg == 1
        t_complete = t_seg; q_complete = q_seg; qd_complete = qd_seg;
        qdd_complete = qdd_seg; qddd_complete = qddd_seg;
    else
        t_complete = [t_complete, t_seg(2:end)];
        q_complete = [q_complete, q_seg(:, 2:end)];
        qd_complete = [qd_complete, qd_seg(:, 2:end)];
        qdd_complete = [qdd_complete, qdd_seg(:, 2:end)];
        qddd_complete = [qddd_complete, qddd_seg(:, 2:end)];
    end
    time_offset = t_seg(end);
    fprintf('✓ (Duration: %.3fs)\n', t_seg(end) - t_seg(1));
end
fprintf('\nTotal trajectory duration: %.3f seconds\n', t_complete(end));

%% STEP 5: Plot Joint Profiles (Position, Velocity, Acceleration, Jerk)
figure('Name', 'Kinematic Profiles', 'Position', [100, 100, 1000, 800]);
titles = {'Position (deg)', 'Velocity (deg/s)', 'Acceleration (deg/s²)', 'Jerk (deg/s³)'};
data = {q_complete, qd_complete, qdd_complete, qddd_complete};
for i = 1:4
    subplot(4, 1, i); hold on;
    for j = 1:6
        plot(t_complete, data{i}(j,:), 'LineWidth', 1.5);
    end
    ylabel(titles{i}); grid on;
    if i == 1
        title('Gaussian S-Curve Multi-Waypoint Trajectory (4 Points)');
        seg1_end = segment_params(1,1)*4 + segment_params(1,2)*2 + 24*sigma;
        seg2_end = seg1_end + segment_params(2,1)*4 + segment_params(2,2)*2 + 24*sigma;
        xline(seg1_end, 'k--', 'LineWidth', 1.5);
        xline(seg2_end, 'k--', 'LineWidth', 1.5);
    end
    if i == 4
        xlabel('Time (s)');
        legend({'J1','J2','J3','J4','J5','J6'}, 'Location', 'eastoutside');
    end
end

%% STEP 6: Forward Kinematics - Compute End-Effector Path
n = length(t_complete);
xyz_path = zeros(n, 3);
rot_error = zeros(n, 1);
R_target = rpy2rotm(target_rpy(1), target_rpy(2), target_rpy(3));

for k = 1:n
    th = deg2rad(q_complete(:, k));
    [x, y, z, R] = flink6dofelephant(th(1), th(2), th(3), th(4), th(5), th(6));
    xyz_path(k,:) = [x, y, z];
    rot_error(k) = norm(R - R_target, 'fro');
end

%% STEP 7 & 8: Plot 3D Cartesian Path and Orientation Error
figure('Name', 'Spatial Analysis', 'Position', [150, 150, 900, 700]);
subplot(2, 1, 1);
plot3(xyz_path(:,1), xyz_path(:,2), xyz_path(:,3), 'b-', 'LineWidth', 2); hold on;
scatter3(xyz_path(1,1), xyz_path(1,2), xyz_path(1,3), 100, 'g', 'filled');
scatter3(xyz_path(end,1), xyz_path(end,2), xyz_path(end,3), 100, 'r', 'filled');
plot3(waypoints_cart(:,1), waypoints_cart(:,2), waypoints_cart(:,3), ...
      'ko--', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerFaceColor', 'k');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('End-Effector Trajectory - Gaussian S-Curve');
legend('Actual Path', 'Start', 'End', 'Desired Waypoints', 'Location', 'best');
grid on; axis equal; view(45, 30);

subplot(2, 1, 2);
plot(t_complete, rot_error, 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Error (Frobenius Norm)');
title('Orientation Deviation from Target (Facing Down)'); grid on;
xline(seg1_end, 'k--', 'LineWidth', 1.5, 'Alpha', 0.5);
xline(seg2_end, 'k--', 'LineWidth', 1.5, 'Alpha', 0.5);

%% STEP 9: Comparison Summary Statistics
fprintf('=== Trajectory Statistics ===\n');
fprintf('Total Duration: %.3f seconds\n', t_complete(end));
fprintf('Max Velocity:     %.2f deg/s\n', max(abs(qd_complete), [], 'all'));
fprintf('Max Acceleration: %.2f deg/s²\n', max(abs(qdd_complete), [], 'all'));
fprintf('Max Jerk:         %.2f deg/s³\n\n', max(abs(qddd_complete), [], 'all'));

%% STEP 10: 3D Visualization using Peter Corke's Robotics Toolbox
%% ========================================================================
% 
% % 1. Define the MyCobot 280 Pi Kinematic Chain
% L(1) = Link('revolute', 'offset', 0,       'd', 131.56,    'a', 0,         'alpha', pi/2);
% L(2) = Link('revolute', 'offset', -pi/2,   'd', 0,         'a', -110.4,    'alpha', 0);
% L(3) = Link('revolute', 'offset', 0,       'd', 0,         'a', -96,       'alpha', 0);
% L(4) = Link('revolute', 'offset', -pi/2,   'd', 66.39,     'a', 0,         'alpha', pi/2);
% L(5) = Link('revolute', 'offset', pi/2,    'd', 73.18,     'a', 0,         'alpha', -pi/2);
% L(6) = Link('revolute', 'offset', 0,       'd', 43.6,      'a', 0,         'alpha', 0);
% myCobot = SerialLink(L, 'name', 'myCobot 280 Pi');
% 
% % Set Joint Limits (converted to radians)
% myCobot.qlim = [-168 168; -135 135; -150 150; -145 145; -165 165; -180 180] * (pi/180);
% 
% % 2. Format Your Trajectory Data
% trajectory_degrees = q_complete; 
% trajectory_radians = deg2rad(trajectory_degrees);
% trajectory_matrix = trajectory_radians'; % Transpose from 6xN to Nx6
% 
% % 3. Animate the Robot
% figure('Name', 'MyCobot 3D Animation', 'Position', [200, 200, 800, 800]);
% view([45 30]); % Set an optimal isometric viewing angle
% 
% fprintf('Rendering animation...\n');
% 
% % Play the animation 
% myCobot.plot(trajectory_matrix, 'trail', 'r');
% 
% % myCobot.plot(trajectory_matrix, 'trail', 'r', 'movie', 'Final_Trajectory_Animation.gif');
%% ========================================================================

%% ========================================================================
%% CORE FUNCTIONS
%% ========================================================================

function [t, q, qd, qdd, qddd] = planner_erf(q0, qf, dt, a, b, sig)
    nJ = numel(q0);
    T = 24*sig + 4*a + 2*b;
    t = 0:dt:T;
    q = zeros(nJ, numel(t));
    qd = q; qdd = q; qddd = q;
    [p1, ~, ~, ~] = profile_erf(1, sig, a, b, t);
    for j = 1:nJ
        A = (qf(j) - q0(j)) / p1(end);
        [pos, vel, acc, jerk] = profile_erf(A, sig, a, b, t);
        q(j,:) = q0(j) + pos; qd(j,:) = vel; qdd(j,:) = acc; qddd(j,:) = jerk;
    end
end

function [pos, vel, acc, jerk] = profile_erf(A, sigma, a, b, t)
    c1 = 3*sigma; c2 = 6*sigma + 2*a + 3*sigma;
    c3 = 12*sigma + 2*a + 2*b + 3*sigma; c4 = 18*sigma + 4*a + 2*b + 3*sigma;
    t1 = 6*sigma; t2 = t1 + 2*a; t3 = t2 + 6*sigma; t4 = t3 + 2*b; t5 = t4 + 6*sigma; t6 = t5 + 2*a; t7 = t6 + 6*sigma;
    
    jerk = zeros(size(t));
    idx = (t >= 0) & (t <= t1); jerk(idx) = A * exp(-((t(idx) - c1).^2) / (2*sigma^2));
    idx = (t > t2) & (t <= t3); jerk(idx) = -A * exp(-((t(idx) - c2).^2) / (2*sigma^2));  
    idx = (t > t4) & (t <= t5); jerk(idx) = -A * exp(-((t(idx) - c3).^2) / (2*sigma^2));  
    idx = (t > t6) & (t <= t7); jerk(idx) = A * exp(-((t(idx) - c4).^2) / (2*sigma^2));
    
    acc = A * 0.5 * (erf((t - c1)/(sqrt(2)*sigma)) + erf((-c1)/(sqrt(2)*sigma)) ...
        - erf((t - c2)/(sqrt(2)*sigma)) - erf((t2 - c2)/(sqrt(2)*sigma)) ...
        - erf((t - c3)/(sqrt(2)*sigma)) - erf((t3 - c3)/(sqrt(2)*sigma)) ...
        + erf((t - c4)/(sqrt(2)*sigma)) + erf((t4 - c4)/(sqrt(2)*sigma)));
    
    vel = cumtrapz(t, acc); pos = cumtrapz(t, vel);
end

function [x, y, z, R] = flink6dofelephant(th1, th2, th3, th4, th5, th6)
    d1=131.56; d4=66.39; d5=73.18; d6=43.6; a2=110.4; a3=96;
    h = trans(th1, d1, 0, pi/2) * trans(th2-pi/2, 0, -a2, 0) * trans(th3, 0, -a3, 0) ...
      * trans(th4-pi/2, d4, 0, pi/2) * trans(th5+pi/2, d5, 0, -pi/2) * trans(th6, d6, 0, 0);
    x = h(1,4); y = h(2,4); z = h(3,4); R = h(1:3, 1:3);
    function T = trans(th, d, a, al)
        T = [cos(th) -sin(th)*cos(al) sin(th)*sin(al) a*cos(th);
             sin(th) cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
             0 sin(al) cos(al) d; 0 0 0 1];
    end
end

function R = rpy2rotm(roll, pitch, yaw)
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R = Rz * Ry * Rx;
end

function [G, E] = ilink6dofelephant_hybrid(pos_target, rpy_target, q_prev)
    N = 6; target_rot = rpy2rotm(rpy_target(1), rpy_target(2), rpy_target(3));
    max_attempts = 5; attempt = 0; success = false;
    best_G_global = zeros(1, 6); best_E_global = inf;
    pos_tol = 10.0; rot_tol = 0.2;
    
    while ~success && attempt < max_attempts
        attempt = attempt + 1;
        H = 150; N_iter = 300; w = 0.7; c1 = 1.4; c2 = 1.4;
        if isempty(q_prev) || attempt > 1
            B = (rand(H, N) - 0.5) * 2 * pi;
        else
            B = [ (rand(round(H*0.4), N) - 0.5) * 2 * pi; q_prev + (randn(round(H*0.6), N) * 0.5) ];
            B = B(1:H, :);
        end
        P = B; v = zeros(H, N); G = B(1,:); EPmin = inf; EB = inf(H, 1);
        
        for iter = 1:N_iter
            for i = 1:H
                for j = 1:5
                    if abs(B(i,j))*180/pi > 165, B(i,j) = sign(B(i,j)) * deg2rad(165); end
                end
                if abs(B(i,6))*180/pi > 175, B(i,6) = sign(B(i,6)) * deg2rad(175); end
                
                is_first_point = isempty(q_prev);
                q_safe = q_prev; if is_first_point, q_safe = zeros(1, 6); end
                score = objfxyz(B(i,:), pos_target, target_rot, q_safe, is_first_point);
                if score < EB(i), P(i,:) = B(i,:); EB(i) = score; end
                if score < EPmin, G = B(i,:); EPmin = score; end
            end
            for i = 1:H
                r = rand(1, 2);
                v(i,:) = w*v(i,:) + c1*r(1)*(P(i,:) - B(i,:)) + c2*r(2)*(G - B(i,:));
                B(i,:) = B(i,:) + v(i,:);
            end
        end
        [x, y, z, h] = flink6dofelephant(G(1), G(2), G(3), G(4), G(5), G(6));
        if sum(([x,y,z] - pos_target).^2) < pos_tol && norm(target_rot - h(1:3,1:3), 'fro') < rot_tol
            success = true; best_G_global = G; best_E_global = EPmin;
        else
            if EPmin < best_E_global, best_G_global = G; best_E_global = EPmin; end
        end
    end
    G = best_G_global; E = best_E_global;
end

function E = objfxyz(current_q, target_pos, target_R, prev_q, is_first_point)
    [x, y, z, h] = flink6dofelephant(current_q(1), current_q(2), current_q(3), current_q(4), current_q(5), current_q(6));
    err_pos = sum(([x,y,z] - target_pos).^2);
    err_rot = norm(target_R - h(1:3,1:3), 'fro');
    penalty_rot = 0; if err_rot > 0.3, penalty_rot = 1e5 * (err_rot - 0.3); end
    err_travel = 0; if ~is_first_point, err_travel = sum(abs(current_q - prev_q)); end
    E = (1.0 * err_pos) + (500.0 * err_rot) + (0.1 * err_travel) + penalty_rot;
end