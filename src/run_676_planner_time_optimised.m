%% W-MOPSO Optimized 6-7-6 Polynomial Trajectory - Multi-Path Tester
%% Evaluates Paths 1 through 5 sequentially with Time Optimization

clear; clc; close all;
fprintf('=== W-MOPSO 6-7-6 Multi-Waypoint Trajectory Tester ===\n\n');

% Orientation: Facing Down 
target_rpy = [pi, 0, 0]; 

% Define Paths 1 to 5 
ConfigPaths = {
    %Path1 
    [200, 100, 100; 200, 100, 180; 150, 120, 140; 100, 150, 100],
    % Path2
    [150, -100, 120; 150, -100, 200; 200, -50, 160; 220, 0, 120],
    % Path3:
    [100, -200, 150; 100, 200, 250; 50, 180, 200; 0, 150, 150],
    % Path4
    [50, 150, 80; 100, 100, 220; 180, 0, 180; 220, -100, 80],
    % Path5
    [180, 0, 50; 180, 0, 250; 180, 120, 200; 180, 120, 150]
};

% Kinematic Limits (Velocity, Acceleration and Jerk)
v_max = 160; 
a_max = 300; 
j_max = 200; 


%% MAIN LOOP: Iterate through Paths 1 to 5
for config = 1:length(ConfigPaths)
    current_path = ConfigPaths{config};
    p_start   = current_path(1, :);
    p_lift    = current_path(2, :);
    p_descent = current_path(3, :);
    p_end     = current_path(4, :);
    
    % Solve IK 
    q_seed = zeros(1, 6); 
    [theta1, ~] = ilink6dofelephant_hybrid(p_start, target_rpy, q_seed);
    [theta2, ~] = ilink6dofelephant_hybrid(p_lift, target_rpy, theta1);
    [theta3, ~] = ilink6dofelephant_hybrid(p_descent, target_rpy, theta2);
    [theta4, ~] = ilink6dofelephant_hybrid(p_end, target_rpy, theta3);
    
    thetaPoints = [theta1; theta2; theta3; theta4];
    
    travel_cost = 0;
    for i = 1:3
        travel_cost = travel_cost + sum(abs(thetaPoints(i+1,:) - thetaPoints(i,:)));
    end
    fprintf('Total Joint Travel: %.2f deg\n', rad2deg(travel_cost));

    % W-MOPSO Time Optimization for t1, t2, t3 
    H_time = 30; N_iter_time = 50;
    w_time = 0.7; c1_time = 1.5; c2_time = 1.5;
    t_min = 0.5; t_max = 5.0;

    B_t = rand(H_time, 3) * (t_max - t_min) + t_min;
    V_t = zeros(H_time, 3); P_t = B_t; Pcost = inf(H_time, 1);
    G_t = B_t(1,:); Gcost = inf;

    for iter = 1:N_iter_time
        for i = 1:H_time
            t_cand = B_t(i,:);
            penalty = 0;
            try
                for j = 1:6
                    Coefs = construct_676_full_numerical(thetaPoints(1,j), thetaPoints(2,j), thetaPoints(3,j), thetaPoints(4,j), t_cand(1), t_cand(2), t_cand(3));
                    
                    ts1 = linspace(0, t_cand(1), 15); ts2 = linspace(0, t_cand(2), 15); ts3 = linspace(0, t_cand(3), 15);
                    v1 = polyval(polyder(Coefs.a), ts1); v2 = polyval(polyder(Coefs.b), ts2); v3 = polyval(polyder(Coefs.c), ts3);
                    a1 = polyval(polyder(polyder(Coefs.a)), ts1); a2 = polyval(polyder(polyder(Coefs.b)), ts2); a3 = polyval(polyder(polyder(Coefs.c)), ts3);
                    j1 = polyval(polyder(polyder(polyder(Coefs.a))), ts1); j2 = polyval(polyder(polyder(polyder(Coefs.b))), ts2); j3 = polyval(polyder(polyder(polyder(Coefs.c))), ts3);
                    
                    max_v = max(abs(rad2deg([v1, v2, v3])));
                    max_a = max(abs(rad2deg([a1, a2, a3])));
                    max_j = max(abs(rad2deg([j1, j2, j3])));
                    
                    if max_v > v_max, penalty = penalty + 1000 * (max_v - v_max)^2; end
                    if max_a > a_max, penalty = penalty + 1000 * (max_a - a_max)^2; end
                    if max_j > j_max, penalty = penalty + 1000 * (max_j - j_max)^2; end
                end
                cost = sum(t_cand) + penalty;
            catch
                cost = inf;
            end
            
            if cost < Pcost(i), P_t(i,:) = t_cand; Pcost(i) = cost; end
            if cost < Gcost, G_t = t_cand; Gcost = cost; end
        end
        
        for i = 1:H_time
            r = rand(1,2);
            V_t(i,:) = w_time*V_t(i,:) + c1_time*r(1)*(P_t(i,:) - B_t(i,:)) + c2_time*r(2)*(G_t - B_t(i,:));
            B_t(i,:) = max(min(B_t(i,:) + V_t(i,:), t_max), t_min);
        end
    end

    t1 = G_t(1); t2 = G_t(2); t3 = G_t(3);
    fprintf('Optimized Times: t1=%.2fs, t2=%.2fs, t3=%.2fs (Total: %.2fs)\n', t1, t2, t3, t1+t2+t3);

    % Generate Final Profiles 
    Traj = cell(6,1);
    for j = 1:6
        Traj{j} = construct_676_full_numerical(thetaPoints(1,j), thetaPoints(2,j), thetaPoints(3,j), thetaPoints(4,j), t1, t2, t3);
    end

    nSamples = 100;
    t_seg1 = linspace(0, t1, nSamples);
    t_seg2 = linspace(0, t2, nSamples);
    t_seg3 = linspace(0, t3, nSamples);
    t_full = [t_seg1, t_seg2 + t1, t_seg3 + t1 + t2];
    nTotal = length(t_full);

    q_mat = zeros(6, nTotal); v_mat = zeros(6, nTotal);
    a_mat = zeros(6, nTotal); j_mat = zeros(6, nTotal);
    s_mat = zeros(6, nTotal);
    
    for j = 1:6
        q_mat(j,:) = [polyval(Traj{j}.a, t_seg1), polyval(Traj{j}.b, t_seg2), polyval(Traj{j}.c, t_seg3)];
        v_mat(j,:) = [polyval(polyder(Traj{j}.a), t_seg1), polyval(polyder(Traj{j}.b), t_seg2), polyval(polyder(Traj{j}.c), t_seg3)];
        a_mat(j,:) = [polyval(polyder(polyder(Traj{j}.a)), t_seg1), polyval(polyder(polyder(Traj{j}.b)), t_seg2), polyval(polyder(polyder(Traj{j}.c)), t_seg3)];
        j_mat(j,:) = [polyval(polyder(polyder(polyder(Traj{j}.a))), t_seg1), polyval(polyder(polyder(polyder(Traj{j}.b))), t_seg2), polyval(polyder(polyder(polyder(Traj{j}.c))), t_seg3)];
        s_mat(j,:) = [polyval(polyder(polyder(polyder(polyder(Traj{j}.a)))), t_seg1), polyval(polyder(polyder(polyder(polyder(Traj{j}.b)))), t_seg2), polyval(polyder(polyder(polyder(polyder(Traj{j}.c)))), t_seg3)];
    end

    % Plot Kinematics 
    figure('Name', sprintf('Path %d: Kinematics', config), 'Position', [50, 100, 800, 900]);
    titles = {'Position (deg)', 'Velocity (deg/s)', 'Acceleration (deg/s²)', 'Jerk (deg/s³)', 'Snap (deg/s⁴)'};
    data = {q_mat, v_mat, a_mat, j_mat, s_mat};
    for i = 1:5
        subplot(5, 1, i); hold on;
        for j = 1:6, plot(t_full, rad2deg(data{i}(j,:)), 'LineWidth', 1.5); end
        ylabel(titles{i}); grid on;
        if i == 1
            title(sprintf('Path %d: W-MOPSO 6-7-6 Trajectory (Total Time: %.2fs)', config, t1+t2+t3));
        elseif i == 5
            xlabel('Time (s)'); legend({'J1','J2','J3','J4','J5','J6'}, 'Location', 'eastoutside');
        end
    end

    % Forward Kinematics & 3D Plot 
    xyz_path = zeros(nTotal, 3);
    for k = 1:nTotal
        q_now = q_mat(:,k);
        [x, y, z, ~] = flink6dofelephant(q_now(1), q_now(2), q_now(3), q_now(4), q_now(5), q_now(6));
        xyz_path(k,:) = [x, y, z];
    end

    figure('Name', sprintf('Path %d: Spatial Tracking', config), 'Position', [900, 300, 700, 600]);
    plot3(xyz_path(:,1), xyz_path(:,2), xyz_path(:,3), 'b-', 'LineWidth', 2); hold on;
    scatter3(xyz_path(1,1), xyz_path(1,2), xyz_path(1,3), 100, 'g', 'filled');
    scatter3(xyz_path(end,1), xyz_path(end,2), xyz_path(end,3), 100, 'r', 'filled');
    plot3(current_path(:,1), current_path(:,2), current_path(:,3), 'ko--', 'LineWidth', 2, 'MarkerFaceColor', 'k');
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title(sprintf('Path %d: End-Effector Trajectory', config));
    legend('FK Trajectory', 'Start', 'End', 'Desired Waypoints', 'Location', 'best');
    grid on; axis equal; view(45, 30);
    
    % Pause
    if config < length(ConfigPaths)
        fprintf('\n---> Press ENTER to process the next path...\n');
        pause;
    end
end
fprintf('\nAll configurations processed successfully.\n');




