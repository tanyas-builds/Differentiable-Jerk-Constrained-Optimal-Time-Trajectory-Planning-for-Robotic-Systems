% === Step 1: Define Path & Orientation ===
% Target Orientation: Facing Down [180, 0, 0]
target_rpy = [pi, 0, 0]; 

p_start   = [200, 100, 100];
p_lift    = [200, 100, 180];
p_descent = [150, 120, 140];
p_end     = [100, 150, 100];

% Define a "Home" guess for the first point (e.g., all zeros or a known safe pose)
q_seed = zeros(1, 6); 

fprintf('=== Solving Shortest Path Inverse Kinematics ===\n');

% === Step 2: Solve IK with "Warm Start" Chaining ===

% Point 1: Use Home Seed
disp('Solving Point 1 (Start)...');
[theta1, ~] = ilink6dofelephant(p_start, target_rpy, q_seed);

% Point 2: Use theta1 as Seed (Shortest path from P1)
disp('Solving Point 2 (Lift)...');
[theta2, ~] = ilink6dofelephant(p_lift, target_rpy, theta1);

% Point 3: Use theta2 as Seed
disp('Solving Point 3 (Descent)...');
[theta3, ~] = ilink6dofelephant(p_descent, target_rpy, theta2);

% Point 4: Use theta3 as Seed
disp('Solving Point 4 (End)...');
[theta4, ~] = ilink6dofelephant(p_end, target_rpy, theta3);

% === Step 3: Verify Continuity ===
thetaPoints = [theta1; theta2; theta3; theta4];

% Calculate "Travel Cost" (Sum of joint movements between points)
travel_cost = 0;
for i = 1:3
    diff = abs(thetaPoints(i+1,:) - thetaPoints(i,:));
    travel_cost = travel_cost + sum(diff);
end
fprintf('Total Joint Travel (Rad): %.4f\n', travel_cost);
disp('Joint Angles (Deg):');
disp(rad2deg(thetaPoints));

% === Step 4: Generate 6-7-6 Trajectory (Standard) ===
t1 = 5; t2 = 5; t3 = 5;
Traj = cell(6,1);
for j = 1:6
    Coefs = construct_676_full_numerical(thetaPoints(1,j), thetaPoints(2,j), ...
                                         thetaPoints(3,j), thetaPoints(4,j), ...
                                         t1, t2, t3);
    Traj{j} = Coefs;
end

% === Evaluate all joint trajectories over time ===
nSamples = 100;
t_seg1 = linspace(0, t1, nSamples);
t_seg2 = linspace(0, t2, nSamples);
t_seg3 = linspace(0, t3, nSamples);

t_full = [t_seg1, t_seg2 + t1, t_seg3 + t1 + t2];
nTotal = length(t_full);

q_mat = zeros(6, nTotal);
v_mat = zeros(6, nTotal);
a_mat = zeros(6, nTotal);
j_mat = zeros(6, nTotal);
s_mat = zeros(6, nTotal);  % Snap (fourth derivative)

for j = 1:6
    q1 = polyval(Traj{j}.a, t_seg1);
    q2 = polyval(Traj{j}.b, t_seg2);
    q3 = polyval(Traj{j}.c, t_seg3);
    q_mat(j,:) = [q1, q2, q3];

    v1 = polyval(polyder(Traj{j}.a), t_seg1);
    v2 = polyval(polyder(Traj{j}.b), t_seg2);
    v3 = polyval(polyder(Traj{j}.c), t_seg3);
    v_mat(j,:) = [v1, v2, v3];

    a1 = polyval(polyder(polyder(Traj{j}.a)), t_seg1);
    a2 = polyval(polyder(polyder(Traj{j}.b)), t_seg2);
    a3 = polyval(polyder(polyder(Traj{j}.c)), t_seg3);
    a_mat(j,:) = [a1, a2, a3];
    
    j1 = polyval(polyder(polyder(polyder(Traj{j}.a))), t_seg1);
    j2 = polyval(polyder(polyder(polyder(Traj{j}.b))), t_seg2);
    j3 = polyval(polyder(polyder(polyder(Traj{j}.c))), t_seg3);
    j_mat(j,:) = [j1, j2, j3];

    s1 = polyval(polyder(polyder(polyder(polyder(Traj{j}.a)))), t_seg1);
    s2 = polyval(polyder(polyder(polyder(polyder(Traj{j}.b)))), t_seg2);
    s3 = polyval(polyder(polyder(polyder(polyder(Traj{j}.c)))), t_seg3);
    s_mat(j,:) = [s1, s2, s3];
end

% === Plot Joint Profiles ===
figure;
subplot(5,1,1); hold on;
for j = 1:6
    plot(t_full, rad2deg(q_mat(j,:)), 'LineWidth', 2);
end
ylabel('Position (deg)');
title('Joint Trajectories');
legend('q₁','q₂','q₃','q₄','q₅','q₆');
grid on;

subplot(5,1,2); hold on;
for j = 1:6
    plot(t_full, rad2deg(v_mat(j,:)), 'LineWidth', 2);
end
ylabel('Velocity (deg/s)');
grid on;

subplot(5,1,3); hold on;
for j = 1:6
    plot(t_full, rad2deg(a_mat(j,:)), 'LineWidth', 2);
end
ylabel('Acceleration (deg/s²)');
grid on;

subplot(5,1,4); hold on;
for j = 1:6
    plot(t_full, rad2deg(j_mat(j,:)), 'LineWidth', 2);
end
ylabel('Jerk (deg/s³)');
grid on;

subplot(5,1,5); hold on;
for j = 1:6
    plot(t_full, rad2deg(s_mat(j,:)), 'LineWidth', 2);
end
ylabel('Snap (deg/s⁴)');
xlabel('Time (s)');
grid on;

% === Forward Kinematic Path Tracking + Desired Path Overlay ===
xyz_path = zeros(nTotal, 3);
rot_error_norm = zeros(nTotal, 1);

% Convert target RPY to Matrix for comparison
R_target_mat = rpy2rotm(target_rpy(1), target_rpy(2), target_rpy(3));

for k = 1:nTotal
    q_now = q_mat(:,k);
    [x, y, z, h] = flink6dofelephant(q_now(1), q_now(2), q_now(3), ...
                                     q_now(4), q_now(5), q_now(6));
    xyz_path(k,:) = [x, y, z];
    
    % Calculate Orientation Error along the path
    R_current = h(1:3, 1:3);
    rot_error_norm(k) = norm(R_target_mat - R_current, 'fro');
end

% Desired waypoints used for IK (already known)
desired_points = [p_start; p_lift; p_descent; p_end];

% === Plot FK Trajectory with Desired Points Overlay ===
figure;
plot3(xyz_path(:,1), xyz_path(:,2), xyz_path(:,3), 'b-', 'LineWidth', 2); hold on;
scatter3(xyz_path(1,1), xyz_path(1,2), xyz_path(1,3), 100, 'g', 'filled');
scatter3(xyz_path(end,1), xyz_path(end,2), xyz_path(end,3), 100, 'r', 'filled');
plot3(desired_points(:,1), desired_points(:,2), desired_points(:,3), ...
      'ko--', 'LineWidth', 2, 'MarkerFaceColor', 'k');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('End-Effector Trajectory with Desired Waypoints');
legend('FK Trajectory', 'Start', 'End', 'Desired Waypoints');
grid on; axis equal;

% === NEW FIGURE: Orientation Error Plot ===

figure;
plot(t_full, rot_error_norm, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Orientation Error (Frobenius Norm)');
title('Deviation from Desired Orientation (Facing Down)');
grid on;

v_matdeg = rad2deg(v_mat);
q_matdeg = rad2deg(q_mat);

% === Function for 6-7-6 Trajectory ===
function Coefs = construct_676_full_numerical(theta1, theta2, theta3, theta4, t1, t2, t3)
% Solves the full 6-7-6 trajectory as a single 22x22 linear system
% Inputs: joint positions θ1 to θ4, and segment durations t1, t2, t3
% Output: Coefs struct with .a (6th-degree), .b (7th-degree), .c (6th-degree)
% Unknowns: [a0 a1 a2 a3 a4 a5 a6 b0 b1 b2 b3 b4 b5 b6 b7 c0 c1 c2 c3 c4 c5 c6]
T = zeros(22, 22);
Y = zeros(22, 1);
% Segment 1 constraints
T(1,1) = 1; Y(1) = theta1;        % s1(0) = a0 = θ1
T(2,2) = 1; Y(2) = 0;             % s1'(0) = a1 = 0
T(3,3) = 2; Y(3) = 0;             % s1''(0) = 2*a2 = 0
T(4,4) = 6; Y(4) = 0;             % s1'''(0) = 6*a3 = 0
T(5,5) = 24; Y(5) = 0;            % s1^(4)(0) = 24*a4 = 0
T(6,1:7) = [1, t1, t1^2, t1^3, t1^4, t1^5, t1^6]; Y(6) = theta2;  % s1(t1) = θ2
% Segment 2 constraints
T(7,8) = 1; Y(7) = theta2;        % s2(0) = b0 = θ2
T(8,8:15) = [1, t2, t2^2, t2^3, t2^4, t2^5, t2^6, t2^7]; Y(8) = theta3;  % s2(t2) = θ3
% Segment 3 constraints
T(9,16) = 1; Y(9) = theta3;       % s3(0) = c0 = θ3
T(10,16:22) = [1, t3, t3^2, t3^3, t3^4, t3^5, t3^6]; Y(10) = theta4;  % s3(t3) = θ4
% Continuity at t1 (s1(t1) = s2(0) already satisfied by rows 6 and 7)
T(11,2:7) = [1, 2*t1, 3*t1^2, 4*t1^3, 5*t1^4, 6*t1^5]; T(11,9) = -1;  % s1'(t1) = s2'(0)
T(12,3:7) = [2, 6*t1, 12*t1^2, 20*t1^3, 30*t1^4]; T(12,10) = -2;     % s1''(t1) = s2''(0)
T(13,4:7) = [6, 24*t1, 60*t1^2, 120*t1^3]; T(13,11) = -6;            % s1'''(t1) = s2'''(0)
T(14,5:7) = [24, 120*t1, 360*t1^2]; T(14,12) = -24;                  % s1^(4)(t1) = s2^(4)(0)
% Continuity at t1 + t2
T(15,9:15) = [1, 2*t2, 3*t2^2, 4*t2^3, 5*t2^4, 6*t2^5, 7*t2^6]; T(15,17) = -1;  % s2'(t2) = s3'(0)
T(16,10:15) = [2, 6*t2, 12*t2^2, 20*t2^3, 30*t2^4, 42*t2^5]; T(16,18) = -2;   % s2''(t2) = s3''(0)
T(17,11:15) = [6, 24*t2, 60*t2^2, 120*t2^3, 210*t2^4]; T(17,19) = -6;         % s2'''(t2) = s3'''(0)
T(18,12:15) = [24, 120*t2, 360*t2^2, 840*t2^3]; T(18,20) = -24;               % s2^(4)(t2) = s3^(4)(0)
% End conditions for s3(t3)
T(19,17:22) = [1, 2*t3, 3*t3^2, 4*t3^3, 5*t3^4, 6*t3^5]; Y(19) = 0;  % s3'(t3) = 0
T(20,18:22) = [2, 6*t3, 12*t3^2, 20*t3^3, 30*t3^4]; Y(20) = 0;       % s3''(t3) = 0
T(21,19:22) = [6, 24*t3, 60*t3^2, 120*t3^3]; Y(21) = 0;              % s3'''(t3) = 0
T(22,20:22) = [24, 120*t3, 360*t3^2]; Y(22) = 0;                     % s3^(4)(t3) = 0
% Solve the system
X = T \ Y;
% Extract coefficients (MATLAB polyval expects high-degree first)
Coefs.a = X(7:-1:1)';    % [a6 a5 a4 a3 a2 a1 a0]
Coefs.b = X(15:-1:8)';   % [b7 b6 b5 b4 b3 b2 b1 b0]
Coefs.c = X(22:-1:16)';  % [c6 c5 c4 c3 c2 c1 c0]
end

% === Kinematics Functions ===
function [x,y,z,h]=flink6dofelephant(theta1,theta2,theta3, theta4, theta5, theta6)
    d1=131.56; d4=66.39; d5=73.18; d6=43.6;
    a2=110.4; a3=96;
    h=trans(theta1,d1,0,pi/2)*trans(theta2-pi/2,0,-a2,0)*trans(theta3,0,-a3,0)...
        *trans(theta4-pi/2,d4,0,pi/2)*trans(theta5+pi/2,d5,0,-pi/2)*trans(theta6,d6,0,0);
    x=h(1,4); y=h(2,4); z=h(3,4);
end

function T=trans(theta,d,a,alpha)
    T=[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
       sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
       0 sin(alpha) cos(alpha) d;
       0 0 0 1];
end

function R = rpy2rotm(roll, pitch, yaw)
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R = Rz * Ry * Rx;
end

function [G, E] = ilink6dofelephant(pos_target, rpy_target, q_prev)
    % Robust IK Solver with Retry Logic and Soft Constraints
    
    N = 6;
    target_rot = rpy2rotm(rpy_target(1), rpy_target(2), rpy_target(3));
    
    % --- RETRY PARAMETERS ---
    max_attempts = 5;      % Try up to 5 times if it fails
    attempt = 0;
    success = false;
    best_G_global = zeros(1,6);
    best_E_global = inf;
    
    % Convergence Thresholds (What counts as "Good Enough"?)
    pos_tol = 10.0; % 10mm tolerance (generous for "success" check)
    rot_tol = 0.2;  % Orientation tolerance
    
    while ~success && attempt < max_attempts
        attempt = attempt + 1;
        
        % --- PSO Parameters ---
        H = 150;           
        N_iter = 300;     
        w = 0.7; c1 = 1.4; c2 = 1.4; 
        
        % --- Initialization (Mixed Strategy) ---
        if isempty(q_prev) || attempt > 1
            % On first run OR RETRY: Use mostly random to break stuck state
            B = (rand(H, N) - 0.5) * 2 * pi;
        else
            % Standard: Seed around previous solution (Warm Start)
            B_rand  = (rand(round(H*0.4), N) - 0.5) * 2 * pi;
            B_local = q_prev + (randn(round(H*0.6), N) * 0.5); 
            B = [B_rand; B_local];
            B = B(1:H, :); 
        end
        
        P = B; 
        v = zeros(H, N);
        G = B(1,:); 
        EPmin = inf;
        EB = inf(H,1);
        
        % --- PSO Main Loop ---
        for iter = 1:N_iter
            for i = 1:H
                % Soft Joint Limits
                for j=1:5
                   if abs(B(i,j))*180/pi > 165, B(i,j) = sign(B(i,j))*deg2rad(165); end
                end
                if abs(B(i,6))*180/pi > 175, B(i,6) = sign(B(i,6))*deg2rad(175); end
                
                % Calculate Cost
                is_first_point = isempty(q_prev);
                q_prev_safe = q_prev;
                if is_first_point, q_prev_safe = zeros(1,6); end
                
                score = objfxyz(B(i,:), pos_target, target_rot, q_prev_safe, is_first_point);
                
                if score < EB(i)
                    P(i,:) = B(i,:);
                    EB(i) = score;
                end
                if score < EPmin
                    G = B(i,:);
                    EPmin = score;
                end
            end
            
            % Update Velocities
            for i=1:H
                 r = rand(1,2);
                 v(i,:) = w*v(i,:) + c1*r(1)*(P(i,:)-B(i,:)) + c2*r(2)*(G-B(i,:));
                 B(i,:) = B(i,:) + v(i,:);
            end
        end
        
        % --- Success Check ---
        % Calculate final real errors to see if we passed
        [x, y, z, h] = flink6dofelephant(G(1),G(2),G(3),G(4),G(5),G(6));
        final_pos_err = sum(([x,y,z] - pos_target).^2);
        final_rot_err = norm(target_rot - h(1:3,1:3), 'fro');
        
        % If valid, we stop retrying
        if final_pos_err < pos_tol && final_rot_err < rot_tol
            success = true;
            best_G_global = G;
            best_E_global = EPmin;
             % fprintf('  -> Success on attempt %d\n', attempt);
        else
            % Keep the best result found so far, just in case we never succeed
            if EPmin < best_E_global
                best_G_global = G;
                best_E_global = EPmin;
            end
             % fprintf('  -> Attempt %d failed (PosErr: %.2f). Retrying...\n', attempt, final_pos_err);
        end
    end
    
    G = best_G_global;
    E = best_E_global;
end

function E = objfxyz(current_q, target_pos, target_R, prev_q, is_first_point)
    % 1. Forward Kinematics
    [x, y, z, h] = flink6dofelephant(current_q(1), current_q(2), current_q(3), ...
                                     current_q(4), current_q(5), current_q(6));
    
    % 2. Errors
    err_pos = sum(([x,y,z] - target_pos).^2);       % Squared Distance
    err_rot = norm(target_R - h(1:3,1:3), 'fro');   % Orientation Error
    
    % --- CRITICAL FIX: SOFT CONSTRAINT ---
    % Instead of returning 'inf', we add a massive penalty gradient.
    % This guides the swarm BACK to the valid zone instead of blinding it.
    penalty_rot = 0;
    if err_rot > 0.3
        % Linear penalty ensures gradient exists (100,000 * amount of violation)
        penalty_rot = 1e5 * (err_rot - 0.3);
    end
    
    % 3. Travel Cost
    err_travel = 0;
    if ~is_first_point
        err_travel = sum(abs(current_q - prev_q));
    end
    
    % --- WEIGHTS ---
    w_pos = 1.0;
    w_rot = 500.0;    
    w_travel = 0.1;
    
    % Total Cost includes the Soft Penalty
    E = (w_pos * err_pos) + (w_rot * err_rot) + (w_travel * err_travel) + penalty_rot;
end