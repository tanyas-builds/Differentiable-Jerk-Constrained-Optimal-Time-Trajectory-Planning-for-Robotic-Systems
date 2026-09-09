
% === Step 1: Define fixed Cartesian path points ===
% https://www.elephantrobotics.com/en/mycobot-280-m5-new-specificatons-en/
p_start   = [200, 100, 100];  
p_lift    = [200, 100, 180];  
p_descent = [150, 120, 140];  
p_end     = [100, 150, 100];  

% Display the generated path
disp('Generated Cartesian Path Points:');
disp('Start:   '); disp(p_start);
disp('Lift:    '); disp(p_lift);
disp('Descent:'); disp(p_descent);
disp('End:     '); disp(p_end);

% === Step 2: Solve IK using your PSO-based function ===
[theta1, ~] = ilink6dofelephant(p_start(1),   p_start(2),   p_start(3));
[theta2, ~] = ilink6dofelephant(p_lift(1),    p_lift(2),    p_lift(3));
[theta3, ~] = ilink6dofelephant(p_descent(1), p_descent(2), p_descent(3));
[theta4, ~] = ilink6dofelephant(p_end(1),     p_end(2),     p_end(3));

% === Step 3: Assemble for 6-7-6 trajectory ===
thetaPoints = [theta1; theta2; theta3; theta4];  % size 4x6

% Optional: Display joint angles
disp('Joint angles at via points (deg):');
disp(rad2deg(thetaPoints));

% === Step 4: Solve 6-7-6 trajectory for each joint ===
t1 = 3; t2 = 3; t3 = 3;
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
for k = 1:nTotal
    q_now = q_mat(:,k);
    [x, y, z, ~] = flink6dofelephant(q_now(1), q_now(2), q_now(3), ...
                                     q_now(4), q_now(5), q_now(6));
    xyz_path(k,:) = [x, y, z];
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

function [G E]=ilink6dofelephant(xd,yd,zd)
% ILINK6DOFELEPHANT Solves Inverse Kinematics for MyCobot 280 using PSO.
% Inputs: xd, yd, zd (Target Position in mm)
% Outputs: G (Best Joint Angles), E_history (Convergence Plot Data)

N=6;                       % DOF
pxyz=[xd yd zd];           % Target Position   
ww=0.8;                    % Inertia weight
c1=1.2;                    % Cognitive Weight
c2=1.2;                    % Social Weight

r=rand(1,2);               % 
H=100;                     % Number of Particles
N_iter=1000; 
vmin=-1; 
vmax=1;

% Randomize particles
B=2*(rand(H,N)-0.5); 
P=2*(rand(H,N)-0.5); 

v=P; 
G=2*(rand(1,N)-0.5);    % 

iter=0; 
EB=zeros(1,H);           % Global Best Position
EPmin=inf;               % Global Best Cost

for i=1:H, EBmin(i)=inf; end

while iter < N_iter
    iter=iter+1;
    % ---- Evaluate Swarm ----
    for i=1:H
        % Check Joint Limits for Joint 1 to 5
        for j=1:5
            if abs(B(i,j))*180/pi>165, B(i,j)=B(i,j)/2; end
        end
        %  Check Joint limits for Joint 6
        if abs(B(i,6))*180/pi>175, B(i,6)=B(i,6)/2; end
        EB(i)=objfxyz(B(i,:),pxyz);
        if EB(i) < EBmin(i)
            P(i,:)=B(i,:); EBmin(i)=EB(i);
        end
    end
    
    for i=1:H
        for j=1:5
            if abs(P(i,j))*180/pi>165, P(i,j)=P(i,j)/2; end
        end
        if abs(P(i,6))*180/pi>175, P(i,6)=P(i,6)/2; end
        EP=objfxyz(P(i,:),pxyz);
        if EP<EPmin, G=P(i,:); EPmin=EP; end
    end
    for i=1:H
        for j=1:N
            r=rand(1,2);
            v(i,j)=ww*v(i,j)+c1*r(1)*(P(i,j)-B(i,j))+c2*r(2)*(G(j)-B(i,j));
            v(i,j)=max(min(v(i,j),vmax),vmin);
        end
        B(i,:)=B(i,:)+v(i,:);
    end
    E(iter)=EPmin;
end
end

function E=objfxyz(BB,HDj)
theta1=BB(1); theta2=BB(2); theta3=BB(3);
theta4=BB(4); theta5=BB(5); theta6=BB(6);
[x y z h]=flink6dofelephant(theta1,theta2,theta3,theta4,theta5,theta6);
hh=[x y z];
E=sum((abs(hh-HDj)).^2)+0.2*sum(abs([theta1 theta2 theta3 theta4 theta5 theta6]));
end