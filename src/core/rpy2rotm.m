% This function converts three Euler angles (Roll Rx, Pitch Ry and Yaw Rz) 
% into a single combined 3 x 3 Rotation Matrix R 
function R = rpy2rotm(roll, pitch, yaw)
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R = Rz * Ry * Rx;
end