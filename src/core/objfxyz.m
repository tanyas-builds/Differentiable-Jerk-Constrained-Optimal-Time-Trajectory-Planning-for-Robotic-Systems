function E = objfxyz(current_q, target_pos, target_R, prev_q, is_first_point)
    [x, y, z, h] = flink6dofelephant(current_q(1), current_q(2), current_q(3), current_q(4), current_q(5), current_q(6));
    err_pos = sum(([x,y,z] - target_pos).^2);
    err_rot = norm(target_R - h(1:3,1:3), 'fro');
    penalty_rot = 0; if err_rot > 0.3, penalty_rot = 1e5 * (err_rot - 0.3); end
    err_travel = 0; if ~is_first_point, err_travel = sum(abs(current_q - prev_q)); end
    E = (1.0 * err_pos) + (500.0 * err_rot) + (0.1 * err_travel) + penalty_rot;
end