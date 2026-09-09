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