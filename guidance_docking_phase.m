function [r_des, v_des] = guidance_docking_phase(r, currentPhase, waypoints, phase)
% Piecewise guidance for initial approach / transposition / final approach

switch currentPhase
    case 1
        % Initial R-bar approach toward x = -100 m
        r_des = waypoints.initialApproach;
        v_des = [0.3; 0; 0];

    case 2
        % Transposition maneuver toward [20; 0; 20]
        r_des = waypoints.transposition;
        dir = r_des - r;
        if norm(dir) > 1e-8
            v_des = 0.15 * dir / norm(dir);
        else
            v_des = [0;0;0];
        end

    case 3
        % Final approach: 0.3 m/s outside 10 m, 0.03 m/s inside 10 m
        r_des = waypoints.finalApproach;
        if norm(r) > phase.finalSlowdownRange
            speed = 0.3;
        else
            speed = 0.03;
        end
        dir = r_des - r;
        if norm(dir) > 1e-8
            v_des = speed * dir / norm(dir);
        else
            v_des = [0;0;0];
        end

    otherwise
        r_des = [-1;0;0];
        v_des = [0;0;0];
end
end
