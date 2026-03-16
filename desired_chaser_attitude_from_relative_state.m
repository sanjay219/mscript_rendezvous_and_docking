function q_cmd = desired_chaser_attitude_from_relative_state(r_rel)
%DESIRED_CHASER_ATTITUDE_FROM_RELATIVE_STATE
% Returns a desired scalar-first quaternion [q0 q1 q2 q3]' for the chaser.
%
% Goal:
%   Make the chaser body +Z axis point toward the target, which is
%   consistent with your conical sensor mounting:
%
%       conicalSensor(chaser,'MountingLocation',[0;0;0.5])
%
% Since r_rel is the chaser relative position with respect to target
% expressed in Hill frame, the direction from chaser to target is:
%
%   d = -r_rel
%
% Frame convention used to build desired body axes:
%   body +Z  -> points toward target
%   body +X  -> chosen using Hill +X as a reference, unless nearly parallel
%   body +Y  -> completes right-handed triad
%
% Output:
%   q_cmd = scalar-first quaternion [q0 q1 q2 q3]'

    r_rel = r_rel(:);

    % Direction from chaser to target
    d = -r_rel;

    if norm(d) < 1e-8 || any(~isfinite(d))
        q_cmd = [1;0;0;0];
        return;
    end

    z_b_in_hill = d / norm(d);

    % Use Hill +X as preferred reference for body +X
    x_ref = [1;0;0];

    % If nearly parallel, switch reference
    if abs(dot(z_b_in_hill, x_ref)) > 0.95
        x_ref = [0;1;0];
    end

    % Orthogonalize x_ref against z_b
    x_b_in_hill = x_ref - dot(x_ref, z_b_in_hill)*z_b_in_hill;

    if norm(x_b_in_hill) < 1e-8 || any(~isfinite(x_b_in_hill))
        x_b_in_hill = [1;0;0];
    else
        x_b_in_hill = x_b_in_hill / norm(x_b_in_hill);
    end

    % Complete right-handed frame
    y_b_in_hill = cross(z_b_in_hill, x_b_in_hill);
    if norm(y_b_in_hill) < 1e-8 || any(~isfinite(y_b_in_hill))
        y_b_in_hill = [0;1;0];
    else
        y_b_in_hill = y_b_in_hill / norm(y_b_in_hill);
    end

    % Recompute x to improve orthogonality
    x_b_in_hill = cross(y_b_in_hill, z_b_in_hill);
    if norm(x_b_in_hill) < 1e-8 || any(~isfinite(x_b_in_hill))
        x_b_in_hill = [1;0;0];
    else
        x_b_in_hill = x_b_in_hill / norm(x_b_in_hill);
    end

    % Rotation from body frame to Hill frame
    % Columns are body axes expressed in Hill frame
    R_b_to_h = [x_b_in_hill, y_b_in_hill, z_b_in_hill];

    q_cmd = rotm_to_quat_scalar_first(R_b_to_h);

    % Final safety cleanup
    if any(~isfinite(q_cmd)) || norm(q_cmd) < 1e-12
        q_cmd = [1;0;0;0];
    else
        q_cmd = q_cmd / norm(q_cmd);
    end
end