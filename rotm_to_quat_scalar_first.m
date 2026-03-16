function q = rotm_to_quat_scalar_first(R)
%ROTM_TO_QUAT_SCALAR_FIRST Convert 3x3 rotation matrix to scalar-first quaternion
% q = [q0 q1 q2 q3]'

    % Basic sanity
    if any(~isfinite(R), 'all') || ~isequal(size(R), [3 3])
        q = [1;0;0;0];
        return;
    end

    tr = trace(R);

    if tr > 0
        S  = sqrt(tr + 1.0) * 2;
        q0 = 0.25 * S;
        q1 = (R(3,2) - R(2,3)) / S;
        q2 = (R(1,3) - R(3,1)) / S;
        q3 = (R(2,1) - R(1,2)) / S;
    elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        S  = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
        q0 = (R(3,2) - R(2,3)) / S;
        q1 = 0.25 * S;
        q2 = (R(1,2) + R(2,1)) / S;
        q3 = (R(1,3) + R(3,1)) / S;
    elseif R(2,2) > R(3,3)
        S  = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
        q0 = (R(1,3) - R(3,1)) / S;
        q1 = (R(1,2) + R(2,1)) / S;
        q2 = 0.25 * S;
        q3 = (R(2,3) + R(3,2)) / S;
    else
        S  = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
        q0 = (R(2,1) - R(1,2)) / S;
        q1 = (R(1,3) + R(3,1)) / S;
        q2 = (R(2,3) + R(3,2)) / S;
        q3 = 0.25 * S;
    end

    q = [q0; q1; q2; q3];

    if any(~isfinite(q)) || norm(q) < 1e-12
        q = [1;0;0;0];
    else
        q = q / norm(q);
    end
end