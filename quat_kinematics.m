function qdot = quat_kinematics(q, w)
%QUAT_KINEMATICS Quaternion kinematics for scalar-first quaternion
%
% Inputs:
%   q = [q0; q1; q2; q3]   scalar-first quaternion
%   w = [wx; wy; wz]       body angular velocity [rad/s]
%
% Output:
%   qdot = [q0dot; q1dot; q2dot; q3dot]

    q = q(:);
    w = w(:);

    if numel(q) ~= 4
        error('quat_kinematics: q must be a 4x1 vector.');
    end

    if numel(w) ~= 3
        error('quat_kinematics: w must be a 3x1 vector.');
    end

    wx = w(1);
    wy = w(2);
    wz = w(3);

    Omega = [  0   -wx   -wy   -wz;
              wx     0    wz   -wy;
              wy   -wz     0    wx;
              wz    wy   -wx     0 ];

    qdot = 0.5 * Omega * q;
end