function [q_next, w_next] = rk4_attitude_step(q, w, tau, dt, Ibody, IbodyInv)
%RK4_ATTITUDE_STEP One RK4 step for quaternion + rigid-body attitude dynamics
%
% State:
%   q = [q0 q1 q2 q3]'   scalar-first quaternion
%   w = [wx wy wz]'      body angular velocity [rad/s]

    x = [q(:); w(:)];

    f = @(xx) local_attitude_dynamics(xx, tau, Ibody, IbodyInv);

    k1 = f(x);
    k2 = f(x + 0.5*dt*k1);
    k3 = f(x + 0.5*dt*k2);
    k4 = f(x + dt*k3);

    x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

    q_next = x_next(1:4);
    w_next = x_next(5:7);

    % Normalize quaternion for numerical stability
    if any(~isfinite(q_next))
        q_next = [1;0;0;0];
    end

    nq = norm(q_next);
    if nq < 1e-12 || ~isfinite(nq)
        q_next = [1;0;0;0];
    else
        q_next = q_next / nq;
    end

    if any(~isfinite(w_next))
        w_next = [0;0;0];
    end
end

% ============================================================
function dx = local_attitude_dynamics(x, tau, Ibody, IbodyInv)

    q = x(1:4);
    w = x(5:7);

    qdot = quat_kinematics(q, w);
    wdot = rigid_body_dynamics(w, tau, Ibody, IbodyInv);

    dx = [qdot; wdot];
end