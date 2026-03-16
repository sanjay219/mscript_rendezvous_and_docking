function wdot = rigid_body_dynamics(w, tau, Ibody, IbodyInv)
%RIGID_BODY_DYNAMICS Euler rigid-body rotational dynamics
%
% Inputs:
%   w        = [wx; wy; wz] body angular velocity [rad/s]
%   tau      = [tx; ty; tz] control torque [N m]
%   Ibody    = 3x3 inertia matrix
%   IbodyInv = inverse of inertia matrix
%
% Output:
%   wdot     = angular acceleration [rad/s^2]
%
% Equation:
%   I * wdot + w x (I w) = tau
%   => wdot = I^{-1} ( tau - w x (I w) )

    w = w(:);
    tau = tau(:);

    if numel(w) ~= 3
        error('rigid_body_dynamics: w must be a 3x1 vector.');
    end

    if numel(tau) ~= 3
        error('rigid_body_dynamics: tau must be a 3x1 vector.');
    end

    if ~isequal(size(Ibody), [3 3])
        error('rigid_body_dynamics: Ibody must be 3x3.');
    end

    if ~isequal(size(IbodyInv), [3 3])
        error('rigid_body_dynamics: IbodyInv must be 3x3.');
    end

    h = Ibody * w;
    wdot = IbodyInv * (tau - cross(w, h));

    if any(~isfinite(wdot))
        wdot = [0;0;0];
    end
end