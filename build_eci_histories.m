function [targetPosECI, chaserPosECI] = build_eci_histories(aT, inc, raan, argp, nu0, mu, t_total, X_total_rel)
N = numel(t_total);
targetPosECI = zeros(3,N);
chaserPosECI = zeros(3,N);
for k = 1:N
    tk = t_total(k);
    [rT, vT] = circularOrbitPropagate(aT, inc, raan, argp, nu0, mu, tk);
    targetPosECI(:,k) = rT;

    ex = rT / norm(rT);
    h = cross(rT, vT);
    ez = h / norm(h);
    ey = cross(ez, ex);
    C_H_to_ECI = [ex ey ez];

    rho_hill = X_total_rel(1:3,k);
    chaserPosECI(:,k) = rT + C_H_to_ECI * rho_hill;
end
end
