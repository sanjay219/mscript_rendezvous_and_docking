function J = objective_dv(p)
% Objective: minimize sum of squared DV magnitudes
% p = [t1 dt2 dv1x dv1y dv1z dv2x dv2y dv2z]'
dv1 = p(3:5);
dv2 = p(6:8);
J = norm(dv1)^2 + norm(dv2)^2;
end
