function ceq = rendezvous_constraint_eq(p, params)
% Equality constraints only for fsolve stage
n = params.n;
x0 = params.x0;
xdes = params.xdes;

t1 = p(1);
dt2 = p(2);
dv1 = p(3:5);
dv2 = p(6:8);

x1m = cw_propagate(x0, t1, n);
x1p = x1m + [0;0;0; dv1];

x2m = cw_propagate(x1p, dt2, n);
x2p = x2m + [0;0;0; dv2];

ceq = x2p - xdes;
end
