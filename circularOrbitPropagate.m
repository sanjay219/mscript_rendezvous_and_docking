function [r,v] = circularOrbitPropagate(a, inc, raan, argp, nu0, mu, t)
n = sqrt(mu/a^3);
nu = nu0 + n*t;
[r,v] = oe2rv_circular(a, inc, raan, argp, nu, mu);
end
