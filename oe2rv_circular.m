function [r,v] = oe2rv_circular(a, inc, raan, argp, nu, mu)
p = a;
r_pf = p*[cos(nu); sin(nu); 0];
v_pf = sqrt(mu/p)*[-sin(nu); cos(nu); 0];

R3_W = [cos(raan) -sin(raan) 0;
        sin(raan)  cos(raan) 0;
        0          0         1];
R1_i = [1 0 0;
        0 cos(inc) -sin(inc);
        0 sin(inc)  cos(inc)];
R3_w = [cos(argp) -sin(argp) 0;
        sin(argp)  cos(argp) 0;
        0          0         1];
Q = R3_W*R1_i*R3_w;
r = Q*r_pf;
v = Q*v_pf;
end
