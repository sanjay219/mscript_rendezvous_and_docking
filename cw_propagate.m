function x = cw_propagate(x0, t, n)
A = cwA(n);
Phi = expm(A*t);
x = Phi*x0;
end
