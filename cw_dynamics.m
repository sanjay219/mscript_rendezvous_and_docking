function dx = cw_dynamics(x, u, n)
dx = zeros(6,1);
dx(1:3) = x(4:6);
dx(4) =  2*n*x(5) + 3*n^2*x(1) + u(1);
dx(5) = -2*n*x(4) + u(2);
dx(6) = -n^2*x(3) + u(3);
end
