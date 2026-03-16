function u = translational_lqr_controller(x, xdes, K)
% LQR control law for translational CW dynamics
% u = -K (x - xdes)
e = x - xdes;
u = -K*e;
end
