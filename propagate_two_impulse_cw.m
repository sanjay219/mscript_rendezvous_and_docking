function X = propagate_two_impulse_cw(x0, dv1, dv2, t1, dt2, n, thist)
X = zeros(6,numel(thist));
for k = 1:numel(thist)
    t = thist(k);
    if t <= t1
        X(:,k) = cw_propagate(x0, t, n);
    else
        x1m = cw_propagate(x0, t1, n);
        x1p = x1m + [0;0;0; dv1];
        if t <= t1 + dt2
            X(:,k) = cw_propagate(x1p, t-t1, n);
        else
            x2m = cw_propagate(x1p, dt2, n);
            x2p = x2m + [0;0;0; dv2];
            X(:,k) = cw_propagate(x2p, t-(t1+dt2), n);
        end
    end
end
end
