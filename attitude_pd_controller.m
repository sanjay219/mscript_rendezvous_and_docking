function tau = attitude_pd_controller(q, qd, w, wd, gains)
% Quaternion PD attitude controller
% tau = -Kq * q_err_vec - Kw * w_err

qe = quat_multiply(quat_conj(qd), q);
if qe(1) < 0
    qe = -qe; % shortest rotation
end

qv = qe(2:4);
we = w - wd;

tau = -gains.Kq*qv - gains.Kw*we;
end
