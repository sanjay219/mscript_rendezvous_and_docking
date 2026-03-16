clear; clc; close all;

%% ============================================================
% MODULAR R-BAR RENDEZVOUS AND DOCKING FRAMEWORK
% Research-style baseline architecture:
%   1) Two-impulse rendezvous optimization using CW equations
%   2) Closed-loop translational docking controller (LQR)
%   3) Rigid-body attitude dynamics with quaternion PD control
%   4) satelliteScenario visualization with colored trajectories
%
% Notes:
% - This is a strong baseline framework, not a full Simulink clone of the
%   MathWorks example.
% - Translational dynamics use CW/Hill equations around a circular target orbit.
% - Attitude dynamics are propagated separately using rigid body dynamics.
%% ============================================================

%% ----------------------------
% CONSTANTS
%% ----------------------------
mu = 3.986004418e14;          % Earth GM [m^3/s^2]
Re = 6378137;                 % Earth radius [m]

%% ----------------------------
% TARGET ORBIT (circular reference for CW)
%% ----------------------------
altTarget = 500e3;
aT = Re + altTarget;
inc = deg2rad(30);
raan = deg2rad(40);
argp = 0;
nu0 = deg2rad(20);

n = sqrt(mu/aT^3);
Torb = 2*pi/n;

fprintf('Mean motion n = %.6e rad/s\n', n);
fprintf('Orbital period = %.2f min\n', Torb/60);

%% ----------------------------
% INITIAL / DESIRED RELATIVE STATE (Hill frame)
% x = radial, y = along-track, z = cross-track
%% ----------------------------
x0_rel = [-5000; 100; 50; 0; 0; 0];
x_rdv_des = [-1000; 0; 0; 0; 0; 0];

%% ----------------------------
% TWO-IMPULSE RENDEZVOUS OPTIMIZATION
% p = [t1 dt2 dv1x dv1y dv1z dv2x dv2y dv2z]
%% ----------------------------
params.n = n;
params.x0 = x0_rel;
params.xdes = x_rdv_des;

% Initial guess for fsolve-like stage (simple seed here)
p_guess = [1500; 4000; 0.5; 0; 0; -0.5; 0; 0];

% Bounds
lb = [200;   10;  -5; -5; -5;  -5; -5; -5];
ub = [86400; 86400; 5;  5;  5;   5;  5;  5];

% First stage: solve equality constraints approximately
fprintf('\nRunning fsolve stage for initial guess...\n');
p_init = solve_initial_guess_fsolve(p_guess, params);

% Second stage: constrained optimization
fprintf('Running fmincon stage for optimized burns...\n');
p_opt = solve_optimal_burns_fmincon(p_init, lb, ub, params);

% Extract burns
burn.t1 = p_opt(1);
burn.dt2 = p_opt(2);
burn.dv1 = p_opt(3:5);
burn.dv2 = p_opt(6:8);
burn.totalDV = norm(burn.dv1) + norm(burn.dv2);

fprintf('\n===== OPTIMIZED RENDEZVOUS SOLUTION =====\n');
fprintf('Burn-1 time t1        = %.2f s\n', burn.t1);
fprintf('Burn-2 after dt2      = %.2f s\n', burn.dt2);
fprintf('dv1 [m/s]             = [%.4f %.4f %.4f]\n', burn.dv1);
fprintf('dv2 [m/s]             = [%.4f %.4f %.4f]\n', burn.dv2);
fprintf('Total DV [m/s]        = %.6f\n', burn.totalDV);

%% ----------------------------
% PROPAGATE RENDEZVOUS TRAJECTORY
%% ----------------------------
dt = 2.0;
t_rdv_total = burn.t1 + burn.dt2;
t_hist_rdv = 0:dt:t_rdv_total;

X_rdv = propagate_two_impulse_cw(x0_rel, burn.dv1, burn.dv2, burn.t1, burn.dt2, n, t_hist_rdv);
x_after_rdv = X_rdv(:,end);

fprintf('\nState after rendezvous [Hill frame]:\n');
disp(x_after_rdv.');

%% ----------------------------
% DOCKING PHASE (INITIAL APPROACH + TRANSPOSITION + FINAL APPROACH)
% Translational dynamics: CW + LQR tracking
% Attitude dynamics: quaternion rigid body dynamics + quaternion PD torque
%% ----------------------------

% Guidance waypoints in Hill frame
waypoints.initialApproach = [-100; 0; 0];
waypoints.transposition  = [20; 0; 20];
waypoints.finalApproach  = [0; 0; 0];

% Timing / thresholds
phase.initialToTranspositionRange = 100;   % m
phase.transpositionTargetTol = 1.0;        % m
phase.finalSlowdownRange = 10;             % m
phase.finalDockRange = 1.0;                % m
phase.finalDockSpeed = 0.05;               % m/s

% Spacecraft inertia for attitude model
Ibody = diag([120, 100, 80]);
IbodyInv = inv(Ibody);

% Initial attitude states [q0 q1 q2 q3]^T and body rates [rad/s]
q_target0 = [1;0;0;0];
w_target0 = [0;0;0];
q_chaser0 = [1;0;0;0];
w_chaser0 = [0;0;0];

% LQR controller for CW translational dynamics
[A,B] = cw_state_matrices(n);
Q = diag([5e-4 5e-3 5e-3 1e0 8e0 8e0]);
R = diag([1,1,1]);
K_lqr = lqr(A,B,Q,R);

% Attitude controller gains
att.Kq = diag([20,20,20]);
att.Kw = diag([200,200,200]);

% Translational accel saturation
u_max = 0.02;  % m/s^2

% Storage
maxDockTime = 12000;
t_hist_dock = 0:dt:maxDockTime;
NdockMax = numel(t_hist_dock);
X_dock = zeros(6,NdockMax);
U_dock = zeros(3,NdockMax);
q_target_hist = zeros(4,NdockMax);
q_chaser_hist = zeros(4,NdockMax);
w_target_hist = zeros(3,NdockMax);
w_chaser_hist = zeros(3,NdockMax);
tau_target_hist = zeros(3,NdockMax);
tau_chaser_hist = zeros(3,NdockMax);
phase_id = zeros(1,NdockMax);

X_dock(:,1) = x_after_rdv;
q_target_hist(:,1) = q_target0;
q_chaser_hist(:,1) = q_chaser0;
w_target_hist(:,1) = w_target0;
w_chaser_hist(:,1) = w_chaser0;

stopIndex = NdockMax;

for k = 1:NdockMax-1
    xk = X_dock(:,k);
    rk = xk(1:3);
    vk = xk(4:6);
    % --------------------
% Docking contact detection
% --------------------
r_contact = [-2;0;0];

dock_radius = 3;   % meters (safe distance between spacecraft centers)

dist = norm(rk);
vel_mag = norm(vk);

if dist < dock_radius && vel_mag < 0.1
    stopIndex = k;
    fprintf('\nDocking contact reached. Distance = %.2f m\n',dist);
    break
end

    % --------------------
% Phase selector (SIMPLIFIED)
% --------------------
if abs(rk(1)) > 30
    currentPhase = 1;   % normal closing phase
else
    currentPhase = 2;   % slow final docking phase
end
    phase_id(k) = currentPhase;

    % --------------------
    % Translational guidance
    % --------------------
    %[r_des, v_des] = guidance_docking_phase(rk, currentPhase, waypoints, phase);

if currentPhase == 1
    % From 1 km below target to 20 m below target
    r_des = [-20; 0; 0];
    v_des = [0.3; 0; 0];
else
    % Final docking from 20 m to 0
    r_des = [-dock_radius; 0; 0];
    v_des = [0.01; 0; 0];
end

    u = translational_lqr_controller(xk, [r_des; v_des], K_lqr);
    u = max(min(u, u_max), -u_max);
    U_dock(:,k) = u;

    X_dock(:,k+1) = rk4_cw_step(xk, u, dt, n);

    % --------------------
    % Attitude guidance
    % Target attitude held inertially fixed.
    % Chaser +x body points toward target in Hill frame approximation.
    % --------------------
% DEBUG MODE: freeze attitude
q_target_hist(:,k+1) = q_target_hist(:,k);
q_chaser_hist(:,k+1) = q_chaser_hist(:,k);
w_target_hist(:,k+1) = [0;0;0];
w_chaser_hist(:,k+1) = [0;0;0];
tau_target_hist(:,k) = [0;0;0];
tau_chaser_hist(:,k) = [0;0;0];
% Docking stop condition: stop at contact point, before body overlap
r_contact = [-2; 0; 0];

pos_err_contact = norm(X_dock(1:3,k+1) - r_contact);
vel_mag = norm(X_dock(4:6,k+1));

%if pos_err_contact < 0.5 && vel_mag < 0.1
%    stopIndex = k+1;
%    fprintf('\nDocking contact reached at t = %.2f s after rendezvous. Stopping simulation.\n', t_hist_dock(k+1));
%    break;
%end
end

X_dock = X_dock(:,1:stopIndex);
U_dock = U_dock(:,1:stopIndex);
q_target_hist = q_target_hist(:,1:stopIndex);
q_chaser_hist = q_chaser_hist(:,1:stopIndex);
w_target_hist = w_target_hist(:,1:stopIndex);
w_chaser_hist = w_chaser_hist(:,1:stopIndex);
tau_target_hist = tau_target_hist(:,1:stopIndex);
tau_chaser_hist = tau_chaser_hist(:,1:stopIndex);
phase_id = phase_id(1:stopIndex);
t_hist_dock = t_hist_dock(1:stopIndex);

fprintf('\n===== DOCKING RESULTS =====\n');
fprintf('Docking time after rendezvous = %.2f s\n', t_hist_dock(end));
fprintf('Final relative position [m]   = [%.4f %.4f %.4f]\n', X_dock(1,end), X_dock(2,end), X_dock(3,end));
fprintf('Final relative velocity [m/s] = [%.4f %.4f %.4f]\n', X_dock(4,end), X_dock(5,end), X_dock(6,end));

% Approximate docking control DV
if numel(t_hist_dock) > 1
    dv_docking = trapz(t_hist_dock, vecnorm(U_dock,2,1));
else
    dv_docking = 0;
end
fprintf('Approx docking control DV [m/s] = %.6f\n', dv_docking);
fprintf('Approx total mission DV [m/s]   = %.6f\n', burn.totalDV + dv_docking);

%% ----------------------------
% PLOTS IN HILL FRAME
%% ----------------------------
figure('Name','Relative Motion in Hill Frame');
plot3(X_rdv(2,:), X_rdv(3,:), X_rdv(1,:), 'b', 'LineWidth', 1.5); hold on;
plot3(X_dock(2,:), X_dock(3,:), X_dock(1,:), 'r', 'LineWidth', 2.0);
plot3(0,0,0,'ko','MarkerFaceColor','k','MarkerSize',8);
xlabel('Along-track y [m]');
ylabel('Cross-track z [m]');
zlabel('Radial x [m]');
title('Relative Motion: Rendezvous + Docking');
legend('Rendezvous','Docking','Target');
grid on; axis equal;

figure('Name','Relative Coordinates');
subplot(3,1,1); plot([t_hist_rdv, t_hist_rdv(end)+t_hist_dock], [X_rdv(1,:), X_dock(1,:)], 'LineWidth',1.4); ylabel('x [m]'); grid on;
subplot(3,1,2); plot([t_hist_rdv, t_hist_rdv(end)+t_hist_dock], [X_rdv(2,:), X_dock(2,:)], 'LineWidth',1.4); ylabel('y [m]'); grid on;
subplot(3,1,3); plot([t_hist_rdv, t_hist_rdv(end)+t_hist_dock], [X_rdv(3,:), X_dock(3,:)], 'LineWidth',1.4); ylabel('z [m]'); xlabel('Time [s]'); grid on;

figure('Name','Docking Control Accelerations');
plot(t_hist_dock, U_dock(1,:), 'LineWidth',1.5); hold on;
plot(t_hist_dock, U_dock(2,:), 'LineWidth',1.5);
plot(t_hist_dock, U_dock(3,:), 'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Acceleration command [m/s^2]');
legend('u_x','u_y','u_z'); title('Docking Controller Commands'); grid on;

figure('Name','Attitude Torque Commands (Chaser)');
plot(t_hist_dock, tau_chaser_hist(1,:), 'LineWidth',1.5); hold on;
plot(t_hist_dock, tau_chaser_hist(2,:), 'LineWidth',1.5);
plot(t_hist_dock, tau_chaser_hist(3,:), 'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Torque [N m]');
legend('\tau_x','\tau_y','\tau_z'); title('Chaser Attitude Controller Torque'); grid on;

%% ----------------------------
% BUILD ECI POSITION / ATTITUDE HISTORIES FOR VISUALIZATION
%% ----------------------------
t_total = [t_hist_rdv, (t_hist_rdv(end)+dt):dt:(t_hist_rdv(end)+t_hist_dock(end))];
Ntotal = numel(t_total);
X_total_rel = [X_rdv, X_dock(:,2:end)];

% For visualization, target attitude kept inertially fixed; chaser attitude uses
% the propagated quaternion history during docking and identity during rendezvous.
q_target_total = repmat([1 0 0 0], Ntotal, 1);
q_chaser_total = [repmat([1 0 0 0], numel(t_hist_rdv), 1); q_chaser_hist(:,2:end).'];

% Safety cleanup: ensure all quaternions are finite and unit norm
for i = 1:size(q_target_total,1)
    qi = q_target_total(i,:).';
    if any(~isfinite(qi)) || norm(qi) < 1e-12
        qi = [1;0;0;0];
    else
        qi = qi / norm(qi);
    end
    q_target_total(i,:) = qi.';
end

for i = 1:size(q_chaser_total,1)
    qi = q_chaser_total(i,:).';
    if any(~isfinite(qi)) || norm(qi) < 1e-12
        qi = [1;0;0;0];
    else
        qi = qi / norm(qi);
    end
    q_chaser_total(i,:) = qi.';
end

disp('Any non-finite in q_target_total?')
disp(any(~isfinite(q_target_total), 'all'))

disp('Any non-finite in q_chaser_total?')
disp(any(~isfinite(q_chaser_total), 'all'))

[targetPosECI, chaserPosECI] = build_eci_histories(aT, inc, raan, argp, nu0, mu, t_total, X_total_rel);
%% ----------------------------
% satelliteScenario visualization
%% ----------------------------
startTime = datetime(2026,3,15,0,0,0);
stopTime  = startTime + seconds(t_total(end));
sc = satelliteScenario(startTime, stopTime, dt);

% Timeseries
chaserPosTS = timeseries(chaserPosECI', t_total);
targetPosTS = timeseries(targetPosECI', t_total);
chaserAttTS = timeseries(q_chaser_total, t_total);
targetAttTS = timeseries(q_target_total, t_total);

chaser = satellite(sc, chaserPosTS, 'Name', 'Chaser');
pointAt(chaser, chaserAttTS);
chaserDockingPort = conicalSensor(chaser, 'MountingLocation', [0;0;0.5]); %#ok<NASGU>

target = satellite(sc, targetPosTS, 'Name', 'Target');
pointAt(target, targetAttTS);
targetDockingPort = conicalSensor(target, 'MountingLocation', [0;0;-0.5]); %#ok<NASGU>

v = satelliteScenarioViewer(sc, 'CameraReferenceFrame', 'Inertial'); %#ok<NASGU>
chaser.Visual3DModel = "SmallSat.glb";
chaser.Visual3DModelScale = 0.8;
target.Visual3DModel = "SmallSat.glb";
target.Visual3DModelScale = 0.8;
coordinateAxes([chaser target], 'Scale', 2);

% Different orbit colors
chaser.Orbit.LineColor = [1 0 0];
target.Orbit.LineColor = [0 0 1];

% Leave orbit lines visible because they help your visualization
% hide(chaser.Orbit)
% hide(target.Orbit)

disp('Type: play(sc)');
disp('Then use: camtarget(v,chaser)');

%% ============================================================
% END OF MAIN SCRIPT
%% ============================================================
