
function q_new = ekf_update(q, gyro, accel, mag, dt)

q     = q(:);
gyro  = gyro(: );
accel = accel(: );
mag   = mag(: );


% Referenzvektoren
accel_ref = [-1; -1; 1];
mag_ref   = [0.44; 0.03; 0.9];  % oder aus erster Messung setzen


persistent P
if isempty(P) || any(any(~isfinite(P)))
    P = eye(4);
end

gyro_noise   = 0.09;
accel_noise  = 0.25;
magnet_noise = 0.64;

% gyro_noise   = 0.0000;
% accel_noise  = 0.0002;
% magnet_noise = 3.0814e-14;  % in T²

% Normalize
accel_n = accel / norm(accel);
mag_n   = mag   / norm(mag);

% Predict
q_pred = normalize_q(q + 0.5 * dt * omega_mat(gyro) * q);
F      = eye(4) + 0.5 * dt * omega_mat(gyro);
W      = 0.5 * dt * W_mat(q);
P_pred = F * P * F' + (W * W') * gyro_noise;

% Measurement
z = [accel_n; mag_n];  % 6x1
h = [rot_vec(q_pred, accel_ref); rot_vec(q_pred, mag_ref)];

H = [obs_jacobian(q_pred, accel_ref); obs_jacobian(q_pred, mag_ref)];  % 6x4

R = diag([accel_noise accel_noise accel_noise magnet_noise magnet_noise magnet_noise]);

S = H * P_pred * H' + R;
K = P_pred * H' / S;  % 4x6

P     = (eye(4) - K * H) * P_pred;
q_new = normalize_q(q_pred + K * (z - h));
end

% ── Hilfsfunktionen ──────────────────────────────────────────────────────────

function q = normalize_q(q)
q = q / norm(q);
end

function M = omega_mat(w)
M = [ 0,   -w(1), -w(2), -w(3);
    w(1),  0,    w(3), -w(2);
    w(2), -w(3),  0,    w(1);
    w(3),  w(2), -w(1),  0  ];
end

function W = W_mat(q)
W = [-q(2), -q(3), -q(4);
    q(1), -q(4),  q(3);
    q(4),  q(1), -q(2);
    -q(3),  q(2),  q(1)] * 0.5;
end

function v = rot_vec(q, ref)
qw=q(1); qx=q(2); qy=q(3); qz=q(4);
R = [1-2*(qy^2+qz^2),   2*(qx*qy+qw*qz),   2*(qx*qz-qw*qy);
    2*(qx*qy-qw*qz),   1-2*(qx^2+qz^2),   2*(qy*qz+qw*qx);
    2*(qx*qz+qw*qy),   2*(qy*qz-qw*qx),   1-2*(qx^2+qy^2)];
v = R * ref;
end

function H = obs_jacobian(q, ref)
qw=q(1); qx=q(2); qy=q(3); qz=q(4);
ax=ref(1); ay=ref(2); az=ref(3);
H = 2 * [
    ax*qw+ay*qz-az*qy,   ax*qx+ay*qy+az*qz,  -ax*qy+ay*qx-az*qw,  -ax*qz+ay*qw+az*qx;
    -ax*qz+ay*qw+az*qx,   ax*qy-ay*qx+az*qw,   ax*qx+ay*qy+az*qz,  -ax*qw-ay*qz+az*qy;
    ax*qy-ay*qx+az*qw,   ax*qz-ay*qw-az*qx,   ax*qw+ay*qz-az*qy,   ax*qx+ay*qy+az*qz];
end