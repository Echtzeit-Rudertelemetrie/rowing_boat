function q_new = ekf_update(q, gyro, accel, mag, dt)

q     = q(:);
gyro  = gyro(:);
accel = accel(:);
mag   = mag(:);

% Referenzvektoren

% Konjugierte Quaternion: rot_vec(q, .) dreht Welt->Body, fuer die
% Referenz-Initialisierung wird die Umkehrung (Body->Welt) gebraucht.
% TODO (2026-07-15): Das ist NICHT die Konjugierte, sondern eine Kopie — der
% Vektorteil muesste negiert werden: q_conj = [q(1); -q(2); -q(3); -q(4)];
% (so macht es ekf_update.m und so macht es die Firmware in conjugate()).
% Aktuell folgenlos, weil main_test_S3.m beim ersten Aufruf mit q = [1;0;0;0]
% startet und die Konjugierte der Identitaet die Identitaet ist — aber eine
% Falle, sobald die Referenzen bei q != Identitaet gelernt werden.
q_conj = [q(1); q(2); q(3); q(4)];

% Accel-Referenz aus der ersten gueltigen Messung setzen (in den Welt-Frame
% gedreht, falls q beim ersten Aufruf nicht die Identitaet ist).
persistent accel_ref_init
if isempty(accel_ref_init) && norm(accel) > 1e-9
    accel_ref_init = rot_vec(q_conj, accel / norm(accel));
end
if isempty(accel_ref_init)
    accel_ref = [0; 0; 0];
else
    accel_ref = accel_ref_init;
end

% Mag-Referenz aus der ersten gueltigen Messung setzen (in den Welt-Frame
% gedreht, falls q beim ersten Aufruf nicht die Identitaet ist).
% Solange noch keine gueltige Messung kam, bleibt mag_ref = 0 und damit
% H_mag = 0 -> das Mag hat keinerlei Einfluss (Gain exakt 0).
persistent mag_ref_init
if isempty(mag_ref_init) && norm(mag) > 1e-9
    mag_ref_init = rot_vec(q_conj, mag / norm(mag));
end
if isempty(mag_ref_init)
    mag_ref = [0; 0; 0];
else
    mag_ref = mag_ref_init;
end

persistent P
if isempty(P) || any(any(~isfinite(P)))
    P = eye(4);
end

gyro_noise   = 0.09;
accel_noise  = 0.25;
magnet_noise = 0.64;

% Normalize
accel_n = accel / norm(accel);
if norm(mag) > 1e-9
    mag_n = mag / norm(mag);
else
    mag_n = [0; 0; 0];   % kein NaN in der Innovation, falls Mag (noch) leer
end

% Predict
q_pred = normalize_q(q + 0.5 * dt * omega_mat(gyro) * q);
F      = eye(4) + 0.5 * dt * omega_mat(gyro);
% TODO (2026-07-15): Das 0.5 ist hier DOPPELT — W_mat(q) enthaelt bereits ein
% *0.5 (siehe unten), diese Zeile multipliziert nochmal 0.5*dt. Ergebnis:
% W = 0.25*dt*M. Theoretisch korrekt ist 0.5*dt*M:
%     aus Omega(w)*q = M(q)*w folgt q_dot = 0.5*Omega(w)*q = 0.5*M(q)*w,
%     das Gyro-Rauschen geht also ueber G = 0.5*dt*M ein.
% Die Firmware (orientation_ekf.cpp) rechnet inzwischen korrekt mit 0.5*dt*M,
% dieses Skript liegt damit Faktor 2 in W und Faktor 4 in Q daneben.
% Fix:  W = dt * W_mat(q);
% Da Q linear mit gyro_noise skaliert, ist das effektiv nur ein Tuning-Faktor 4
% — aber solange es abweicht, rechnen Sim und Firmware nicht dasselbe.
W      = 0.5 * dt * W_mat(q);

% TODO (2026-07-15): Prozessrauschen als W*Sigma_w*W' statt (W*W')*sigma^2 und R
% als echte Diagonale — die Firmware macht das inzwischen so (Vec3-Rauschwerte).
% Die Annahme gleicher Achsvarianzen (sigma_wx = sigma_wy = sigma_wz) trifft beim
% realen Gyro praktisch nie zu. Siehe TODO in sensor_noise_calibration.m.
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
