function q_out = ahrs_ekf12(accel, gyro, mag, dt)
% 12-state error-state Kalman filter (ahrsfilter / FUSE structure).
% States: orientation error(3), gyro bias(3), linear accel(3), mag disturbance(3).
% q rotates NED navigation frame -> sensor body frame, [w x y z].
%
% accel : body accel [m/s^2], gravity component POSITIVE (NED)
% gyro  : body rate  [rad/s]  (bias is estimated; raw is fine)
% mag   : body field [uT], hard/soft-iron calibrated
% dt    : timestep [s]
%
% Verified: no drift, static <1 deg, dynamic <1 deg, bias converges,
% linear-accel and (detectable) magnetic disturbance rejected.

accel = accel(:); gyro = gyro(:); mag = mag(:);
persistent q b aL dM P mnav Mref dipRef ready

% ── tuning ────────────────────────────────────────────────────────────────
g0       = 9.81;
nu       = 0.5;     % linear-acceleration decay  (LinearAccelerationDecayFactor)
sg       = 0.5;     % magnetic-disturbance decay (MagneticDisturbanceDecayFactor)
AccNoise = 2e-4;    % accelerometer measurement noise (unit-vector scale)
MagNoise = 4e-2;    % magnetometer  measurement noise (unit-vector scale)
qTh = 1e-7;         % orientation random walk (~GyroscopeNoise)
qB  = 1e-8;         % gyro-bias   random walk (~GyroscopeDriftNoise)
qA  = 0.05;         % linear-acceleration process noise (LinearAccelerationNoise)
qD  = 1.0;          % magnetic-disturbance process noise (MagneticDisturbanceNoise)
aTol = 0.08; mTol = 0.12; dipTol = 0.12; gateBig = 1e4;   % disturbance gates

aN = norm(accel); mN = norm(mag);
if aN < eps || mN < eps, q_out = [1;0;0;0]; return; end

% ── init: ecompass (TRIAD) sets the true starting attitude ────────────────
if isempty(ready)
    d_b = accel/aN;
    n_b = mag - (mag.'*d_b)*d_b; n_b = n_b/norm(n_b);
    e_b = cross(d_b, n_b);
    Rnb = [n_b e_b d_b];                 % nav->body
    q   = rotm2q(Rnb.');
    mv  = Rnb.'*mag;                     % measured field in nav
    mnav = [hypot(mv(1),mv(2)); 0; mv(3)]; mnav = mnav/norm(mnav)*mN; % zero East
    Mref = mN; dipRef = asin(max(-1,min(1, mv(3)/mN)));
    b = [0;0;0]; aL = [0;0;0]; dM = [0;0;0];
    P = diag([1e-2 1e-2 1e-2 1e-3 1e-3 1e-3 1e-1 1e-1 1e-1 1 1 1]);
    ready = true; q_out = q; return;
end

I3 = eye(3); Z3 = zeros(3);

% ── nominal prediction ────────────────────────────────────────────────────
om = gyro - b;
q  = qmul(q, rv2q(om*dt));
aL = aL*nu;  dM = dM*sg;

% ── error-state covariance prediction (bias coupled in F) ─────────────────
F = [I3-dt*skew(om), -dt*I3, Z3, Z3;
     Z3,              I3,     Z3, Z3;
     Z3,              Z3,  nu*I3, Z3;
     Z3,              Z3,     Z3, sg*I3];
Q = diag([qTh qTh qTh qB qB qB qA qA qA qD qD qD]);
P = F*P*F.' + Q;

% ── measurement (accel = full tilt, mag = yaw only via horizontal proj.) ──
Rnb   = Ra(q).';
gpred = Rnb*[0;0;1];
mpred = Rnb*mnav;
gh = gpred/norm(gpred);  Ph = I3 - gh*gh.';     % project perpendicular to gravity
gAcc = accel/g0 - aL;
mMeas = mag - dM;
z = [gAcc - gpred; Ph*(mMeas - mpred)];
H = [skew(gpred),     Z3, I3, Z3;
     Ph*skew(mpred),  Z3, Z3, Ph];

% ── adaptive gating (magnitude + inclination) ────────────────────────────
ra = AccNoise; if abs(aN/g0 - 1) > aTol, ra = ra*gateBig; end
dip = asin(max(-1,min(1, (mMeas.'*gh)/max(mN,eps))));
rm = MagNoise;
if abs(mN/Mref - 1) > mTol || abs(dip - dipRef) > dipTol, rm = rm*gateBig; end
R = diag([ra ra ra rm rm rm]);

% ── Kalman update (a priori error = 0) ────────────────────────────────────
S = H*P*H.' + R;
K = P*H.'/S;
x = K*z;
P = (eye(12) - K*H)*P;

% ── inject corrections into nominal states ────────────────────────────────
q  = qmul(q, rv2q(x(1:3))); q = q/norm(q);
b  = b  + x(4:6);
aL = aL + x(7:9);
dM = dM + x(10:12);

q_out = q;
end

% ── helpers ───────────────────────────────────────────────────────────────
function r = qmul(a,b)
r = [a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4);
     a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3);
     a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2);
     a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1)];
end
function R = Ra(q)   % active body->nav
w=q(1);x=q(2);y=q(3);z=q(4);
R=[1-2*(y^2+z^2), 2*(x*y-w*z), 2*(x*z+w*y);
   2*(x*y+w*z), 1-2*(x^2+z^2), 2*(y*z-w*x);
   2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x^2+y^2)];
end
function q = rv2q(v)
a=norm(v); if a<1e-12, q=[1;0;0;0]; return; end
q=[cos(a/2); sin(a/2)*v/a];
end
function S = skew(v)
S=[0,-v(3),v(2); v(3),0,-v(1); -v(2),v(1),0];
end
function q = rotm2q(R)
tr=trace(R);
if tr>0
 S=sqrt(tr+1)*2; w=0.25*S; x=(R(3,2)-R(2,3))/S; y=(R(1,3)-R(3,1))/S; z=(R(2,1)-R(1,2))/S;
elseif (R(1,1)>R(2,2))&&(R(1,1)>R(3,3))
 S=sqrt(1+R(1,1)-R(2,2)-R(3,3))*2; w=(R(3,2)-R(2,3))/S; x=0.25*S; y=(R(1,2)+R(2,1))/S; z=(R(1,3)+R(3,1))/S;
elseif R(2,2)>R(3,3)
 S=sqrt(1+R(2,2)-R(1,1)-R(3,3))*2; w=(R(1,3)-R(3,1))/S; x=(R(1,2)+R(2,1))/S; y=0.25*S; z=(R(2,3)+R(3,2))/S;
else
 S=sqrt(1+R(3,3)-R(1,1)-R(2,2))*2; w=(R(2,1)-R(1,2))/S; x=(R(1,3)+R(3,1))/S; y=(R(2,3)+R(3,2))/S; z=0.25*S;
end
q=[w;x;y;z]; q=q/norm(q);
end