close all
clear
clc
delete(serialportfind)
clear ekf_update



fs = 100;
try
    s = serialport("/dev/cu.usbserial-10", 115200);
catch
    s = serialport("/dev/cu.usbserial-110", 115200);
end
configureTerminator(s, "LF");
flush(s);

FUSE = ahrsfilter('SampleRate', fs);
t_last = tic;

viewer = poseplot;

accel = zeros(1,3);
gyro  = zeros(1,3);
mag   = zeros(1,3);

%b = [30.7073, -19.2218, -46.3756];
%b = [42.9229, 101.1279, -5.0567];
%b = [1.0e+03 * -1.4673, 1.0e+03 * -0.0265, 1.0e+03 * 0.3888]; %x and y swapped
A = eye(3);
b = [1.0e+03 * 2.1045    0.8154    0.8486];

% A = [1.0962         0         0
%     0    1.4670         0
%     0         0    0.6219];
% b = [-115.4853    6.6805  -14.4159];

% Vor der while-Loop, ~100 Samples bei Stillstand sammeln
gyro_bias = zeros(1,3);
% acce_bias = zeros(1,3);

for i = 1:100
    line = readline(s);
    if contains(line, "Gyro_x"), gyro_bias(3) = gyro_bias(3) + sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Gyro_y"), gyro_bias(2) = gyro_bias(2) + sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Gyro_z"), gyro_bias(1) = gyro_bias(1) + sscanf(extractAfter(line,": "),"%f"); end
    % if contains(line, "Acce_x"), acce_bias(3) = acce_bias(3) + sscanf(extractAfter(line,": "),"%f"); end
    % if contains(line, "Acce_y"), acce_bias(2) = acce_bias(2) + sscanf(extractAfter(line,": "),"%f"); end
    % if contains(line, "Acce_z"), acce_bias(1) = acce_bias(1) + sscanf(extractAfter(line,": "),"%f"); end
end
gyro_bias = gyro_bias / 100;
% acce_bias = acce_bias / 100;


q = [1; 0; 0; 0];

% GPS-Koordinaten (Konstanz als Beispiel)
lat = 47.6779;
lon = 9.1732;
alt = 0.4;  % km über NN
date_str = '2026-05-27';

url = sprintf('https://www.geomag.bgs.ac.uk/web_service/GMModels/wmm/2025?latitude=%.4f&longitude=%.4f&altitude=%.1f&date=%s&resultFormat=json', ...
    lat, lon, alt, date_str);

opts = weboptions('CertificateFilename', '');
data = webread(url, opts);

% Feldkomponenten in nT
X = data.geomagnetic_field_model_result.field_value.north_intensity.value;  % Nord
Y = data.geomagnetic_field_model_result.field_value.east_intensity.value;   % Ost
Z = data.geomagnetic_field_model_result.field_value.vertical_intensity.value; % Down

% Referenzvektor für EKF (normalisiert, NED-Rahmen)
mag_ref_world = [X; Y; Z];
mag_ref_world = mag_ref_world / norm(mag_ref_world);

line = readline(s);
% Beispiel: ">imu/Acce_x: 10.0724"
if contains(line, "Acce_x"), accel(3) = sscanf(extractAfter(line,": "), "%f"); end
if contains(line, "Acce_y"), accel(2) = sscanf(extractAfter(line,": "), "%f"); end
if contains(line, "Acce_z"), accel(1) = sscanf(extractAfter(line,": "), "%f"); end
if contains(line, "mag/x"),  mag(1)   = sscanf(extractAfter(line,": "), "%f"); end
if contains(line, "mag/y"),  mag(2)   = sscanf(extractAfter(line,": "), "%f"); end
if contains(line, "mag/z"),  mag(3) = sscanf(extractAfter(line,": "), "%f"); end

fprintf("accelref: %.2f %.2f %.2f\n", accel(3), accel(2), accel(1));
fprintf("magref: %.2f %.2f %.2f\n", mag(1), mag(2), mag(3));


pause(1);


while true
    line = readline(s);
    % Beispiel: ">imu/Acce_x: 10.0724"
    if contains(line, "Acce_x"), accel(3) = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Acce_y"), accel(2) = -sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Acce_z"), accel(1) = -sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Gyro_x"), gyro(3)  = -sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Gyro_y"), gyro(2)  = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Gyro_z"), gyro(1)  = sscanf(extractAfter(line,": "), "%f"); end

    if contains(line, "mag/x"),  mag(1)   = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "mag/y"),  mag(2)   = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "mag/z")
        mag(3) = sscanf(extractAfter(line,": "), "%f");

        dt = toc(t_last);
        t_last = tic;

        %use calibration results
        mag_calibrated = (mag - b) * A;
        %mag_calibrated = mag;

        % fprintf("mag: %.2f %.2f %.2f\n", mag_ref_world(1), mag_ref_world(2), mag_ref_world(3));

        % vollständiges Sample → fusionieren
        %q = FUSE((accel - acce_bias), (gyro - gyro_bias) * (pi/180), mag_calibrated * 1e-6); % µT → T

        % q = FUSE(accel, gyro, mag_calibrated); % µT → T
        % viewer.Orientation = q;

        % q_out = ahrs_ekf12(accel, gyro, mag_calibrated, dt);
        % viewer.Orientation = quaternion(q_out(1),q_out(2),q_out(3),q_out(4));

        % eul = eulerd(q, 'ZYX', 'frame');  % [yaw, pitch, roll] in Grad
        % fprintf("Roll: %.2f°  Pitch: %.2f°  Yaw: %.2f°\n", eul(3), eul(2), eul(1));

        % q = ekf_update(q, -gyro, accel*9.81, mag_calibrated*1e-6, dt);
        % viewer.Orientation = quaternion(q(1), q(2), q(3), q(4));
        %

        % angles = eulerd(q1, 'ZYX', 'frame');
        % fprintf("Yaw: %.2f°  Pitch: %.2f°  Roll: %.2f°\n", angles(1), angles(2), angles(3));




        q_out = ekf_update(q, gyro, accel, mag_calibrated, dt);
        viewer.Orientation = quaternion(q_out(1), q_out(2), q_out(3), q_out(4));
        % %
        % %  fprintf("qout: %.2f°\n",rad2deg(q_out(4)));
        % % %
        q = q_out;

    end
    % fprintf("Accel: X=%.2f  Y=%.2f  Z=%.2f\n", accel(3), accel(2), accel(1));
    % fprintf("Gyro: X=%.2f  Y=%.2f  Z=%.2f\n", gyro(3), gyro(2), gyro(1));
    % fprintf("Mag: X=%.2f  Y=%.2f  Z=%.2f\n", mag(1), mag(2), mag(3));

end