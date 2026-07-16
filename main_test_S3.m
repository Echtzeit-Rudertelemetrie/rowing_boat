close all
clear
clc
delete(serialportfind)
clear ekf_update
clear ekf_update_neu % setzt gelernte Referenzen + Anfangs-q + P zurueck



fs = 100;

s = serialport; %findet serialport automatisch....


configureTerminator(s, "LF");
flush(s);

FUSE = ahrsfilter('SampleRate', fs);
t_last = tic;

viewer = poseplot;
ax = gca;
view(ax, 0, 90);   % Draufsicht: Azimut 0°, Elevation 90°

accel = zeros(1,3);
gyro  = zeros(1,3);
mag   = zeros(1,3);

%b = [30.7073, -19.2218, -46.3756];
%b = [42.9229, 101.1279, -5.0567];
%b = [1.0e+03 * -1.4673, 1.0e+03 * -0.0265, 1.0e+03 * 0.3888]; %x and y swapped
% A = eye(3);
% b = [1.0e+03 * 2.1045    0.8154    0.8486];

A = [0.732492 0 0;0 1.59771 0;0 0 0.854473];
b = [49.6071 -89.4899 -73.1404];

q = [1; 0; 0; 0];

ref_printed = false;

% ─────────────────────────────────────────────────────────────────────────────
% TODO (2026-07-15): Achsen-Mapping an die Firmware angleichen. AKTUELL FALSCH.
%
% Zwei Probleme im Block unten:
%
%   1) accel/gyro werden als [z,y,x] gelesen (x und z getauscht), mag dagegen
%      1:1 als [x,y,z]. Schwerkraft und Magnetfeld liegen damit in VERSCHIEDENEN
%      Koordinatensystemen — der Winkel zwischen accel_ref und mag_ref ist
%      schlicht falsch, und genau die beiden fusioniert der EKF gegeneinander.
%
%   2) Der x<->z-Tausch hat Determinante -1, ist also eine SPIEGELUNG und keine
%      Drehung. Accel und Mag sind polare Vektoren, der Gyro ist ein Pseudo-
%      vektor — der transformiert sich unter einer Spiegelung ANDERS als die
%      beiden. Deshalb dreht sich in einem gespiegelten Frame die Drehrichtung
%      um, waehrend Schwerkraft und Nordrichtung normal aussehen. Vermutlich der
%      Grund, warum die Achsen urspruenglich getauscht wurden: das "falsche
%      Vorzeichen" war die Nebenwirkung der Spiegelung, nicht die Ursache.
%
% Die Firmware (AngleReader::readSample) ist die Referenz und macht die
% zyklische Vertauschung EKF = (body_y, body_z, body_x), det = +1:
%
%   % Stream ist bereits body-aligned (test_imu_stream_s3.cpp sendet mag als
%   % x,-y,-z, entsprechend ICM-20948-Datenblatt Kap. 10.1) -> hier NICHT
%   % nochmal drehen, sonst ist es wieder kaputt.
%   if contains(line,"Acce_x"), a(1) = ...; end   % alles 1:1 einlesen
%   if contains(line,"Acce_y"), a(2) = ...; end
%   if contains(line,"Acce_z"), a(3) = ...; end
%   % ... gyro nach g(1..3), mag nach m(1..3), ebenso 1:1
%
%   m_cal = (m - b) * A;        % Kalibrierung im Body-Frame, VOR der Permutation
%                               % (A/b wurden in genau diesem Frame gelernt!)
%   accel = [a(2), a(3), a(1)]; % zyklisch wie readSample()
%   gyro  = [g(2), g(3), g(1)];
%   mag   = [m_cal(2), m_cal(3), m_cal(1)];
%
% Faustregel: Mapping als 3x3-Matrix hinschreiben, det() muss +1 sein. EINE
% Achse zu negieren ist ebenfalls det = -1 (Spiegelung); zwei Achsen negieren
% ist det = +1 (180°-Drehung). Laeuft der Winkel danach in die falsche Richtung,
% das ERGEBNIS negieren — nicht am Frame drehen.
%
% Solange das nicht angeglichen ist, validiert dieses Skript eine andere Welt
% als die, die auf dem ESP laeuft: Rauschwerte und magcal-Fits sind dann nicht
% uebertragbar.
% ─────────────────────────────────────────────────────────────────────────────
while true
    line = readline(s);
    fprintf(line);
    % Beispiel: ">imu/Acce_x: 10.0724"
    if contains(line, "Acce_x"), accel(3) = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Acce_y"), accel(2) = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Acce_z"), accel(1) = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Gyro_x"), gyro(3)  = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Gyro_y"), gyro(2)  = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Gyro_z"), gyro(1)  = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "mag/x"),  mag(1)   = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "mag/y"),  mag(2)   = sscanf(extractAfter(line,": "), "%f"); end
    % mag/z ist die letzte Zeile eines Samples — erst hier ist das Tripel
    % vollstaendig, also laeuft nur hier der EKF-Schritt (dt ~ 10 ms).
    if contains(line, "mag/z")
        mag(3) = sscanf(extractAfter(line,": "), "%f");

       % in der Schleife, nach dem all(got)-Block:
        if ~ref_printed && any(gyro ~= 0 & isfinite(gyro))
            fprintf("gyroref:  %.4f %.4f %.4f\n", gyro);
            fprintf("accelref: %.4f %.4f %.4f\n", accel);
            fprintf("magref:   %.4f %.4f %.4f\n", mag);
            ref_printed = true;
        end

        % if ref_printed
            dt = toc(t_last);
            t_last = tic;
    
            %use calibration results
            mag_calibrated = (mag - b) * A;
            %mag_calibrated = mag;
    
            % fprintf("mag: %.2f %.2f %.2f\n", mag_ref_world(1), mag_ref_world(2), mag_ref_world(3));
    
            % vollständiges Sample → fusionieren
            %q = FUSE((accel - acce_bias), (gyro - gyro_bias) * (pi/180), mag_calibrated * 1e-6); % µT → T
    
            % q = FUSE(-accel, -gyro, mag_calibrated); % µT → T
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
    
    
    
    
            q_out = ekf_update_neu(q, gyro, accel, mag_calibrated, dt);
            q_view = quaternion(cosd(45), sind(45), 0, 0);   % -90° um X

            viewer.Orientation = q_view* quaternion(q_out(1), q_out(2), q_out(3), q_out(4));
            % %
            % fprintf("qout: %.2f°\n",rad2deg(q_out(4)));
            % % %
            q = q_out;
        % end

    end
    % fprintf("Accel: X=%.2f  Y=%.2f  Z=%.2f\n", accel(1), accel(2), accel(3));
    % fprintf("Gyro: X=%.2f  Y=%.2f  Z=%.2f\n", gyro(1), gyro(2), gyro(3));
    %  fprintf("Mag: X=%.2f  Y=%.2f  Z=%.2f\n", mag(1), mag(2), mag(3));

end