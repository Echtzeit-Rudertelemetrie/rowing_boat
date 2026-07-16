% sensor_noise_calibration.m
% Sensor auf flacher, ruhiger Unterlage liegen lassen — waehrend der
% Messung NICHT bewegen!
%
% Liest den Rohdaten-Stream von test_imu_stream_s3.cpp (env:xiao_s3_matlab),
% Zeilenformat ">imu/Acce_x: 1.2345" usw., mag/z = letzte Zeile pro Sample.
% Einheiten wie von der Firmware gesendet: Accel m/s², Gyro rad/s, Mag µT
% (Mag-Achsen bereits in den Accel/Gyro-Frame gedreht, aber noch OHNE
% Hard-/Soft-Iron-Kalibrierung — magcal steht noch aus).
%
% WICHTIG: ekf_update_neu.m braucht die drei Rauschwerte in unterschied-
% lichen Skalen:
%   gyro_noise   -> Varianz des ROHEN Gyro (rad/s)^2 (Gyro geht roh in die
%                   Praediktion ein)
%   accel_noise  -> Varianz des NORMIERTEN Accel-Vektors accel/norm(accel),
%                   dimensionslos, weil z/h in ekf_update_neu.m normierte
%                   Vektoren sind
%   magnet_noise -> Varianz des NORMIERTEN Mag-Vektors, ebenso dimensionslos
% Rohe physikalische Varianzen (m/s²)^2 bzw. µT^2 gehoeren NICHT direkt in
% accel_noise/magnet_noise — das war der Grund, warum die EKF-Korrektur zu
% langsam war (R war um Groessenordnungen zu groß fuer den normierten Raum).
%
% ─────────────────────────────────────────────────────────────────────────────
% TODO (2026-07-15): Rechnung an die Firmware angleichen.
% AngleReader::calibrateSensorNoise() rechnet inzwischen bewusst anders; solange
% dieses Skript nicht nachzieht, sind die hier gemessenen Werte NICHT die, die
% auf dem ESP wirken:
%
%   1) VARIANZ JE ACHSE STATT EINES SKALARS.
%      Der EKF nimmt jetzt Vec3-Rauschwerte: Prozessrauschen als W*Sigma_w*W'
%      (Sigma_w = diag der drei Gyro-Achsvarianzen) statt (W*W')*sigma^2, und R
%      als echte Diagonale. Die Annahme sigma_wx = sigma_wy = sigma_wz trifft
%      beim realen Gyro nie zu. mean(var(...)) unten wirft genau das weg.
%
%   2) ROHE Varianz messen und mit 1/|v|^2 skalieren, statt var() auf dem
%      NORMIERTEN Vektor zu rechnen. Grund: Normieren entfernt das Rauschen
%      entlang der dominanten Richtung — die Schwerkraft-Achse zeigt dann eine
%      Varianz von ~1e-13. Das ist geometrisch korrekt, aber WELCHE Achse es
%      trifft, haengt an der Kalibrierlage. Per-Achsen-R aus normierten Daten
%      betoniert also die Messlage in R ein, und sobald sich die Dolle dreht,
%      passt R nicht mehr. Die rohe Achsvarianz ist dagegen eine echte
%      Sensoreigenschaft (MEMS-Achsen rauschen unterschiedlich) und dreht mit
%      dem Body-Frame mit. Gemeint ist:
%          accel_var(i) = var(accel_raw(:,i)) / mean(vecnorm(accel_raw,2,2))^2
%          mag_var(i)   = var(mag_raw(:,i))   / mean(vecnorm(mag_raw,2,2))^2
%      Das aktuelle mean(var(accel_n)) mittelt die Radial-Achse (~0) mit den
%      beiden Tangential-Achsen und liegt dadurch ca. Faktor 1.5 zu niedrig.
%
%   3) Der Gyro bleibt roh und unskaliert — er geht roh in die Praediktion ein.
%
%   4) var() ist hier numerisch unkritisch (double), die Firmware musste dafuer
%      auf Welford umstellen: E[x^2]-E[x]^2 loescht sich in float aus, wenn der
%      Mittelwert das Rauschen dominiert (Accel ~9.81 vs ~0.009) — dort kamen
%      negative Varianzen heraus. Beim Vergleich der Zahlen beachten: die
%      Firmware normiert mit (n-1), wie MATLABs var().
%
% Achtung Reihenfolge: die Varianzen muessen im EKF-Frame vorliegen, also NACH
% der zyklischen Achsvertauschung (y,z,x) — siehe TODO in main_test_S3.m.
% ─────────────────────────────────────────────────────────────────────────────

close all; clear; clc;
delete(serialportfind)

%% Config
N_SAMPLES = 1000; % Anzahl Samples pro Achse

%% Collect
s = serialport;
configureTerminator(s, "LF");
flush(s);

fprintf("Collecting %d samples — Sensor RUHIG halten!\n", N_SAMPLES);

accel_raw = zeros(N_SAMPLES, 3);
gyro_raw  = zeros(N_SAMPLES, 3);
mag_raw   = zeros(N_SAMPLES, 3);

accel_buf = zeros(1,3);
gyro_buf  = zeros(1,3);
mag_buf   = zeros(1,3);

count = 0;
while count < N_SAMPLES
    line = readline(s);
    if contains(line, "Acce_x"), accel_buf(1) = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Acce_y"), accel_buf(2) = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Acce_z"), accel_buf(3) = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Gyro_x"), gyro_buf(1)  = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Gyro_y"), gyro_buf(2)  = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Gyro_z"), gyro_buf(3)  = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "mag/x"),  mag_buf(1)   = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "mag/y"),  mag_buf(2)   = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "mag/z")
        mag_buf(3) = sscanf(extractAfter(line,": "),"%f");
        count = count + 1;
        accel_raw(count,:) = accel_buf;
        gyro_raw(count,:)  = gyro_buf;
        mag_raw(count,:)   = mag_buf;
        if mod(count, 100) == 0
            fprintf("  %d / %d\n", count, N_SAMPLES);
        end
    end
end
delete(s);
fprintf("Done collecting.\n\n");

%% Rohstatistik (nur zur Info)
labels   = ["Accel X","Accel Y","Accel Z","Gyro X","Gyro Y","Gyro Z","Mag X","Mag Y","Mag Z"];
data_raw = [accel_raw, gyro_raw, mag_raw];
vars_raw = var(data_raw);

fprintf("%-10s  %10s  %10s  %10s\n", "Sensor", "Mean", "Std Dev", "Variance");
fprintf("%s\n", repmat('-',1,48));
for i = 1:9
    fprintf("%-10s  %10.5f  %10.5f  %10.5f\n", labels(i), mean(data_raw(:,i)), sqrt(vars_raw(i)), vars_raw(i));
end

%% Normierte Vektoren -- genau das, was ekf_update_neu.m als Messung z sieht
accel_n = accel_raw ./ vecnorm(accel_raw, 2, 2);
mag_n   = mag_raw   ./ vecnorm(mag_raw,   2, 2);

accel_noise_ekf  = mean(var(accel_n));
magnet_noise_ekf = mean(var(mag_n));
gyro_noise_ekf   = mean(var(gyro_raw));   % Gyro bleibt roh -> rad/s

fprintf("\n=== Rohwerte, physikalische Einheiten (nur zur Info) ===\n");
fprintf("accel  (m/s^2)^2 : %.6g\n", mean(var(accel_raw)));
fprintf("mag    (uT)^2    : %.6g\n", mean(var(mag_raw)));

fprintf("\n=== Diese Werte in ekf_update_neu.m einsetzen ===\n");
fprintf("gyro_noise   = %.6g;  %% (rad/s)^2, roh\n", gyro_noise_ekf);
fprintf("accel_noise  = %.6g;  %% normierter Accel-Vektor, dimensionslos\n", accel_noise_ekf);
fprintf("magnet_noise = %.6g;  %% normierter Mag-Vektor, dimensionslos\n", magnet_noise_ekf);
