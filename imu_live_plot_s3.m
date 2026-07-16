% imu_live_plot_s3.m — Live-Plot der ICM-20948-Rohachsen (XIAO ESP32-S3)
%
% Firmware: env "xiao_s3_matlab" im angle_ekf__wt-Worktree flashen
% (test/test_imu_stream_s3.cpp). Die streamt mit 100 Hz:
%   Accel [m/s²], Gyro [rad/s], Mag [µT, bereits in den Accel/Gyro-Frame gedreht]
%
% Achsen werden hier 1:1 geplottet, OHNE die Umsortierung aus main_test_S3.m:
% was im Plot "x" heisst, ist Sensor-X. Damit laesst sich direkt pruefen,
% welche Achse bei welcher Bewegung reagiert (Vorzeichen-Check Catch->Finish).

close all; clear; clc;
delete(serialportfind)

%% Port automatisch finden (XIAO S3 = natives USB-CDC -> /dev/cu.usbmodem*)
ports = serialportlist("available");
port = ports(contains(ports, "usbmodem"));
assert(~isempty(port), "Kein /dev/cu.usbmodem* gefunden — XIAO angeschlossen?");
s = serialport(port(end), 115200);
configureTerminator(s, "LF");
flush(s);

%% Ringpuffer & Plot-Setup
N  = 500;                 % Fensterbreite: 5 s bei 100 Hz
t  = (1-N:0) / 100;       % Sekunden, 0 = jetzt
accel = nan(N,3); gyro = nan(N,3); mag = nan(N,3);

fig = figure('Name', 'ICM-20948 live');
titles = ["Accel [m/s^2]", "Gyro [rad/s]", "Mag [\muT]"];
h = gobjects(3,3);
for k = 1:3
    subplot(3,1,k);
    hold on
    for a = 1:3
        h(k,a) = plot(t, nan(N,1));
    end
    hold off
    title(titles(k)); legend(["x","y","z"]); grid on
    xlabel("t [s]");
end

%% Empfangsschleife: mag/z schliesst ein Sample ab (wie in main_test_S3.m)
buf_a = zeros(1,3); buf_g = zeros(1,3); buf_m = zeros(1,3);
count = 0;
while isvalid(fig)
    line = readline(s);
    rest = extractAfter(line, ": ");
    if ismissing(rest), continue; end
    val = sscanf(rest, "%f");
    if isempty(val), continue; end

    if     contains(line, "Acce_x"), buf_a(1) = val;
    elseif contains(line, "Acce_y"), buf_a(2) = val;
    elseif contains(line, "Acce_z"), buf_a(3) = val;
    elseif contains(line, "Gyro_x"), buf_g(1) = val;
    elseif contains(line, "Gyro_y"), buf_g(2) = val;
    elseif contains(line, "Gyro_z"), buf_g(3) = val;
    elseif contains(line, "mag/x"),  buf_m(1) = val;
    elseif contains(line, "mag/y"),  buf_m(2) = val;
    elseif contains(line, "mag/z")
        buf_m(3) = val;

        % Sample komplett -> in die Ringpuffer schieben
        accel = [accel(2:end,:); buf_a];
        gyro  = [gyro(2:end,:);  buf_g];
        mag   = [mag(2:end,:);   buf_m];

        % Plots nur mit ~10 Hz aktualisieren — 100 Hz drawnow wuerde den
        % Serial-Puffer ueberlaufen lassen und die Anzeige verzoegern.
        count = count + 1;
        if mod(count, 10) == 0
            for a = 1:3
                set(h(1,a), 'YData', accel(:,a));
                set(h(2,a), 'YData', gyro(:,a));
                set(h(3,a), 'YData', mag(:,a));
            end
            drawnow limitrate
        end
    end
end
delete(s);
