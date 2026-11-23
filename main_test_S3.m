% main_test_S3.m — Live-Visualisierung der Dollen-IMU (XIAO ESP32-S3 + ICM-20948)
%
% Datenquelle: PlatformIO-Env `xiao_s3_angle_diag` (test/angle_diagnostics_s3.cpp).
% Die Firmware sendet ueber natives USB-CDC eine CSV-Zeile je Sample (50 Hz,
% 460800 Baud) mit 41 Spalten. Das frueher hier erwartete Teleplot-Format
% (">imu/Acce_x: ...") kam aus test/test_imu_stream_s3.cpp — diese Datei und ihr
% Env `xiao_s3_matlab` existieren im Firmware-Repo nicht mehr.
%
% Flashen:
%   ~/.platformio/penv/bin/pio run -e xiao_s3_angle_diag -t upload
% Nach dem Reset das Geraet ~6 s ruhig liegen lassen (Gyro-Bias-Kalibrierung).
% Erst wenn ANGLE_CAL,SUCCESS kommt, ist calibration_state == 3 (Valid). Ohne
% das driftet der Winkel mit der vollen Nullpunktrate des Gyros.
%
% Angezeigt wird:
%   links       — poseplot der MATLAB-EKF-Orientierung (ekf_update_neu)
%   rechts oben — Winkelverlauf des MATLAB-EKF
%   rechts unten— Winkelverlauf des Firmware-EKF auf denselben Rohdaten
% Auseinanderlaufende Kurven zeigen, wo Sim und Firmware nicht dasselbe rechnen.

close all
clear
clc
delete(serialportfind)
clear ekf_update
clear ekf_update_neu   % setzt gelernte Referenzen + Anfangs-q + P zurueck

% ── Konfiguration ────────────────────────────────────────────────────────────
PORT       = "/dev/cu.usbmodem1101"; % ls /dev/cu.* — Board mit Diagnose-Firmware
BAUD       = 460800;                 % muss zu monitor_speed in platformio.ini passen
WINDOW_S   = 30;                     % Zeitfenster der Verlaufsplots [s]
PLOT_EVERY = 5;                      % nur jedes n-te Sample zeichnen (Tempo)

% Body -> EKF-Frame. Die Firmware (AngleReader::readSample) macht die zyklische
% Vertauschung EKF = (body_y, body_z, body_x), Determinante +1 — also eine echte
% Drehung, keine Spiegelung. Accel, Gyro und Mag werden GLEICH permutiert; sonst
% liegen Schwerkraft und Magnetfeld in verschiedenen Koordinatensystemen und der
% EKF fusioniert zwei unvereinbare Referenzen gegeneinander.
%
% Faustregel beim Aendern: Mapping als 3x3-Matrix hinschreiben, det() muss +1
% sein. EINE Achse negieren ist det = -1 (Spiegelung); ZWEI Achsen negieren ist
% det = +1 (180-Grad-Drehung). Laeuft der Winkel danach in die falsche Richtung,
% das ERGEBNIS negieren — nicht am Frame drehen.
AXIS_MAP = [2 3 1];   % ekf(1:3) = body(AXIS_MAP)

% Einbaulage: zeigt die Sensor-X-Achse nach oben, landet sie mit obigem Mapping
% auf EKF-Z. Die Drehung der Dolle erscheint dann als YAW, nicht als Roll. Soll
% sie als Roll herauskommen, muss die Vertauschung in der FIRMWARE geaendert
% werden (AngleReader.cpp); hier laesst sie sich ueber AXIS_MAP nur vergleichend
% durchspielen.

% ── CSV-Spalten aus angle_diagnostics_s3.cpp::printHeader() ──────────────────
COLS = ["timestamp_us","sequence_number","dt_s", ...
        "gyro_raw_x_dps","gyro_raw_y_dps","gyro_raw_z_dps", ...
        "gyro_bias_x_dps","gyro_bias_y_dps","gyro_bias_z_dps", ...
        "gyro_corrected_x_dps","gyro_corrected_y_dps","gyro_corrected_z_dps", ...
        "accel_x","accel_y","accel_z","accel_norm", ...
        "mag_raw_x","mag_raw_y","mag_raw_z", ...
        "mag_calibrated_x","mag_calibrated_y","mag_calibrated_z","mag_norm", ...
        "quaternion_w","quaternion_x","quaternion_y","quaternion_z", ...
        "yaw_deg","pitch_deg","roll_deg","output_angle_deg", ...
        "accel_valid","mag_valid","mag_fresh","stationary", ...
        "queue_depth","dropped_events","calibration_state","temperature_c", ...
        "invalid_dt_count","gyro_clip_count"];
N_COLS = numel(COLS);
C = struct();
for k = 1:N_COLS
    C.(COLS(k)) = k;   % C.roll_deg liefert den Spaltenindex
end

% ── Serielle Verbindung ──────────────────────────────────────────────────────
s = serialport(PORT, BAUD);
configureTerminator(s, "LF");
flush(s);
cleanup = onCleanup(@() delete(serialportfind));   % Port bei Ctrl+C freigeben

% ── Figure aufbauen ──────────────────────────────────────────────────────────
fig = figure('Name', 'Dollen-IMU live', 'Position', [80 80 1400 720]);

ax_pose = subplot(1, 2, 1);
has_poseplot = exist('poseplot', 'file') > 0;
if has_poseplot
    viewer = poseplot(quaternion(1, 0, 0, 0));
    view(ax_pose, 0, 90);          % Draufsicht: Azimut 0 Grad, Elevation 90 Grad
    title(ax_pose, 'MATLAB-EKF Orientierung');
else
    viewer = [];
    text(0.5, 0.5, {'poseplot nicht verfuegbar', ...
                    '(Navigation / Sensor Fusion Toolbox fehlt)'}, ...
         'HorizontalAlignment', 'center', 'Parent', ax_pose);
    axis(ax_pose, 'off');
end

ax_ml = subplot(2, 2, 2);
hold(ax_ml, 'on'); grid(ax_ml, 'on');
h_ml_roll  = plot(ax_ml, NaN, NaN, 'LineWidth', 1.2);
h_ml_pitch = plot(ax_ml, NaN, NaN, 'LineWidth', 1.0);
h_ml_yaw   = plot(ax_ml, NaN, NaN, 'LineWidth', 1.0);
ylabel(ax_ml, 'Winkel [Grad]');
legend(ax_ml, {'Roll', 'Pitch', 'Yaw'}, 'Location', 'northwest');
title(ax_ml, 'MATLAB-EKF (ekf\_update\_neu)');

ax_fw = subplot(2, 2, 4);
hold(ax_fw, 'on'); grid(ax_fw, 'on');
h_fw_roll  = plot(ax_fw, NaN, NaN, 'LineWidth', 1.2);
h_fw_pitch = plot(ax_fw, NaN, NaN, 'LineWidth', 1.0);
h_fw_yaw   = plot(ax_fw, NaN, NaN, 'LineWidth', 1.0);
xlabel(ax_fw, 'Zeit [s]'); ylabel(ax_fw, 'Winkel [Grad]');
legend(ax_fw, {'Roll', 'Pitch', 'Yaw'}, 'Location', 'northwest');
title(ax_fw, 'Firmware-EKF (orientation\_ekf)');

% ── Ringpuffer fuer die Verlaufsplots ────────────────────────────────────────
cap    = 4000;
buf_t  = nan(1, cap);
buf_ml = nan(3, cap);   % Roll, Pitch, Yaw aus MATLAB
buf_fw = nan(3, cap);   % Roll, Pitch, Yaw aus der Firmware
n = 0;

q = [1; 0; 0; 0];
t0 = NaN;
draw_counter = 0;
state_reported = false;

fprintf('Lese %s @ %d Baud. Abbruch mit Ctrl+C.\n', PORT, BAUD);

while isvalid(fig)
    line = readline(s);
    if ismissing(line) || strlength(line) == 0
        continue
    end
    % Firmware-Statuszeilen (#..., ANGLE_CAL,..., ANGLE_TIMING,...) und die
    % einmalige Kopfzeile sind keine Messdaten.
    if startsWith(line, "#") || startsWith(line, "ANGLE_") || startsWith(line, "timestamp_us")
        fprintf('%s\n', line);
        continue
    end

    v = sscanf(line, '%f,');
    if numel(v) ~= N_COLS
        continue   % unvollstaendige Zeile, z. B. beim Verbinden mitten im Stream
    end

    if ~state_reported
        state_reported = true;
        if v(C.calibration_state) ~= 3
            warning(['calibration_state = %d (3 = Valid). Der Gyro-Bias ist nicht ' ...
                     'gueltig bestimmt — der Winkel driftet dann mit der vollen ' ...
                     'Nullpunktrate. Board zuruecksetzen und waehrend der ' ...
                     'Kalibrierung nicht bewegen.'], v(C.calibration_state));
        end
    end

    % Rohwerte im BODY-Frame. accel in mg, mag in uT, gyro in Grad/s. Der EKF
    % normiert Accel und Mag, deren Einheit ist also egal; der Gyro geht als
    % Rate ein und muss in rad/s vorliegen.
    accel_body = [v(C.accel_x); v(C.accel_y); v(C.accel_z)];
    % mag_calibrated_* ist bereits hard-/soft-iron-korrigiert (MAG_A/MAG_B in
    % AngleReaderConfig.h, erzeugt von scripts/calibrate_magnetometer.py).
    % Deshalb hier KEINE zweite Kalibrierung mit lokalen A/b-Werten anwenden.
    mag_body = [v(C.mag_calibrated_x); v(C.mag_calibrated_y); v(C.mag_calibrated_z)];
    gyro_body = deg2rad([v(C.gyro_raw_x_dps) - v(C.gyro_bias_x_dps); ...
                         v(C.gyro_raw_y_dps) - v(C.gyro_bias_y_dps); ...
                         v(C.gyro_raw_z_dps) - v(C.gyro_bias_z_dps)]);

    % Body -> EKF-Frame, fuer alle drei Sensoren identisch.
    accel = accel_body(AXIS_MAP);
    mag   = mag_body(AXIS_MAP);
    gyro  = gyro_body(AXIS_MAP);

    dt = v(C.dt_s);
    if ~isfinite(dt) || dt <= 0
        continue
    end

    q = ekf_update_neu(q, gyro, accel, mag, dt);

    % Zeitbasis aus der Geraetezeit, nicht aus der Hostzeit.
    if isnan(t0)
        t0 = v(C.timestamp_us);
    end
    t = (v(C.timestamp_us) - t0) * 1e-6;

    q_ml   = quaternion(q(1), q(2), q(3), q(4));
    eul_ml = eulerd(q_ml, 'ZYX', 'frame');   % [Yaw, Pitch, Roll]

    n   = n + 1;
    idx = mod(n - 1, cap) + 1;
    buf_t(idx)     = t;
    buf_ml(:, idx) = [eul_ml(3); eul_ml(2); eul_ml(1)];
    buf_fw(:, idx) = [v(C.roll_deg); v(C.pitch_deg); v(C.yaw_deg)];

    draw_counter = draw_counter + 1;
    if draw_counter < PLOT_EVERY
        continue
    end
    draw_counter = 0;

    if has_poseplot
        % 90 Grad um X, damit die Draufsicht die Dolle in Einbaulage zeigt.
        q_view = quaternion(cosd(45), sind(45), 0, 0);
        viewer.Orientation = q_view * q_ml;
    end

    % Chronologisch sortiertes Fenster aus dem Ringpuffer holen.
    if n < cap
        sel = 1:n;
    else
        sel = [idx+1:cap, 1:idx];
    end
    tt   = buf_t(sel);
    keep = tt >= (t - WINDOW_S);
    tt   = tt(keep);
    ml   = buf_ml(:, sel); ml = ml(:, keep);
    fw   = buf_fw(:, sel); fw = fw(:, keep);

    set(h_ml_roll,  'XData', tt, 'YData', ml(1, :));
    set(h_ml_pitch, 'XData', tt, 'YData', ml(2, :));
    set(h_ml_yaw,   'XData', tt, 'YData', ml(3, :));
    set(h_fw_roll,  'XData', tt, 'YData', fw(1, :));
    set(h_fw_pitch, 'XData', tt, 'YData', fw(2, :));
    set(h_fw_yaw,   'XData', tt, 'YData', fw(3, :));

    xlim(ax_ml, [max(0, t - WINDOW_S), max(WINDOW_S, t)]);
    xlim(ax_fw, [max(0, t - WINDOW_S), max(WINDOW_S, t)]);

    title(ax_fw, sprintf(['Firmware-EKF — mag\\_valid=%d  accel\\_valid=%d  ' ...
                          'stationary=%d  cal\\_state=%d'], ...
                         v(C.mag_valid), v(C.accel_valid), ...
                         v(C.stationary), v(C.calibration_state)));

    drawnow limitrate
end
