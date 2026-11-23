% xiao_s3_debug.m — Live-Anzeige der Dollen-/Hub-Daten ueber Bluetooth Low Energy
%
% Die Sensorfusion laeuft KOMPLETT auf dem ESP32 (AngleReader + orientation_ekf).
% MATLAB rechnet nichts nach, sondern dekodiert und zeigt nur an.
%
% ── Wer sendet hier eigentlich BLE? ──────────────────────────────────────────
% NICHT die Dolle. Das Env `xiao_s3` (src/mainDolle.cpp -> EspNow_sender_App)
% funkt per ESP-NOW, nicht per Bluetooth. BLE macht der HUB, Env
% `prod_BLE_sender` (src/main_BLE_sender.cpp) — ebenfalls ein XIAO ESP32-S3,
% daher die Namensverwechslung. Die Kette lautet:
%
%   Dolle (ESP-NOW)  ->  Aggregator (ESP-NOW rx, UART fwd)  ->  Hub (BLE out)
%
% MATLAB verbindet sich also mit dem Hub, der als "RowingBoat" wirbt.
%
% ── Wire-Format (lib/AppTypes/AppTypes.h, MeasurementPack, 36 Byte) ──────────
%   uint32  espIdAndSeqenceNum        4 B   oben 4 Bit ID | unten 28 Bit Sequenz
%   uint16  force_values[8]          16 B
%   uint16  angle_values[8]          16 B
% Alles little-endian (ESP32 wie auch der Mac), PACKET_VALUES wird unten aus der
% Paketlaenge abgeleitet statt hart gesetzt — der Wert ist am 24.07.2026 von 32
% auf 8 gewechselt und wuerde sonst beim naechsten Wechsel still brechen.
%
%   ID 0      -> Telemetrie: force-Region traegt GpsData, angle-Region ImuData
%   ID 1..15  -> Dolle #ID:  angle_values sind quantisierte Winkel
%                            deg = code / 65535 * 360 - 180
%
% ── Voraussetzungen ──────────────────────────────────────────────────────────
% Aus der MATLAB-OBERFLAECHE starten, nicht aus einem Headless-Aufruf: macOS
% fragt die Bluetooth-Berechtigung ueber einen Dialog ab. Ohne GUI bricht
% blelist/ble mit einem fatalen Fehler ab. Falls kein Dialog erscheint:
% Systemeinstellungen -> Datenschutz & Sicherheit -> Bluetooth -> MATLAB.

close all
clear
clc

% ── Konfiguration ────────────────────────────────────────────────────────────
DEVICE       = "RowingBoat";  % BleSender.h: DEVICE_NAME
SERVICE_UUID = "a1b2c3d4-0001-4a2b-9c3d-1234567890ab";
PACKET_UUID  = "a1b2c3d4-0002-4a2b-9c3d-1234567890ab";
WINDOW_S     = 30;            % Zeitfenster der Verlaufsplots [s]

ANGLE_MIN_DEG = -180;         % DataSender.h
ANGLE_MAX_DEG =  180;

% ── Verbinden ────────────────────────────────────────────────────────────────
fprintf('Suche "%s" ...\n', DEVICE);
b = ble(DEVICE);
fprintf('Verbunden. Charakteristiken werden abonniert.\n');
c = characteristic(b, SERVICE_UUID, PACKET_UUID);

% ── Figure ───────────────────────────────────────────────────────────────────
fig = figure('Name', 'RowingBoat live (BLE)', 'Position', [80 80 1400 720]);

ax_pose = subplot(1, 2, 1);
viewer = poseplot(quaternion(1, 0, 0, 0));
% Schraege Ansicht statt Draufsicht: hier wird nur Roll gesetzt (Pitch/Yaw
% bleiben 0). Von oben laege die Roll-Achse fast in Blickrichtung und die
% Rotation waere praktisch unsichtbar.
view(ax_pose, 135, 20);
title(ax_pose, 'Dollenwinkel (Roll)');

ax_oar = subplot(2, 2, 2);
h_oar = plot(ax_oar, NaN, NaN, 'LineWidth', 1.2);
grid(ax_oar, 'on');
ylabel(ax_oar, 'Winkel [Grad]');
title(ax_oar, 'Dolle (ID 1..15) — noch keine Pakete');

ax_hub = subplot(2, 2, 4);
hold(ax_hub, 'on'); grid(ax_hub, 'on');
h_hub_roll  = plot(ax_hub, NaN, NaN, 'LineWidth', 1.2);
h_hub_pitch = plot(ax_hub, NaN, NaN, 'LineWidth', 1.0);
h_hub_yaw   = plot(ax_hub, NaN, NaN, 'LineWidth', 1.0);
xlabel(ax_hub, 'Zeit [s]'); ylabel(ax_hub, 'Winkel [Grad]');
legend(ax_hub, {'Roll', 'Pitch', 'Yaw'}, 'Location', 'northwest');
title(ax_hub, 'Hub-IMU (ID 0) — noch keine Pakete');

% ── Zustand im Figure ablegen, damit der BLE-Callback ihn erreicht ───────────
cap = 4000;
fig.UserData = struct( ...
    'cap',        cap, ...
    't_oar',      nan(1, cap), 'a_oar', nan(1, cap), 'n_oar', 0, ...
    't_hub',      nan(1, cap), 'a_hub', nan(3, cap), 'n_hub', 0, ...
    'timer',      tic, ...
    'oar_id',     NaN, ...
    'pkt_total',  0, ...
    'pkt_bad',    0, ...
    'angle_span', [ANGLE_MIN_DEG ANGLE_MAX_DEG]);

c.DataAvailableFcn = @(src, ~) onPacket(src, fig);
subscribe(c, "notification");
cleanup = onCleanup(@() cleanupBle(c));

fprintf('Empfange. Fenster schliessen oder Ctrl+C zum Beenden.\n');

% ── Zeichenschleife (Dekodierung passiert im Callback) ───────────────────────
while isvalid(fig)
    st = fig.UserData;
    t_now = toc(st.timer);

    if st.n_oar > 0
        [tt, aa] = windowOf(st.t_oar, st.a_oar, st.n_oar, st.cap, t_now, WINDOW_S);
        set(h_oar, 'XData', tt, 'YData', aa(1, :));
        xlim(ax_oar, [max(0, t_now - WINDOW_S), max(WINDOW_S, t_now)]);
        title(ax_oar, sprintf('Dolle ID %d — aktuell %.2f Grad  (%d Pakete)', ...
                              st.oar_id, aa(1, end), st.n_oar));
        viewer.Orientation = quaternion([0 0 aa(1, end)], 'eulerd', 'ZYX', 'frame');
    end

    if st.n_hub > 0
        [tt, aa] = windowOf(st.t_hub, st.a_hub, st.n_hub, st.cap, t_now, WINDOW_S);
        set(h_hub_roll,  'XData', tt, 'YData', aa(1, :));
        set(h_hub_pitch, 'XData', tt, 'YData', aa(2, :));
        set(h_hub_yaw,   'XData', tt, 'YData', aa(3, :));
        xlim(ax_hub, [max(0, t_now - WINDOW_S), max(WINDOW_S, t_now)]);
        title(ax_hub, sprintf('Hub-IMU (ID 0) — %d Pakete, %d verworfen', ...
                              st.n_hub, st.pkt_bad));
    end

    drawnow limitrate
    pause(0.03);
end

% ── Lokale Funktionen ────────────────────────────────────────────────────────

function onPacket(src, fig)
% Dekodiert alle wartenden Notifications und legt sie in fig.UserData ab.
if ~isvalid(fig)
    return
end
st = fig.UserData;

while true
    % read wirft eine Warnung/Fehler, wenn die Queue leer ist — je nach Release
    % unterschiedlich. Beides bedeutet hier schlicht "nichts mehr da".
    try
        raw = read(src, 'oldest');
    catch
        break
    end
    if isempty(raw)
        break
    end
    raw = uint8(raw(:)');
    st.pkt_total = st.pkt_total + 1;

    % 4 Byte Kopf + je 2 Byte fuer force und angle -> Anzahl Werte ableiten.
    if numel(raw) < 8 || mod(numel(raw) - 4, 4) ~= 0
        st.pkt_bad = st.pkt_bad + 1;
        continue
    end
    nvals = (numel(raw) - 4) / 4;

    idseq = typecast(raw(1:4), 'uint32');
    id    = double(bitshift(idseq, -28));              % obere 4 Bit
    t     = toc(st.timer);

    if id == 0
        % Telemetriepaket: ImuData (16 B) liegt in der angle-Region.
        off = 4 + 2 * nvals;
        if numel(raw) < off + 16
            st.pkt_bad = st.pkt_bad + 1;
            continue
        end
        rpy = double(typecast(raw(off+7:off+12), 'int16')) / 100;  % roll/pitch/yaw [cdeg]
        st.n_hub = st.n_hub + 1;
        idx = mod(st.n_hub - 1, st.cap) + 1;
        st.t_hub(idx)    = t;
        st.a_hub(:, idx) = rpy(:);
    else
        % Dollenpaket: angle_values sind quantisierte Winkel.
        codes = double(typecast(raw(5 + 2*nvals:end), 'uint16'));
        span  = st.angle_span;
        degs  = codes / 65535 * (span(2) - span(1)) + span(1);
        st.oar_id = id;
        for k = 1:numel(degs)
            st.n_oar = st.n_oar + 1;
            idx = mod(st.n_oar - 1, st.cap) + 1;
            st.t_oar(idx) = t;
            st.a_oar(idx) = degs(k);
        end
    end
end

fig.UserData = st;
end

function [tt, aa] = windowOf(tbuf, abuf, n, cap, t_now, window_s)
% Holt das chronologisch sortierte Zeitfenster aus einem Ringpuffer.
idx = mod(n - 1, cap) + 1;
if n < cap
    sel = 1:n;
else
    sel = [idx+1:cap, 1:idx];
end
tt = tbuf(sel);
aa = abuf(:, sel);
keep = tt >= (t_now - window_s);
tt = tt(keep);
aa = aa(:, keep);
end

function cleanupBle(c)
% Notifications abbestellen. Die BLE-Verbindung selbst wird geschlossen, sobald
% das ble-Objekt aus dem Workspace verschwindet (clear b).
try
    unsubscribe(c);
catch
end
end
