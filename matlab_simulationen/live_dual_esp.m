% LIVE DUAL ESP – GPS Track + IMU Heading in 2D
% GPS-ESP:  Position + COG auf Karte
% IMU-ESP:  Magnetometer-Yaw als Richtungspfeil an aktueller Position
% ─────────────────────────────────────────────────────────────────────────────
close all; clear; clc;
delete(serialportfind);

%% Ports
PORT_GPS = "/dev/cu.usbserial-110";
PORT_IMU = "/dev/cu.usbserial-10";

s_gps = serialport(PORT_GPS, 115200);
s_imu = serialport(PORT_IMU, 115200);
configureTerminator(s_gps, "LF");
configureTerminator(s_imu, "LF");
flush(s_gps); flush(s_imu);

%% Magnetometer-Kalibrierung (aus test6050.m)
A = [1.0962    0        0;
     0         1.4670   0;
     0         0        0.6219];
b = [-115.4853   6.6805   -14.4159];

%% FUSE (ahrsfilter) – NED-Mapping wie in README
FUSE = ahrsfilter('SampleRate', 100);

accel = zeros(1,3); gyro = zeros(1,3); mag = zeros(1,3);
imu_yaw = 0;

lat = 0; lon = 0; fix = 0; cog = 0; speed = 0; sats = 0;
track_lat = []; track_lon = [];

%% Figure – Karte links, Kompass rechts
fig = figure('Name','GPS + IMU Live','Position',[100 100 1300 700]);

% ── Linke Seite: GPS Track ──────────────────────────────────────────────────
ax_map = subplot(1,2,1);
hold(ax_map,'on'); grid(ax_map,'on');
xlabel(ax_map,'Lon'); ylabel(ax_map,'Lat');
title(ax_map,'GPS Track');
h_track = plot(ax_map, NaN, NaN, 'c-', 'LineWidth',1.5, 'DisplayName','GPS Track');
h_pos   = plot(ax_map, NaN, NaN, 'wo', 'MarkerSize',10,  'MarkerFaceColor','w');

% ── Rechte Seite: Kompass ───────────────────────────────────────────────────
ax_cmp = subplot(1,2,2);
hold(ax_cmp,'on'); axis(ax_cmp,'equal'); axis(ax_cmp,'off');
title(ax_cmp,'Heading-Vergleich');
xlim(ax_cmp,[-1.4 1.4]); ylim(ax_cmp,[-1.4 1.4]);

% Kompassring + Himmelsrichtungen
t = linspace(0,2*pi,200);
plot(ax_cmp, cos(t), sin(t), 'w-', 'LineWidth', 1);
for deg = 0:30:330
    x1 = 0.88*sind(deg); y1 = 0.88*cosd(deg);
    x2 =      sind(deg); y2 =      cosd(deg);
    plot(ax_cmp,[x1 x2],[y1 y2],'w-','LineWidth',1);
end
for deg = 0:10:350
    plot(ax_cmp, 0.93*sind(deg), 0.93*cosd(deg), 'w.', 'MarkerSize',3);
end
text(ax_cmp,  0,    1.2,  'N', 'Color','w','HorizontalAlignment','center','FontSize',12,'FontWeight','bold');
text(ax_cmp,  1.2,  0,    'O', 'Color','w','HorizontalAlignment','center','FontSize',12,'FontWeight','bold');
text(ax_cmp,  0,   -1.2,  'S', 'Color','w','HorizontalAlignment','center','FontSize',12,'FontWeight','bold');
text(ax_cmp, -1.2,  0,    'W', 'Color','w','HorizontalAlignment','center','FontSize',12,'FontWeight','bold');

% Nadeln
h_cog_n = quiver(ax_cmp,0,0, 0,0.8, 'y','LineWidth',4,'MaxHeadSize',0.15,'AutoScale','off','DisplayName','GPS COG');
h_mag_n = quiver(ax_cmp,0,0, 0,0.8, 'm','LineWidth',4,'MaxHeadSize',0.15,'AutoScale','off','DisplayName','IMU Yaw');
legend(ax_cmp,'Location','southoutside','Orientation','horizontal');

% Delta-Text mittig
t_delta = text(ax_cmp, 0, 0, 'Δ -°', 'Color','w','HorizontalAlignment','center', ...
    'FontSize',16,'FontWeight','bold');
t_side  = text(ax_cmp, 0,-0.25,'?', 'Color','c','HorizontalAlignment','center', ...
    'FontSize',14,'FontWeight','bold');

t_info = annotation('textbox',[0.01 0.01 0.98 0.04],'String','Warte...', ...
    'EdgeColor','none','FontSize',10,'HorizontalAlignment','center');

% Pfeillänge wird dynamisch 20% der Achsenbreite gesetzt (siehe Loop)

fprintf("Live läuft. Ctrl+C zum Beenden.\n");

%% Loop
while true

    % ── IMU: Buffer leeren ─────────────────────────────────────────────────
    while s_imu.NumBytesAvailable > 0
        line = readline(s_imu);
        if contains(line,"Acce_x"), accel(3) = sscanf(extractAfter(line,": "),"%f"); end
        if contains(line,"Acce_y"), accel(2) = sscanf(extractAfter(line,": "),"%f"); end
        if contains(line,"Acce_z"), accel(1) = sscanf(extractAfter(line,": "),"%f"); end
        if contains(line,"Gyro_x"), gyro(3)  = sscanf(extractAfter(line,": "),"%f"); end
        if contains(line,"Gyro_y"), gyro(2)  = sscanf(extractAfter(line,": "),"%f"); end
        if contains(line,"Gyro_z"), gyro(1)  = sscanf(extractAfter(line,": "),"%f"); end
        if contains(line,"mag/x"),  mag(1)   = sscanf(extractAfter(line,": "),"%f"); end
        if contains(line,"mag/y"),  mag(2)   = sscanf(extractAfter(line,": "),"%f"); end
        if contains(line,"mag/z")
            mag(3)   = sscanf(extractAfter(line,": "),"%f");
            mag_cal  = (mag - b) * A;
            % NED-Alignment: Gravitation auf Accel_Y → [Y, X, Z]
            accel_ned = [accel(2), accel(1), accel(3)];
            gyro_ned  = [gyro(2),  gyro(1),  gyro(3)];
            q_ml    = FUSE(accel_ned, gyro_ned, mag_cal * 1e-6);
            eul     = eulerd(q_ml,'ZYX','frame');
            imu_yaw = eul(1);
        end
    end

    % ── GPS: Buffer leeren ──────────────────────────────────────────────────
    while s_gps.NumBytesAvailable > 0
        line = readline(s_gps);
        if contains(line,">gps/cog_deg"),   cog   = sscanf(extractAfter(line,": "),"%f"); end
        if contains(line,">gps/speed_kmh"), speed = sscanf(extractAfter(line,": "),"%f"); end
        if contains(line,">gps/sats"),      sats  = sscanf(extractAfter(line,": "),"%f"); end
        if contains(line,"i2c") && contains(line,"lat:")
            fix = double(contains(line,"fix: yes"));
            tok = regexp(line,'lat:\s*([\d.\-]+).*lon:\s*([\d.\-]+)','tokens');
            if ~isempty(tok)
                lat = str2double(tok{1}{1});
                lon = str2double(tok{1}{2});
                if fix; track_lat(end+1)=lat; track_lon(end+1)=lon; end
            end
        end
    end

    % ── Karte updaten ───────────────────────────────────────────────────────
    if fix && ~isempty(track_lat)
        set(h_track,'XData',track_lon,'YData',track_lat);
        set(h_pos,  'XData',lon,'YData',lat);
        if length(track_lat) > 1
            pad = 0.0004;
            xlim(ax_map,[min(track_lon)-pad, max(track_lon)+pad]);
            ylim(ax_map,[min(track_lat)-pad, max(track_lat)+pad]);
        end
    end

    % ── Kompass updaten (immer, unabhängig von GPS-Fix) ─────────────────────
    set(h_cog_n,'UData', 0.8*sind(cog),     'VData', 0.8*cosd(cog));
    set(h_mag_n,'UData', 0.8*sind(imu_yaw), 'VData', 0.8*cosd(imu_yaw));

    delta = wrapTo180(imu_yaw - cog);
    if     delta >  20, seite = '→ RECHTS';
    elseif delta < -20, seite = '← LINKS';
    else,               seite = '↑ VORNE';
    end
    set(t_delta,'String', sprintf('Δ %.1f°', delta));
    set(t_side, 'String', seite);

    set(t_info,'String', sprintf( ...
        'COG: %.1f°   IMU Yaw: %.1f°   Δ: %.1f°   |   %.1f km/h   %d Sats   Fix: %d', ...
        cog, imu_yaw, delta, speed, sats, fix));

    drawnow;
    if s_gps.NumBytesAvailable==0 && s_imu.NumBytesAvailable==0
        pause(0.01);
    end
end
