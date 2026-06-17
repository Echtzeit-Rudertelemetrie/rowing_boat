% ─────────────────────────────────────────────────────────────────────────────
% Hall-Sensor Live-Simulation: AS5600 (On-Axis) + ESP32-Teleplot-Stream
%
% Geometrie passend zum AS5600: der Chip sitzt zentriert AUF der Drehachse,
% direkt unter dem diametral magnetisierten Ringmagneten, getrennt durch einen
% kleinen Luftspalt. Der Magnet rotiert um die vertikale Achse; das am Sensor
% ankommende In-Plane-Feld (Bx,By) dreht sich 1:1 mit (um 180° phasenverschoben)
% und wird chip-intern zu einem Winkel ausgewertet. Genau dieser fertige Winkel
% kommt per Teleplot vom ESP32 (siehe test_gyro_hall.cpp):
%
%   ">as5600/angle_raw: <float>\n"
%
% ─────────────────────────────────────────────────────────────────────────────

clear; close all; clc;

%% ── Konfiguration ────────────────────────────────────────────────────────────
fs           = 100;             % Abtastrate [Hz] (nur für Buffer-Dimensionierung)
BUFFER_LEN   = 500;             % Anzahl Samples im Ringpuffer
air_gap      = 0.0020;          % Luftspalt Magnet <-> AS5600 [m] (Datenblatt: 0.5–3 mm)
magnet_r     = 0.0030;          % Magnetradius [m] (nur für die Darstellung)
FORMAT       = 'C';             % Teleplot-Zeile ">key: <float>"
TELEPLOT_KEY = 'as5600/angle_raw';

%% ── Serial-Port öffnen ───────────────────────────────────────────────────────
try
    s = serialport("/dev/cu.usbserial-10", 115200);
catch
    s = serialport("/dev/cu.usbserial-110", 115200);
end
configureTerminator(s, "LF");
flush(s);
fprintf("Serial offen: %s\n", s.Port);

%% ── On-Axis-Geometrie ────────────────────────────────────────────────────────
% Magnet liegt in der xy-Ebene bei z=0 und dreht sich um die z-Achse;
% der AS5600 sitzt zentriert auf eben dieser Achse, einen Luftspalt darunter.
sensor_pos = [0; 0; -air_gap];

%% ── Ringpuffer ───────────────────────────────────────────────────────────────
buf_angle = nan(1, BUFFER_LEN);
buf_idx   = 1;
total     = 0;

%% ── Figure aufbauen ──────────────────────────────────────────────────────────
fig = figure('Name','AS5600 Live', 'NumberTitle','off', ...
             'Position',[80 60 1150 460], ...
             'CloseRequestFcn', @(~,~) cleanup(s));

% ── Axes 1: Draufsicht (Blick entlang der Drehachse) ─────────────────────────
ax1 = subplot(1,3,1);
hold(ax1,'on'); axis(ax1,'equal'); box(ax1,'on'); grid(ax1,'off');
lim1 = magnet_r*1.7;
xlim(ax1,[-lim1 lim1]); ylim(ax1,[-lim1 lim1]);
xlabel(ax1,'x [m]'); ylabel(ax1,'y [m]');
title(ax1,'Draufsicht: Magnetstellung (Blick entlang Drehachse)');

[ph_N, ph_S, th_N, th_S, qsens, hAnnoTop] = init_top_view(ax1, 0, magnet_r);

% ── Axes 2: Seitenansicht (vertikale Anordnung / Luftspalt) ──────────────────
ax2 = subplot(1,3,2);
hold(ax2,'on'); axis(ax2,'equal'); box(ax2,'on'); grid(ax2,'off');
lim2x = magnet_r*2.4;
zTop  = magnet_r*1.0;
zBot  = -air_gap - magnet_r*1.1;
xlim(ax2,[-lim2x lim2x]); ylim(ax2,[zBot, zTop]);
xlabel(ax2,'x [m]'); ylabel(ax2,'z, vertikal [m]');
title(ax2,'Seitenansicht: Magnet über Sensor, Luftspalt');

[hPoleSide] = init_side_view(ax2, 0, magnet_r, air_gap);

% ── Axes 3: Winkel-Zeitreihe ──────────────────────────────────────────────────
ax3 = subplot(1,3,3);
hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
t_axis = (-(BUFFER_LEN-1):0) / fs;
hLine  = plot(ax3, t_axis, buf_angle, 'b-', 'LineWidth',1.2);
xlim(ax3, [t_axis(1) 0]);
ylim(ax3, [0 360]);
yticks(ax3, 0:90:360);
xlabel(ax3,'Zeit [s]'); ylabel(ax3,'Winkel [°]');
title(ax3,'AS5600: gemeldeter Winkel');
hAnno = annotation('textbox', [0.70 0.88 0.18 0.08], ...
                   'String','—', 'FontSize',13, 'FontWeight','bold', ...
                   'EdgeColor','none', 'Color',[0.1 0.1 0.1], ...
                   'HorizontalAlignment','center');

%% ── Live-Loop ────────────────────────────────────────────────────────────────
fprintf("Live-Loop gestartet. Fenster schließen zum Beenden.\n");

while ishandle(fig)
    % ── Zeile einlesen ────────────────────────────────────────────────────────
    try
        line = readline(s);
    catch
        break;
    end
    line = strtrim(line);
    if isempty(line); continue; end

    % ── Parsen ────────────────────────────────────────────────────────────────
    angle_deg = parse_line(line, FORMAT, TELEPLOT_KEY);
    if isnan(angle_deg); continue; end

    % Winkel auf [0, 360) normieren
    angle_deg = mod(angle_deg, 360);

    % ── Ringpuffer schreiben ──────────────────────────────────────────────────
    buf_angle(buf_idx) = angle_deg;
    buf_idx = mod(buf_idx, BUFFER_LEN) + 1;
    total   = total + 1;

    % Geordneter Ringpuffer für Plot
    if total >= BUFFER_LEN
        idx_ord = [buf_idx:BUFFER_LEN, 1:buf_idx-1];
    else
        idx_ord = 1:min(total, BUFFER_LEN);
    end
    angles_ord = buf_angle(idx_ord);

    % ── Erwartetes Feld am On-Axis-Sensor für den gemeldeten Winkel ───────────
    % (diametraler Dipol m=[cosθ,sinθ,0]; auf der Achse steht B antiparallel zu
    %  m und dreht sich exakt mit θ mit — das ist die Grundlage, auf der der
    %  AS5600 seinen Winkel überhaupt erst bestimmen kann.)
    m_cur = [cosd(angle_deg); sind(angle_deg); 0];
    B_cur = dipole_field(m_cur, sensor_pos);

    % ── Axes 1: Draufsicht aktualisieren ──────────────────────────────────────
    update_top_view(ph_N, ph_S, th_N, th_S, qsens, hAnnoTop, angle_deg, magnet_r, B_cur);

    % ── Axes 2: Seitenansicht aktualisieren ───────────────────────────────────
    update_side_view(hPoleSide, angle_deg, magnet_r);

    % ── Axes 3: Zeitreihe aktualisieren ───────────────────────────────────────
    n_plot = length(angles_ord);
    t_plot = ((1:n_plot) - n_plot) / fs;
    set(hLine, 'XData', t_plot, 'YData', angles_ord);
    xlim(ax3, [max(t_plot(1), -BUFFER_LEN/fs)  0]);
    set(hAnno, 'String', sprintf('%.1f°', angle_deg));

    drawnow limitrate;
end

cleanup(s);

%% ═══════════════════════════════════════════════════════════════════════════
%% Lokale Hilfsfunktionen
%% ═══════════════════════════════════════════════════════════════════════════

% ── Dipol-Feld ────────────────────────────────────────────────────────────────
function B = dipole_field(m_vec, r_vec)
    r = norm(r_vec);
    if r < 1e-10; B = [0;0;0]; return; end
    r_hat = r_vec / r;
    B = (3 * dot(m_vec, r_hat) * r_hat - m_vec) / r^3;
end

% ── Zeile parsen ──────────────────────────────────────────────────────────────
function angle = parse_line(line, fmt, key)
    angle = nan;
    try
        switch fmt
            case 'A'   % "<float>"
                angle = str2double(line);
            case 'B'   % "<angle>,<raw>"
                parts = strsplit(line, ',');
                angle = str2double(parts{1});
            case 'C'   % ">key: <float>"  (Teleplot, z.B. ">as5600/angle_raw: 127.40")
                pat = ['>' regexptranslate('escape', key) ':\s*([\d.\-]+)'];
                tok = regexp(line, pat, 'tokens', 'once');
                if ~isempty(tok); angle = str2double(tok{1}); end
        end
    catch
    end
    if ~isfinite(angle); angle = nan; end
end

% ── Magnet-Patch-Vertices (Draufsicht) ────────────────────────────────────────
function [vN, vS] = magnet_verts(th, mr)
    mlen = mr*0.85; hw = mr*0.28;
    nx = cosd(th); ny = sind(th);
    px = -ny;      py =  nx;
    vN = [nx*mlen - px*hw, nx*mlen + px*hw, px*hw, -px*hw;
          ny*mlen - py*hw, ny*mlen + py*hw, py*hw, -py*hw];
    vS = [-nx*mlen - px*hw, -nx*mlen + px*hw, px*hw, -px*hw;
          -ny*mlen - py*hw, -ny*mlen + py*hw, py*hw, -py*hw];
end

% ── Draufsicht initialisieren ─────────────────────────────────────────────────
function [ph_N, ph_S, th_N, th_S, qsens, hAnno] = init_top_view(ax, th0, mr)
    [vN, vS] = magnet_verts(th0, mr);
    ph_N = patch(ax, vN(1,:), vN(2,:), [0.85 0.2 0.2], 'EdgeColor','none');
    ph_S = patch(ax, vS(1,:), vS(2,:), [0.2 0.35 0.75], 'EdgeColor','none');
    rTxt = mr*0.55;
    th_N = text(ax,  cosd(th0)*rTxt,  sind(th0)*rTxt, 'N', ...
                'Color','w', 'FontWeight','bold', 'HorizontalAlignment','center', 'FontSize',9);
    th_S = text(ax, -cosd(th0)*rTxt, -sind(th0)*rTxt, 'S', ...
                'Color','w', 'FontWeight','bold', 'HorizontalAlignment','center', 'FontSize',9);

    % AS5600 sitzt zentriert auf der Drehachse -> projiziert exakt im Ursprung
    scatter(ax, 0, 0, 90, 's', 'filled', 'MarkerFaceColor',[0.25 0.7 0.35], ...
            'MarkerEdgeColor','k', 'LineWidth',1.2);
    text(ax, 0, -mr*1.35, 'AS5600 (on-axis, darunter)', ...
         'FontSize',7.5, 'HorizontalAlignment','center');

    % Feldvektor am Sensor (Projektion in die xy-Ebene)
    qsens = quiver(ax, 0, 0, mr*0.8, 0, 0, 'Color',[0.9 0.6 0.1], ...
                   'LineWidth',2, 'MaxHeadSize',0.6);
    hAnno = text(ax, 0, mr*1.4, '', 'FontSize',8, 'Color',[0.5 0.3 0.0], ...
                 'HorizontalAlignment','center');
end

% ── Draufsicht aktualisieren ──────────────────────────────────────────────────
function update_top_view(ph_N, ph_S, th_N, th_S, qsens, hAnno, th, mr, B_cur)
    [vN, vS] = magnet_verts(th, mr);
    set(ph_N, 'XData', vN(1,:), 'YData', vN(2,:));
    set(ph_S, 'XData', vS(1,:), 'YData', vS(2,:));
    rTxt = mr*0.55;
    set(th_N, 'Position', [ cosd(th)*rTxt,  sind(th)*rTxt, 0]);
    set(th_S, 'Position', [-cosd(th)*rTxt, -sind(th)*rTxt, 0]);

    Bxy   = B_cur(1:2);
    Bnorm = norm(Bxy) + eps;
    arrow_len = mr*0.8;
    set(qsens, 'UData', Bxy(1)/Bnorm*arrow_len, 'VData', Bxy(2)/Bnorm*arrow_len);

    angle_B = mod(atan2d(Bxy(2), Bxy(1)), 360);
    set(hAnno, 'String', sprintf('Feldvektor am Sensor: %.1f°  (Magnet: %.1f°)', angle_B, th));
end

% ── Seitenansicht initialisieren ──────────────────────────────────────────────
function hPoleSide = init_side_view(ax, th0, mr, gap)
    % Magnet als horizontaler Balken (Kantenansicht der Scheibe), Drehebene bei z=0
    plot(ax, [-mr mr], [0 0], 'Color',[0.3 0.3 0.3], 'LineWidth',6);
    text(ax, 0, mr*0.6, 'Ringmagnet (Kantenansicht)', ...
         'FontSize',7.5, 'HorizontalAlignment','center');

    % Projektion des Nordpols entlang der Sichtachse — wandert mit der Drehung
    hPoleSide = plot(ax, cosd(th0)*mr, 0, 'o', 'MarkerSize',9, ...
                     'MarkerFaceColor',[0.85 0.2 0.2], 'MarkerEdgeColor','k', 'LineWidth',1);

    % AS5600-Chip, zentriert auf der Achse, einen Luftspalt darunter
    cw = mr*0.9; ch = mr*0.35;
    patch(ax, [-cw cw cw -cw], -gap + [-ch -ch ch ch], ...
          [0.25 0.7 0.35], 'EdgeColor','k', 'LineWidth',1.2);
    text(ax, 0, -gap-ch-mr*0.35, 'AS5600', 'FontSize',7.5, 'HorizontalAlignment','center');

    % Luftspalt-Bemaßung
    plot(ax, [mr*1.5 mr*1.5], [0 -gap], 'k--', 'LineWidth',1);
    text(ax, mr*1.6, -gap/2, sprintf('Luftspalt\n%.2f mm', gap*1000), ...
         'FontSize',7, 'VerticalAlignment','middle');

    % Drehachse andeuten
    plot(ax, [0 0], [mr*0.9, -gap-ch*1.4], 'k:', 'LineWidth',0.8);
end

% ── Seitenansicht aktualisieren ───────────────────────────────────────────────
function update_side_view(hPoleSide, th, mr)
    % Blick entlang der y-Achse: der Nordpol wandert mit cos(θ) zwischen den
    % Magnetkanten hin und her (steht er senkrecht zur Sichtachse, verschwindet
    % er hinter der Mitte — genau dann ist er am weitesten von dieser Ansicht
    % entfernt, aber am nächsten an der jeweils anderen Seitenposition).
    set(hPoleSide, 'XData', cosd(th)*mr, 'YData', 0);
end

% ── Cleanup ───────────────────────────────────────────────────────────────────
function cleanup(s)
    try; clear s; catch; end
    fprintf("Serial geschlossen.\n");
    delete(gcf);
end
