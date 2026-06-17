close all; clear; clc;
delete(serialportfind);

s = serialport("/dev/cu.usbserial-110", 115200);
configureTerminator(s, "LF");
flush(s);

%% Figure
fig = figure('Name', 'GPS Live', 'Position', [100 100 700 700]);
ax  = axes('Parent', fig);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'Lon'); ylabel(ax, 'Lat');
title(ax, 'GPS Track');

h_track   = plot(ax, NaN, NaN, 'c-',  'LineWidth', 1.5);
h_pos     = plot(ax, NaN, NaN, 'yo',  'MarkerSize', 10, 'MarkerFaceColor', 'y');
h_cog     = quiver(ax, 0, 0, 0, 0,    'r', 'LineWidth', 2, 'MaxHeadSize', 1, 'AutoScale', 'off');
t_info    = title(ax, 'Warte auf Fix...');

track_lat = [];
track_lon = [];

lat = 0; lon = 0; fix = 0; cog = 0; speed = 0; sats = 0;

fprintf("Warte auf GPS-Fix...\n");

while true
    if s.NumBytesAvailable == 0; pause(0.01); continue; end

    % Alle gepufferten Zeilen auf einmal leeren
    while s.NumBytesAvailable > 0
        line = readline(s);

        if contains(line, ">gps/cog_deg"),   cog   = sscanf(extractAfter(line, ": "), "%f"); end
        if contains(line, ">gps/speed_kmh"), speed = sscanf(extractAfter(line, ": "), "%f"); end
        if contains(line, ">gps/sats"),      sats  = sscanf(extractAfter(line, ": "), "%f"); end

        if contains(line, "i2c") && contains(line, "lat:")
            fix = double(contains(line, "fix: yes"));
            tok = regexp(line, 'lat:\s*([\d.\-]+).*lon:\s*([\d.\-]+)', 'tokens');
            if ~isempty(tok)
                lat = str2double(tok{1}{1});
                lon = str2double(tok{1}{2});
                if fix
                    track_lat(end+1) = lat;
                    track_lon(end+1) = lon;
                end
            end
        end
    end

    % Plot einmal nach dem Leeren des Buffers updaten
    if fix && ~isempty(track_lat)
        set(h_track, 'XData', track_lon, 'YData', track_lat);
        set(h_pos,   'XData', lon, 'YData', lat);
        arrow_len = max(0.00005, speed * 0.000005);
        set(h_cog, 'XData', lon, 'YData', lat, ...
                   'UData', arrow_len * sind(cog), ...
                   'VData', arrow_len * cosd(cog));
        if length(track_lat) > 1
            pad = 0.0002;
            xlim(ax, [min(track_lon)-pad, max(track_lon)+pad]);
            ylim(ax, [min(track_lat)-pad, max(track_lat)+pad]);
        end
    end
    set(t_info, 'String', sprintf('Fix: %d  |  Lat: %.6f  Lon: %.6f  |  COG: %.1f°  |  %.1f km/h  |  %d Sats', ...
        fix, lat, lon, cog, speed, sats));
    drawnow;
end
