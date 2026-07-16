close all; clear

s = serialport; % anpassen
configureTerminator(s, "LF");
flush(s);

num_points = 1000;          % ein großer Datensatz statt 5 kleiner
mag_raw = zeros(num_points, 3);
mag = nan(1, 3);
got = false(1, 3);          % Komponenten-Tracking statt Reihenfolgeannahme
i = 0;

% Live-Plot zur Abdeckungskontrolle
fig = figure; ax = axes(fig); hold(ax, "on"); axis(ax, "equal");
h = scatter3(ax, nan, nan, nan, 8, "filled");
title(ax, "Sensor langsam um alle Achsen taumeln — Sphäre füllen");

fprintf("Aufnahme läuft (%d Punkte)...\n", num_points);
while i < num_points
    line = readline(s);
    if contains(line, "mag/x"), mag(1) = sscanf(extractAfter(line, ": "), "%f"); got(1) = true; end
    if contains(line, "mag/y"), mag(2) = sscanf(extractAfter(line, ": "), "%f"); got(2) = true; end
    if contains(line, "mag/z"), mag(3) = sscanf(extractAfter(line, ": "), "%f"); got(3) = true; end

    if all(got)                             % nur vollständige Tripel
        if all(isfinite(mag))
            i = i + 1;
            mag_raw(i, :) = mag;
            if mod(i, 20) == 0              % Plot-Update gedrosselt
                set(h, "XData", mag_raw(1:i,1), "YData", mag_raw(1:i,2), "ZData", mag_raw(1:i,3));
                drawnow limitrate
            end
        end
        got(:) = false; mag(:) = nan;
    end
end

% Ein Fit über alle Daten
[A, b, expMFS] = magcal(mag_raw);
mag_cal = (mag_raw - b) * A;

% Qualitätsmetrik: Residuum der Feldstärke
r = vecnorm(mag_cal, 2, 2);
res_pct = 100 * std(r) / expMFS;
fprintf("expMFS = %.2f µT, Residuum = %.2f %% (Ziel: < 2–3 %%)\n", expMFS, res_pct);

% Vorher/Nachher
figure; hold on; axis equal
scatter3(mag_raw(:,1), mag_raw(:,2), mag_raw(:,3), 8, "r", "filled");
scatter3(mag_cal(:,1), mag_cal(:,2), mag_cal(:,3), 8, "b", "filled");
legend("roh", "kalibriert"); view(3);

disp("A = " + mat2str(A, 6));
disp("b = " + mat2str(b, 6));