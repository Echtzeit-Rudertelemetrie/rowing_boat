% OAR ANGLE CALIBRATION SIMULATION
% ─────────────────────────────────────────────────────────────────────────────
% Problem:
%   - Middle ESP: GPS → Course Over Ground (COG) = Bootsrichtung (Grad von Nord)
%   - Left/Right ESP: ahrsfilter → Yaw = absolute Himmelsrichtung des Sensors
%
% Ziel:
%   1. Welche Seite bin ich? (links / rechts)
%   2. Wo ist mein Nullpunkt? (Ruder senkrecht zum Boot)
%
% Konzept:
%   oar_angle = Yaw_sensor - GPS_COG
%   → positiv = Backbord (rechts wenn du in Fahrtrichtung schaust)
%   → negativ = Steuerbord (links)
%
%   Beim Start: Boot fährt geradeaus, Ruder wird senkrecht gehalten (90°)
%   → null_offset = Yaw_sensor - GPS_COG - 90°  (für rechtes Ruder)
%   → null_offset = Yaw_sensor - GPS_COG + 90°  (für linkes Ruder)
% ─────────────────────────────────────────────────────────────────────────────

close all; clear;

%% Parameter
GPS_COG_deg   = 45;          % Boot fährt in Richtung NE (45° von Nord)
stroke_deg    = 60;          % Ruderschlag ±60° um Nullpunkt
n_samples     = 200;         % Simulationsschritte pro Schlag

%% Ruderschlag simulieren
% t: 0 → 1 über einen Schlag
t = linspace(0, 2*pi, n_samples);
stroke = stroke_deg * sin(t);   % Sinusförmige Schlagbewegung

% Nullpunkte der Ruder: senkrecht zum Boot
null_right_deg = GPS_COG_deg + 90;   % Rechtes Ruder: 90° rechts von Bootsachse
null_left_deg  = GPS_COG_deg - 90;   % Linkes Ruder: 90° links von Bootsachse

% Absoluter Yaw des Sensors (was der ahrsfilter ausgibt)
yaw_right = null_right_deg + stroke;   % Rechtes Ruder
yaw_left  = null_left_deg  + stroke;   % Linkes Ruder

% Rauschen hinzufügen (simuliert Sensor-Noise)
noise_std = 0.5;   % Grad
yaw_right = yaw_right + noise_std * randn(size(yaw_right));
yaw_left  = yaw_left  + noise_std * randn(size(yaw_left));

%% Winkelberechnung (so wie es der ESP macht)
% Schritt 1: relativer Winkel zur Bootsachse
rel_right = wrapTo180(yaw_right - GPS_COG_deg);   % +90° in Ruhe → rechts
rel_left  = wrapTo180(yaw_left  - GPS_COG_deg);   % -90° in Ruhe → links

% Schritt 2: Seite bestimmen
% In Kalibrierphase (erster Wert): Ruder senkrecht gehalten
cal_right = rel_right(1);
cal_left  = rel_left(1);
fprintf("Kalibrierung:\n");
fprintf("  Rechtes Ruder – rel. Winkel: %.1f°  → Seite: %s\n", cal_right, ite(cal_right > 0, "RECHTS", "LINKS"));
fprintf("  Linkes  Ruder – rel. Winkel: %.1f°  → Seite: %s\n", cal_left,  ite(cal_left  > 0, "RECHTS", "LINKS"));

% Schritt 3: Nullpunkt-Offset bestimmen
% Erwartung: rechtes Ruder = +90°, linkes = -90°
null_offset_right = cal_right - 90;
null_offset_left  = cal_left  + 90;
fprintf("  Nullpunkt-Offset rechts: %.1f°\n", null_offset_right);
fprintf("  Nullpunkt-Offset links:  %.1f°\n", null_offset_left);

% Schritt 4: Ruderwinkel relativ zum Nullpunkt
oar_right = rel_right - null_offset_right - 90;
oar_left  = rel_left  - null_offset_left  + 90;

%% Plot
figure('Name', 'Oar Angle Calibration Simulation', 'Position', [100 100 1200 700]);

% --- Plot 1: Absolute Yaw der Sensoren ---
subplot(2,2,1);
plot(yaw_right, 'r-', 'LineWidth', 1.5); hold on;
plot(yaw_left,  'b-', 'LineWidth', 1.5);
yline(GPS_COG_deg, 'k--', 'GPS COG', 'LineWidth', 1);
yline(null_right_deg, 'r:', 'Null rechts');
yline(null_left_deg,  'b:', 'Null links');
legend('Yaw rechts', 'Yaw links', 'Location', 'best');
title('Absoluter Yaw (ahrsfilter Output)');
xlabel('Sample'); ylabel('Grad von Nord');
grid on;

% --- Plot 2: Relativer Winkel zur Bootsachse ---
subplot(2,2,2);
plot(rel_right, 'r-', 'LineWidth', 1.5); hold on;
plot(rel_left,  'b-', 'LineWidth', 1.5);
yline( 90, 'r:', 'Nullpunkt rechts (+90°)');
yline(-90, 'b:', 'Nullpunkt links (-90°)');
yline(0, 'k--', 'Bootsachse');
legend('Rel. Winkel rechts', 'Rel. Winkel links', 'Location', 'best');
title('Winkel relativ zur Bootsachse (= Yaw - GPS\_COG)');
xlabel('Sample'); ylabel('Grad');
grid on;

% --- Plot 3: Ruderwinkel nach Kalibrierung ---
subplot(2,2,3);
plot(oar_right, 'r-', 'LineWidth', 1.5); hold on;
plot(oar_left,  'b-', 'LineWidth', 1.5);
plot(stroke, 'k--', 'LineWidth', 1, 'DisplayName', 'Soll (ohne Rauschen)');
yline(0, 'k:', 'Nullpunkt');
legend('Ruderwinkel rechts', 'Ruderwinkel links', 'Soll', 'Location', 'best');
title('Ruderwinkel nach Kalibrierung');
xlabel('Sample'); ylabel('Grad');
grid on;

% --- Plot 4: Draufsicht auf Boot + Ruder ---
subplot(2,2,4);
theta = linspace(0, 2*pi, 100);
i_show = round(linspace(1, n_samples, 8));   % 8 Zeitpunkte zeigen
hold on; axis equal;
for k = 1:length(i_show)
    i = i_show(k);
    alpha_r = deg2rad(yaw_right(i) - 90);   % Ruderrichtung in Kartenkoordinaten
    alpha_l = deg2rad(yaw_left(i)  - 90);
    quiver(1, 0, cos(alpha_r)*0.8, sin(alpha_r)*0.8, 0, 'r', 'LineWidth', 1.2);
    quiver(-1, 0, cos(alpha_l)*0.8, sin(alpha_l)*0.8, 0, 'b', 'LineWidth', 1.2);
end
% Boot
boat_angle = deg2rad(GPS_COG_deg - 90);
quiver(0, 0, cos(boat_angle)*1.5, sin(boat_angle)*1.5, 0, 'k', 'LineWidth', 2);
plot(0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot(1, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');   % rechter Punkt
plot(-1, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');  % linker Punkt
text(0.15, 0, sprintf('COG = %d°', GPS_COG_deg), 'FontSize', 8);
title('Draufsicht: Ruderpositionen über Zeit');
xlabel('Ost →'); ylabel('Nord ↑');
grid on;

sgtitle(sprintf('Simulation: Boot fährt %d° (NE), Schlag ±%d°', GPS_COG_deg, stroke_deg));

%% Zusammenfassung
fprintf("\nErgebnis:\n");
fprintf("  RMSE rechts: %.2f°\n", rms(oar_right - stroke'));
fprintf("  RMSE links:  %.2f°\n", rms(oar_left  - stroke'));
fprintf("  → Genauigkeit durch Sensor-Rauschen von %.1f° std\n", noise_std);

% ─────────────────────────────────────────────────────────────────────────────
% Hilfsfunktion: ternärer Operator
function out = ite(cond, a, b)
    if cond; out = a; else; out = b; end
end
