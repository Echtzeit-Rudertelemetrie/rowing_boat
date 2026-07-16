close all
clear
clc
delete(serialportfind)
PACKET_VALUES = 32;          % siehe AppTypes.h
s = serialport; %findet serialport automatisch....
configureTerminator(s, "LF");
flush(s);
viewer = poseplot;
ax = gca;
% KEINE Draufsicht (Elevation 90°) wie main_test_S3.m: dort war die volle
% 3D-Fusion aktiv und Yaw (Drehung um die vertikale Achse) von oben gut
% sichtbar. Hier wird nur Roll gesetzt (Pitch/Yaw fest 0) -- aus der
% Draufsicht liegt die Roll-Achse fast in der Blickrichtung, die Rotation
% ist praktisch unsichtbar. Schraege Ansicht zeigt Roll dagegen deutlich.
view(ax, 135, 20);
while true
    line = readline(s);
    fprintf("%s\n", line);
% Beispiel: "Angle Values [deg]: [12.34, -5.67, ..., 8.90]"
if contains(line, "Angle Values")
        angle = sscanf(extractAfter(line, "[deg]: ["), "%f,")';
if numel(angle) == PACKET_VALUES
for i = 1:PACKET_VALUES
                q = quaternion([0 0 angle(i)], 'eulerd', 'ZYX', 'frame');
                viewer.Orientation = q;
                drawnow limitrate
end
end
end
end