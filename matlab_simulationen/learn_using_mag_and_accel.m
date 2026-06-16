% Aus dem normierten Accelerometer ergibt sich die Down-Richtung D = -accel, aus cross(D, mag) die East-Achse E, aus cross(E, D) die North-Achse N.
% Die drei Achsen bilden eine Rotationsmatrix C, die per dcm2quat zur Quaternion wird und in poseplot angezeigt wird.
% Das erklärt das Messmodell des EKF: Schwerkraft + Magnetfeld legen die Orientierung absolut fest.

close all
clear
delete(serialportfind)

s = serialport("/dev/cu.usbserial-110", 115200);
configureTerminator(s, "LF");
flush(s);

viewer = poseplot;

accel = zeros(1,3);
gyro  = zeros(1,3);
mag   = zeros(1,3);

b = [30.7073, -19.2218, -46.3756];
A = eye(3);

while true
    line = readline(s);
    if contains(line, "Acce_x"), accel(3) = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Acce_y"), accel(2) = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Acce_z"), accel(1) = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "mag/x"),  mag(1)   = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "mag/y"),  mag(2)   = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "mag/z")
        mag(3) = sscanf(extractAfter(line,": "), "%f");

        mag_cal = (mag - b) * A;
        %mag_cal = mag;

        mag_unit = mag_cal ./ norm(mag_cal);

        acc_ave = accel; %mean(accel);
        accel_unit = acc_ave ./ norm(acc_ave);

        D = -accel_unit;
        E = cross(D, mag_unit);
        E = E ./ norm(E);
        N = cross(E, D);
        N = N ./ norm(N);

        C = [N', E', D'];
        Q = quaternion(dcm2quat(C));
        viewer.Orientation = Q;
    end
end