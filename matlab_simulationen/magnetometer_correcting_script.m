close all
clear
s = serialport("/dev/cu.usbserial-110", 115200);
configureTerminator(s, "LF");

num_points = 300;
num_runs   = 5;  % Anzahl Iterationen

A_all = zeros(3, 3, num_runs);
b_all = zeros(num_runs, 3);

for run = 1:num_runs
    fprintf("Run %d/%d — Sensor rotieren, dann Enter drücken\n", run, num_runs);
    pause();  % Warten bis bereit

    mag_ave = zeros(num_points, 3);
    mag = zeros(1, 3);
    i = 0;

    while i < num_points
        line = readline(s);
        if contains(line, "mag/x"), mag(1) = sscanf(extractAfter(line, ": "), "%f"); end
        if contains(line, "mag/y"), mag(2) = sscanf(extractAfter(line, ": "), "%f"); end
        if contains(line, "mag/z")
            mag(3) = sscanf(extractAfter(line, ": "), "%f");
            if all(isfinite(mag))
                i = i + 1;
                mag_ave(i, :) = mag;
            end
        end
    end

    [A_run, b_run, ~] = magcal(mag_ave);
    A_all(:,:,run) = A_run;
    b_all(run, :)  = b_run;

    fprintf("Run %d fertig: b = %.4f  %.4f  %.4f\n", run, b_run(1), b_run(2), b_run(3));
end

A = mean(A_all, 3);
b = mean(b_all, 1);

disp('Gemitteltes A:'); disp(A);
disp('Gemitteltes b:'); disp(b);


% distorted ellipsoid
scatter3(mag_ave(:,1), mag_ave(:,2), mag_ave(:,3));
hold all

% Find calibration coefficients
[A, b, expMFS] = magcal(mag_ave);
xCorrected = (mag_ave - b) * A;

%Calibrated ellipsoid
scatter3(xCorrected(:,1), xCorrected(:,2), xCorrected(:,3));

display(A); %soft iron correction matrix
display(b); %hard iron correction bias