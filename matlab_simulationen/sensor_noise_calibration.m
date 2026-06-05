% sensor_noise_calibration.m
% Sensor still on a flat surface, do not move during measurement!
% Outputs noise variances ready to paste into ekf_update.m

close all; clear; clc;
delete(serialportfind)

%% Config
N_SAMPLES   = 1000;   % number of samples per sensor axis
PORT        = "/dev/cu.usbserial-110";
BAUD        = 115200;

A = [1.0962  0       0;
     0       1.4670  0;
     0       0       0.6219];
b = [-115.4853   6.6805  -14.4159];

%% Collect
s = serialport(PORT, BAUD);
configureTerminator(s, "LF");
flush(s);

fprintf("Collecting %d samples — keep sensor STILL!\n", N_SAMPLES);

accel_raw = zeros(N_SAMPLES, 3);
gyro_raw  = zeros(N_SAMPLES, 3);
mag_raw   = zeros(N_SAMPLES, 3);

accel_buf = zeros(1,3);
gyro_buf  = zeros(1,3);
mag_buf   = zeros(1,3);

count = 0;
while count < N_SAMPLES
    line = readline(s);
    if contains(line, "Acce_x"), accel_buf(3) = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Acce_y"), accel_buf(2) = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Acce_z"), accel_buf(1) = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Gyro_x"), gyro_buf(3)  = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Gyro_y"), gyro_buf(2)  = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "Gyro_z"), gyro_buf(1)  = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "mag/x"),  mag_buf(1)   = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "mag/y"),  mag_buf(2)   = sscanf(extractAfter(line,": "),"%f"); end
    if contains(line, "mag/z")
        mag_buf(3) = sscanf(extractAfter(line,": "),"%f");
        count = count + 1;
        accel_raw(count,:) = accel_buf;
        gyro_raw(count,:)  = gyro_buf;
        mag_raw(count,:)   = mag_buf;
        if mod(count, 100) == 0
            fprintf("  %d / %d\n", count, N_SAMPLES);
        end
    end
end
delete(s);
fprintf("Done collecting.\n\n");

%% Apply magnetometer calibration
mag_cal = (mag_raw - b) * A;   % µT
mag_T   = mag_cal * 1e-6;      % T (for EKF)

%% Statistics
labels = ["Accel X","Accel Y","Accel Z","Gyro X","Gyro Y","Gyro Z","Mag X","Mag Y","Mag Z"];
data   = [accel_raw, gyro_raw, mag_cal];

means = mean(data);
stds  = std(data);
vars  = var(data);

fprintf("%-10s  %10s  %10s  %10s\n", "Sensor", "Mean", "Std Dev", "Variance");
fprintf("%s\n", repmat('-',1,48));
for i = 1:9
    fprintf("%-10s  %10.5f  %10.5f  %10.5f\n", labels(i), means(i), stds(i), vars(i));
end

%% EKF noise values (variance of the measurement noise)
accel_noise_ekf  = mean(vars(1:3));
gyro_noise_ekf   = mean(vars(4:6));
magnet_noise_ekf = mean(vars(7:9) * 1e-12);  % converted to T²

fprintf("\n=== Paste these into ekf_update.m ===\n");
fprintf("gyro_noise   = %.4f;\n", gyro_noise_ekf);
fprintf("accel_noise  = %.4f;\n", accel_noise_ekf);
fprintf("magnet_noise = %.4e;  %% in T²\n", magnet_noise_ekf);

%% Plots
figure("Name","Sensor Noise");
sensor_data = {accel_raw, gyro_raw, mag_cal};
sensor_names = {"Accelerometer (raw m/s²)", "Gyroscope (raw °/s)", "Magnetometer calibrated (µT)"};
axis_labels  = ["X","Y","Z"];

for s_idx = 1:3
    d = sensor_data{s_idx};
    for ax = 1:3
        subplot(3,3,(s_idx-1)*3+ax);
        plot(d(:,ax));
        hold on;
        yline(mean(d(:,ax)),   'r--', 'Mean');
        yline(mean(d(:,ax))+std(d(:,ax)), 'g:', '+σ');
        yline(mean(d(:,ax))-std(d(:,ax)), 'g:', '-σ');
        title(sprintf("%s %s  σ=%.4f", sensor_names{s_idx}, axis_labels(ax), std(d(:,ax))));
        xlabel("Sample"); grid on;
    end
end
