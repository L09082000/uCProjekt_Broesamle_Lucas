%% =========================================================
%  Live Plot STM32 Sensor Data (LSM6DSL + LIS3MDL)
%  Autor: Lucas Brösamle
%  Abtastrate: 100 Hz
%  Datenformat: packed struct (float, little-endian)
% =========================================================

clear; clc; close all;

%% ----------------- Serielle Schnittstelle ----------------
port = "COM4";
baudrate = 115200;

s = serialport(port, baudrate);
configureTerminator(s, 0);
flush(s);

disp("✔ Serielle Verbindung geöffnet");

%% ----------------- Protokollparameter -------------------
DELIMITER = single(123456.0);   % exakt derselbe float wie im STM32!
NUM_FLOATS = 1 + 6 + 6 + 3 + 3;  % delimiter + acc + gyro + acc_f + gyro_f + mag + mag_f
PACKET_BYTES = NUM_FLOATS * 4;  % float32

%% ----------------- Speicher für Daten -------------------
fs = 100;              % Hz
t = [];                % Zeitachse

acc_raw  = [];
acc_filt = [];
gyro_raw  = [];
gyro_filt = [];
mag_raw  = [];
mag_filt = [];

sample_idx = 0;

%% ================== FIGURE 1: Accelerometer ==============
figure(1); clf; hold on; grid on;
title("Accelerometer");
xlabel("Zeit [s]");
ylabel("m/s²");

hAccX  = plot(nan, nan, 'b');
hAccY  = plot(nan, nan, 'r');
hAccZ  = plot(nan, nan, 'g');
hAccXf = plot(nan, nan, 'b', 'LineWidth', 1.5);
hAccYf = plot(nan, nan, 'r', 'LineWidth', 1.5);
hAccZf = plot(nan, nan, 'g', 'LineWidth', 1.5);

legend([hAccX hAccY hAccZ hAccXf hAccYf hAccZf], ...
       {'Acc X raw','Acc Y raw','Acc Z raw', ...
        'Acc X filt','Acc Y filt','Acc Z filt'});

%% ================== FIGURE 2: Gyroscope ==================
figure(2); clf; hold on; grid on;
title("Gyroscope");
xlabel("Zeit [s]");
ylabel("°/s");

hGyroX  = plot(nan, nan, 'b');
hGyroY  = plot(nan, nan, 'r');
hGyroZ  = plot(nan, nan, 'g');
hGyroXf = plot(nan, nan, 'b', 'LineWidth', 1.5);
hGyroYf = plot(nan, nan, 'r', 'LineWidth', 1.5);
hGyroZf = plot(nan, nan, 'g', 'LineWidth', 1.5);

legend([hGyroX hGyroY hGyroZ hGyroXf hGyroYf hGyroZf], ...
       {'Gyro X raw','Gyro Y raw','Gyro Z raw', ...
        'Gyro X filt','Gyro Y filt','Gyro Z filt'});

%% ================== FIGURE 3: Magnetometer ===============
figure(3); clf; hold on; grid on;
title("Magnetometer");
xlabel("Zeit [s]");
ylabel("µT");

hMagX  = plot(nan, nan, 'b');
hMagY  = plot(nan, nan, 'r');
hMagZ  = plot(nan, nan, 'g');
hMagXf = plot(nan, nan, 'b', 'LineWidth', 1.5);
hMagYf = plot(nan, nan, 'r', 'LineWidth', 1.5);
hMagZf = plot(nan, nan, 'g', 'LineWidth', 1.5);

legend([hMagX hMagY hMagZ hMagXf hMagYf hMagZf], ...
       {'Mag X raw','Mag Y raw','Mag Z raw', ...
        'Mag X filt','Mag Y filt','Mag Z filt'});

%% ================== Hauptschleife ========================
try
    while true

        % Warten bis ein komplettes Paket verfügbar ist
        if s.NumBytesAvailable < PACKET_BYTES
            pause(0.001);
            continue;
        end

        % Rohdaten lesen
        raw_bytes = read(s, PACKET_BYTES, "uint8");

        % In float32 umwandeln
        data = typecast(uint8(raw_bytes), 'single');

        % Delimiter prüfen
        if data(1) ~= DELIMITER
            continue;   % Paket verwerfen
        end

        % ----------------- Daten aufteilen ----------------
        acc_r   = data(2:4);
        gyro_r  = data(5:7);
        acc_f   = data(8:10);
        gyro_f  = data(11:13);
        mag_r   = data(14:16);
        mag_f   = data(17:19);

        % ----------------- Zeit & Speicher ----------------
        sample_idx = sample_idx + 1;
        t(sample_idx) = sample_idx / fs;

        acc_raw(:,sample_idx)  = acc_r;
        acc_filt(:,sample_idx) = acc_f;
        gyro_raw(:,sample_idx)  = gyro_r;
        gyro_filt(:,sample_idx) = gyro_f;
        mag_raw(:,sample_idx)  = mag_r;
        mag_filt(:,sample_idx) = mag_f;

        % ----------------- Plots aktualisieren -------------
        set(hAccX,  'XData', t, 'YData', acc_raw(1,:));
        set(hAccY,  'XData', t, 'YData', acc_raw(2,:));
        set(hAccZ,  'XData', t, 'YData', acc_raw(3,:));
        set(hAccXf, 'XData', t, 'YData', acc_filt(1,:));
        set(hAccYf, 'XData', t, 'YData', acc_filt(2,:));
        set(hAccZf, 'XData', t, 'YData', acc_filt(3,:));

        set(hGyroX,  'XData', t, 'YData', gyro_raw(1,:));
        set(hGyroY,  'XData', t, 'YData', gyro_raw(2,:));
        set(hGyroZ,  'XData', t, 'YData', gyro_raw(3,:));
        set(hGyroXf, 'XData', t, 'YData', gyro_filt(1,:));
        set(hGyroYf, 'XData', t, 'YData', gyro_filt(2,:));
        set(hGyroZf, 'XData', t, 'YData', gyro_filt(3,:));

        set(hMagX,  'XData', t, 'YData', mag_raw(1,:));
        set(hMagY,  'XData', t, 'YData', mag_raw(2,:));
        set(hMagZ,  'XData', t, 'YData', mag_raw(3,:));
        set(hMagXf, 'XData', t, 'YData', mag_filt(1,:));
        set(hMagYf, 'XData', t, 'YData', mag_filt(2,:));
        set(hMagZf, 'XData', t, 'YData', mag_filt(3,:));

        drawnow limitrate
    end

catch
    disp("Messung beendet");
end

%% ----------------- Aufräumen ------------------------------
clear s
disp("✔ Serielle Verbindung geschlossen");