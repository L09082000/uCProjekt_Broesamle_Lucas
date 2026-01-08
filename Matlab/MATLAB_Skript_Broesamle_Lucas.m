% IMU Datenvisualisierung mit Extended Kalman Filter und Magnetometer-Kalibrierung
% Autor: Generiert für IMU-Datenerfassung
% Datum: 2025

classdef MATLAB_Skript_Broesamle_Lucas < handle
    properties
        % Serielle Verbindung
        serialPort
        portName = 'COM4'  % Anpassen an Ihren Port
        baudRate = 115200
        
        % GUI Elemente
        fig
        axAcc
        axGyro
        axMag
        ax3D
        btnCalibrate
        btnSaveCalib
        btnLoadCalib
        btnResetPose
        btnConnect
        btnStop
        txtStatus
        txtInstructions
        txtLog  % Neues Log-Textfeld
        progressBarX  % Fortschrittsbalken X
        progressBarY  % Fortschrittsbalken Y
        progressBarZ  % Fortschrittsbalken Z
        txtProgressX  % Text für X-Fortschritt
        txtProgressY  % Text für Y-Fortschritt
        txtProgressZ  % Text für Z-Fortschritt
        
        % Datenstruktur
        dataSize = 19  % 19 floats pro Paket
        delimiter = 0xDEADBEEF  % Beispiel-Delimiter
        
        % Datenpuffer
        maxBufferSize = 1000
        timeBuffer
        accBuffer
        gyroBuffer
        magBuffer
        accFilterBuffer
        gyroFilterBuffer
        magFilterBuffer
        
        % EKF Zustand
        ekf_state  % [q0, q1, q2, q3, bias_x, bias_y, bias_z] - Quaternion + Gyro Bias
        ekf_P      % Kovarianzmatrix
        dt = 0.1   % 100ms Abtastrate
        
        % Kalibrierungsdaten
        isCalibrating = false
        calibData
        magOffset = [0; 0; 0]
        magScale = [1; 1; 1]
        calibFileName = 'imu_mag_calibration.mat'
        calibCoverage = [0; 0; 0]  % Abdeckung für X, Y, Z Achse (0-100%)
        
        % Pose Offset für Nullstellung
        poseOffset = [0; 0; 0]  % [roll_offset, pitch_offset, yaw_offset]
        
        % Plot Handles
        hAcc
        hGyro
        hMag
        hAccFilt
        hGyroFilt
        hMagFilt
        hBody
        
        % Laufzeitvariablen
        isRunning = false
        dataCount = 0
    end
    
    methods
        function obj = MATLAB_Skript_Broesamle_Lucas()
            obj.initGUI();
            obj.initEKF();
            obj.initBuffers();
            obj.loadCalibration();
        end
        
        function initGUI(obj)
            % Hauptfenster erstellen
            obj.fig = figure('Name', 'IMU Visualisierung & Kalibrierung', ...
                'NumberTitle', 'off', ...
                'Position', [50 50 1600 900], ...
                'CloseRequestFcn', @(~,~)obj.closeApp());
            
            % Beschleunigungssensor Plot (oben links)
            obj.axAcc = subplot(2,3,1);
            title('Beschleunigung [m/s²]');
            xlabel('Zeit [s]');
            ylabel('Beschleunigung');
            hold on; grid on;
            obj.hAcc = plot(0, [0 0 0]);
            obj.hAcc = plot(0, [0 0 0], 'LineWidth', 1.5);
            obj.hAccFilt = plot(0, [0 0 0], '--', 'LineWidth', 1.2);
            legend('X roh','Y roh','Z roh','X gef.','Y gef.','Z gef.');
            
            % Gyroskop Plot (oben mitte)
            obj.axGyro = subplot(2,3,2);
            title('Winkelgeschwindigkeit [°/s]');
            xlabel('Zeit [s]');
            ylabel('Winkelgeschwindigkeit');
            hold on; grid on;
            obj.hGyro = plot(0, [0 0 0]);
            obj.hGyro = plot(0, [0 0 0], 'LineWidth', 1.5);
            obj.hGyroFilt = plot(0, [0 0 0], '--', 'LineWidth', 1.2);
            legend('X roh','Y roh','Z roh','X gef.','Y gef.','Z gef.');
            
            % Magnetometer Plot (oben rechts)
            obj.axMag = subplot(2,3,3);
            title('Magnetfeld [µT]');
            xlabel('Zeit [s]');
            ylabel('Magnetfeld');
            hold on; grid on;
            obj.hMag = plot(0, [0 0 0]);
            obj.hMag = plot(0, [0 0 0], 'LineWidth', 1.5);
            obj.hMagFilt = plot(0, [0 0 0], '--', 'LineWidth', 1.2);
            legend('X roh','Y roh','Z roh','X gef.','Y gef.','Z gef.');
            
            % 3D Pose Visualisierung (unten links, größer)
            obj.ax3D = subplot(2,3,[4,5]);
            title('3D Orientierung (Extended Kalman Filter)');
            xlabel('X'); ylabel('Y'); zlabel('Z');
            axis equal; grid on;
            view(45, 30);
            hold on;
            
            % Log-Textfeld mit Scrollbar (unten rechts)
            obj.txtLog = uicontrol('Style', 'listbox', ...
                'String', {}, ...
                'Position', [1070 80 510 320], ...
                'FontName', 'Courier New', ...
                'FontSize', 8, ...
                'HorizontalAlignment', 'left', ...
                'Max', 2, ...
                'Min', 0);
            
            uicontrol('Style', 'text', ...
                'String', 'System-Log:', ...
                'Position', [1070 405 100 20], ...
                'HorizontalAlignment', 'left', ...
                'FontWeight', 'bold');
            
            % Steuerungsbuttons
            obj.btnConnect = uicontrol('Style', 'pushbutton', ...
                'String', 'Verbinden', ...
                'Position', [20 20 100 40], ...
                'Callback', @(~,~)obj.connectSerial());
            
            obj.btnStop = uicontrol('Style', 'pushbutton', ...
                'String', 'Stop', ...
                'Position', [130 20 100 40], ...
                'Enable', 'off', ...
                'Callback', @(~,~)obj.stopAcquisition());
            
            obj.btnCalibrate = uicontrol('Style', 'pushbutton', ...
                'String', 'Magnetometer kalibrieren', ...
                'Position', [240 20 150 40], ...
                'BackgroundColor', [0.3 0.7 1.0], ...
                'Callback', @(~,~)obj.startCalibration());
            
            obj.btnSaveCalib = uicontrol('Style', 'pushbutton', ...
                'String', 'Kalibrierung speichern', ...
                'Position', [400 20 140 40], ...
                'Callback', @(~,~)obj.saveCalibration());
            
            obj.btnLoadCalib = uicontrol('Style', 'pushbutton', ...
                'String', 'Kalibrierung laden', ...
                'Position', [550 20 140 40], ...
                'Callback', @(~,~)obj.loadCalibration());
            
            obj.btnResetPose = uicontrol('Style', 'pushbutton', ...
                'String', 'Pose auf 0° setzen', ...
                'Position', [700 20 140 40], ...
                'BackgroundColor', [0.8 1.0 0.8], ...
                'Callback', @(~,~)obj.resetPose());
            
            % Statustext
            obj.txtStatus = uicontrol('Style', 'text', ...
                'String', 'Bereit zum Verbinden', ...
                'Position', [850 20 740 40], ...
                'HorizontalAlignment', 'left', ...
                'FontSize', 10);
            
            % Kalibrierungsanweisungen (höher positioniert)
            obj.txtInstructions = uicontrol('Style', 'text', ...
                'String', '', ...
                'Position', [20 770 1000 110], ...
                'HorizontalAlignment', 'left', ...
                'FontSize', 9, ...
                'BackgroundColor', [1 1 0.8], ...
                'Visible', 'off');
            
            % Fortschrittsbalken für Kalibrierung (initial unsichtbar)
            yPos = 685;
            barWidth = 400;
            barHeight = 25;
            xStart = 1070;
            
            % X-Achse Fortschritt
            uicontrol('Style', 'text', ...
                'String', 'X-Achse:', ...
                'Position', [xStart-80 yPos+45 70 20], ...
                'HorizontalAlignment', 'right', ...
                'FontWeight', 'bold', ...
                'Tag', 'calibProgressLabel', ...
                'Visible', 'off');
            
            obj.progressBarX = axes('Parent', obj.fig, ...
                'Position', [(xStart)/obj.fig.Position(3) (yPos+45)/obj.fig.Position(4) barWidth/obj.fig.Position(3) barHeight/obj.fig.Position(4)], ...
                'XLim', [0 100], 'YLim', [0 1], ...
                'XTick', [], 'YTick', [], ...
                'Box', 'on', 'Tag', 'calibProgressBar', ...
                'Visible', 'off');
            
            obj.txtProgressX = uicontrol('Style', 'text', ...
                'String', '0%', ...
                'Position', [xStart+barWidth+10 yPos+45 50 20], ...
                'HorizontalAlignment', 'left', ...
                'FontWeight', 'bold', ...
                'Tag', 'calibProgressLabel', ...
                'Visible', 'off');
            
            % Y-Achse Fortschritt
            uicontrol('Style', 'text', ...
                'String', 'Y-Achse:', ...
                'Position', [xStart-80 yPos+15 70 20], ...
                'HorizontalAlignment', 'right', ...
                'FontWeight', 'bold', ...
                'Tag', 'calibProgressLabel', ...
                'Visible', 'off');
            
            obj.progressBarY = axes('Parent', obj.fig, ...
                'Position', [(xStart)/obj.fig.Position(3) (yPos+15)/obj.fig.Position(4) barWidth/obj.fig.Position(3) barHeight/obj.fig.Position(4)], ...
                'XLim', [0 100], 'YLim', [0 1], ...
                'XTick', [], 'YTick', [], ...
                'Box', 'on', 'Tag', 'calibProgressBar', ...
                'Visible', 'off');
            
            obj.txtProgressY = uicontrol('Style', 'text', ...
                'String', '0%', ...
                'Position', [xStart+barWidth+10 yPos+15 50 20], ...
                'HorizontalAlignment', 'left', ...
                'FontWeight', 'bold', ...
                'Tag', 'calibProgressLabel', ...
                'Visible', 'off');
            
            % Z-Achse Fortschritt
            uicontrol('Style', 'text', ...
                'String', 'Z-Achse:', ...
                'Position', [xStart-80 yPos-15 70 20], ...
                'HorizontalAlignment', 'right', ...
                'FontWeight', 'bold', ...
                'Tag', 'calibProgressLabel', ...
                'Visible', 'off');
            
            obj.progressBarZ = axes('Parent', obj.fig, ...
                'Position', [(xStart)/obj.fig.Position(3) (yPos-15)/obj.fig.Position(4) barWidth/obj.fig.Position(3) barHeight/obj.fig.Position(4)], ...
                'XLim', [0 100], 'YLim', [0 1], ...
                'XTick', [], 'YTick', [], ...
                'Box', 'on', 'Tag', 'calibProgressBar', ...
                'Visible', 'off');
            
            obj.txtProgressZ = uicontrol('Style', 'text', ...
                'String', '0%', ...
                'Position', [xStart+barWidth+10 yPos-15 50 20], ...
                'HorizontalAlignment', 'left', ...
                'FontWeight', 'bold', ...
                'Tag', 'calibProgressLabel', ...
                'Visible', 'off');
        end
        
        function initEKF(obj)
            % Initialisierung Extended Kalman Filter mit Quaternions
            % Zustand: [q0, q1, q2, q3, bias_x, bias_y, bias_z]
            obj.ekf_state = [1; 0; 0; 0; 0; 0; 0];  % Einheitsquaternion + Bias
            obj.ekf_P = eye(7) * 0.1;
        end
        
        function initBuffers(obj)
            obj.timeBuffer = zeros(obj.maxBufferSize, 1);
            obj.accBuffer = zeros(obj.maxBufferSize, 3);
            obj.gyroBuffer = zeros(obj.maxBufferSize, 3);
            obj.magBuffer = zeros(obj.maxBufferSize, 3);

            obj.accFilterBuffer = zeros(obj.maxBufferSize, 3);
            obj.gyroFilterBuffer = zeros(obj.maxBufferSize, 3);
            obj.magFilterBuffer = zeros(obj.maxBufferSize, 3);

            obj.calibData = [];
        end
        
        function connectSerial(obj)
            try
                % Serielle Verbindung öffnen
                obj.serialPort = serialport(obj.portName, obj.baudRate);
                configureTerminator(obj.serialPort, "LF");
                obj.serialPort.Timeout = 1;
                
                obj.isRunning = true;
                set(obj.btnConnect, 'Enable', 'off');
                set(obj.btnStop, 'Enable', 'on');
                set(obj.txtStatus, 'String', 'Verbunden - Datenempfang läuft');
                
                obj.addLogMessage('Serielle Verbindung hergestellt');
                obj.addLogMessage(sprintf('Port: %s, Baudrate: %d', obj.portName, obj.baudRate));
                
                % Datenerfassung starten
                obj.acquireData();
                
            catch e
                obj.addLogMessage(sprintf('FEHLER: Verbindung fehlgeschlagen - %s', e.message));
                set(obj.txtStatus, 'String', 'Verbindung fehlgeschlagen');
            end
        end
        
        function acquireData(obj)
            while obj.isRunning && isvalid(obj.fig)
                try
                    % Prüfen ob Daten verfügbar sind
                    if obj.serialPort.NumBytesAvailable >= obj.dataSize * 4
                        % 19 floats lesen (4 Bytes pro float)
                        data = read(obj.serialPort, obj.dataSize, 'single');
                        
                        % Delimiter prüfen (optional)
                        if abs(data(1) - obj.delimiter) < 1e6
                            obj.processData(data);
                        end
                    end
                    
                    % GUI aktualisieren
                    drawnow;
                    
                catch e
                    disp(['Fehler beim Lesen: ' e.message]);
                end
            end
        end
        
        function processData(obj, data)
            obj.dataCount = obj.dataCount + 1;
            
            % Daten extrahieren
            % Rohwerte
            acc = [data(2); data(3); data(4)];
            gyro = [data(5); data(6); data(7)];
            mag = [data(8); data(9); data(10)];
        
            % Gefilterte Werte
            accFilter = [data(11); data(12); data(13)];
            gyroFilter = [data(14); data(15); data(16)];
            magFilter = [data(17); data(18); data(19)];
            
            % Magnetometer Koordinatensystem anpassen (X und Y invertiert)
            mag(1) = -mag(1);  % X-Achse invertieren
            mag(2) = -mag(2);  % Y-Achse invertieren
            % Z-Achse bleibt unverändert
            
            % Magnetometer kalibrieren
            mag = (mag - obj.magOffset) .* obj.magScale;
            
            % Kalibrierungsdaten sammeln
            if obj.isCalibrating
                obj.calibData = [obj.calibData; mag'];
                
                % Kalibrierungsfortschritt berechnen und aktualisieren
                if mod(obj.dataCount, 5) == 0  % Alle 5 Samples aktualisieren
                    obj.updateCalibrationProgress();
                end
            end
            
            % Daten in Buffer speichern
            idx = mod(obj.dataCount - 1, obj.maxBufferSize) + 1;
            obj.timeBuffer(idx) = obj.dataCount * obj.dt;
            obj.accBuffer(idx, :) = acc';
            obj.gyroBuffer(idx, :) = gyro';
            obj.magBuffer(idx, :) = mag';

            obj.accFilterBuffer(idx, :)  = accFilter';
            obj.gyroFilterBuffer(idx, :) = gyroFilter';
            obj.magFilterBuffer(idx, :)  = magFilter';
            
            % EKF Update
            obj.updateEKF(acc, gyro, mag);
            
            % Plots aktualisieren (alle 10 Samples)
            if mod(obj.dataCount, 10) == 0
                obj.updatePlots();
            end
        end
        
        function updateEKF(obj, acc, gyro, mag)
            % Extended Kalman Filter mit Quaternion-Darstellung
            
            % Eingangsdaten validieren
            if any(isnan(acc)) || any(isnan(gyro)) || any(isnan(mag))
                return; % Überspringe Update bei ungültigen Daten
            end
            
            if norm(acc) < 0.1 || norm(mag) < 0.1
                return; % Zu kleine Werte, wahrscheinlich Sensorfehler
            end
            
            % Aktuellen Zustand extrahieren
            q = obj.ekf_state(1:4);
            bias = obj.ekf_state(5:7);
            
            % Quaternion normalisieren
            q = q / norm(q);
            
            % Gyro bias kompensieren
            gyro_comp = gyro - bias;
            
            % === PRÄDIKTION (Prozessmodell) ===
            dt = obj.dt;
            
            % Quaternion-Ableitung: q_dot = 0.5 * q ⊗ [0; gyro]
            omega = [0; gyro_comp];
            q_dot = 0.5 * obj.quatMultiply(q, omega);
            
            % Euler-Integration
            q_pred = q + q_dot * dt;
            q_pred = q_pred / norm(q_pred);  % Normalisieren
            
            % Prozessrauschen
            Q = diag([0.001, 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001]);
            obj.ekf_P = obj.ekf_P + Q;
            
            % === UPDATE (Messmodell) ===
            
            % 1. Beschleunigungsmessung für Roll und Pitch
            acc_norm = acc / norm(acc);
            
            % Erwartete Gravitationsrichtung im Körpersystem (Rotation von [0;0;1])
            g_pred = obj.quatRotate(q_pred, [0; 0; 1]);
            
            % 2. Magnetfeldmessung für Yaw
            mag_norm = mag / norm(mag);
            
            % Erwartete Magnetfeldrichtung im Körpersystem
            % Annahme: Magnetfeld zeigt in globaler XY-Ebene nach Norden
            mag_ref = [cos(0); sin(0); 0];  % Referenz-Magnetfeld (Nord)
            m_pred = obj.quatRotate(q_pred, mag_ref);
            
            % Innovation (Messfehler)
            z_acc = acc_norm - g_pred;
            z_mag = mag_norm - m_pred;
            
            % NaN-Check
            if any(isnan(z_acc)) || any(isnan(z_mag))
                obj.ekf_state(1:4) = q_pred;
                return;
            end
            
            % Messrauschen
            R_acc = eye(3) * 0.1;
            R_mag = eye(3) * 0.2;
            
            % Jacobi-Matrix der Messgleichung (vereinfacht)
            H_acc = obj.computeAccJacobian(q_pred);
            H_mag = obj.computeMagJacobian(q_pred);
            
            % Kalman Gain für Beschleunigung
            S_acc = H_acc * obj.ekf_P(1:4,1:4) * H_acc' + R_acc;
            K_acc = obj.ekf_P(1:4,1:4) * H_acc' / S_acc;
            
            % Kalman Gain für Magnetometer
            S_mag = H_mag * obj.ekf_P(1:4,1:4) * H_mag' + R_mag;
            K_mag = obj.ekf_P(1:4,1:4) * H_mag' / S_mag;
            
            % NaN-Check für Kalman Gains
            if any(isnan(K_acc(:))) || any(isnan(K_mag(:)))
                obj.ekf_state(1:4) = q_pred;
                return;
            end
            
            % Quaternion-Update (als kleine Rotation)
            delta_q_acc = K_acc * z_acc;
            delta_q_mag = K_mag * z_mag;
            
            % Gewichtete Kombination der Korrekturen
            delta_theta = [delta_q_acc(1:3); 0] * 0.5 + [delta_q_mag(1:3); 0] * 0.3;
            
            % Kleine Rotation als Quaternion
            angle = norm(delta_theta(1:3));
            if angle > 0.001
                delta_q = [cos(angle/2); sin(angle/2) * delta_theta(1:3) / angle];
            else
                delta_q = [1; 0; 0; 0];
            end
            
            % Quaternion-Korrektur anwenden
            q_updated = obj.quatMultiply(q_pred, delta_q);
            q_updated = q_updated / norm(q_updated);
            
            % Pose-Offset anwenden (als Quaternion-Rotation)
            if norm(obj.poseOffset) > 0.001
                q_offset = obj.eulerToQuat(obj.poseOffset);
                q_updated = obj.quatMultiply(obj.quatConjugate(q_offset), q_updated);
            end
            
            % Zustand aktualisieren
            obj.ekf_state(1:4) = q_updated;
            
            % Kovarianz Update (vereinfacht)
            I = eye(7);
            K_combined = zeros(7, 6);
            K_combined(1:4, 1:3) = K_acc(:, 1:3);
            K_combined(1:4, 4:6) = K_mag(:, 1:3);
            H_combined = zeros(6, 7);
            H_combined(1:3, 1:4) = H_acc;
            H_combined(4:6, 1:4) = H_mag;
            
            obj.ekf_P = (I - K_combined * H_combined) * obj.ekf_P;
            
            % Kovarianz stabilisieren
            obj.ekf_P = (obj.ekf_P + obj.ekf_P') / 2;
            
            % Finale NaN-Prüfung
            if any(isnan(obj.ekf_state)) || any(isnan(obj.ekf_P(:)))
                obj.initEKF();
            end
        end
        
        % === HILFSFUNKTIONEN FÜR QUATERNIONS ===
        
        function qout = quatMultiply(~, q1, q2)
            % Quaternion-Multiplikation: q1 ⊗ q2
            if length(q2) == 3
                q2 = [0; q2];  % Vektor zu Quaternion
            end
            
            w1 = q1(1); v1 = q1(2:4);
            w2 = q2(1); v2 = q2(2:4);
            
            w = w1*w2 - dot(v1, v2);
            v = w1*v2 + w2*v1 + cross(v1, v2);
            
            qout = [w; v];
        end
        
        function qc = quatConjugate(~, q)
            % Quaternion-Konjugation
            qc = [q(1); -q(2:4)];
        end
        
        function v_rot = quatRotate(obj, q, v)
            % Rotiere Vektor v mit Quaternion q
            q_v = [0; v];
            q_conj = obj.quatConjugate(q);
            v_rot_q = obj.quatMultiply(obj.quatMultiply(q, q_v), q_conj);
            v_rot = v_rot_q(2:4);
        end
        
        function [roll, pitch, yaw] = quatToEuler(~, q)
            % Quaternion zu Euler-Winkel (ZYX-Konvention)
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
            
            % Roll (X-Achse)
            roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
            
            % Pitch (Y-Achse)
            sinp = 2*(q0*q2 - q3*q1);
            if abs(sinp) >= 1
                pitch = sign(sinp) * pi/2;
            else
                pitch = asin(sinp);
            end
            
            % Yaw (Z-Achse)
            yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
        end
        
        function q = eulerToQuat(~, euler)
            % Euler-Winkel zu Quaternion (ZYX-Konvention)
            roll = euler(1); pitch = euler(2); yaw = euler(3);
            
            cy = cos(yaw * 0.5);
            sy = sin(yaw * 0.5);
            cp = cos(pitch * 0.5);
            sp = sin(pitch * 0.5);
            cr = cos(roll * 0.5);
            sr = sin(roll * 0.5);
            
            q = zeros(4,1);
            q(1) = cr * cp * cy + sr * sp * sy;
            q(2) = sr * cp * cy - cr * sp * sy;
            q(3) = cr * sp * cy + sr * cp * sy;
            q(4) = cr * cp * sy - sr * sp * cy;
        end
        
        function R = quatToRotMat(~, q)
            % Quaternion zu Rotationsmatrix
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
            
            R = [1-2*(q2^2+q3^2),   2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2);
                 2*(q1*q2+q0*q3),   1-2*(q1^2+q3^2),   2*(q2*q3-q0*q1);
                 2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1),   1-2*(q1^2+q2^2)];
        end
        
        function H = computeAccJacobian(obj, q)
            % Jacobi-Matrix für Beschleunigungsmessung (vereinfacht)
            % Ableitung von g_pred nach q
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
            
            H = 2 * [
                -q2,  q3, -q0,  q1;
                 q1,  q0,  q3,  q2;
                 q0, -q1, -q2,  q3
            ];
        end
        
        function H = computeMagJacobian(obj, q)
            % Jacobi-Matrix für Magnetfeldmessung (vereinfacht)
            H = obj.computeAccJacobian(q);  % Gleiche Struktur
        end
        
        function updatePlots(obj)
            % Zeitachse vorbereiten
            validIdx = obj.timeBuffer > 0;
            t = obj.timeBuffer(validIdx);
            
            if isempty(t)
                return;
            end
            
            % Beschleunigungsplot
            set(obj.hAcc(1), 'XData', t, 'YData', obj.accBuffer(validIdx, 1));
            set(obj.hAcc(2), 'XData', t, 'YData', obj.accBuffer(validIdx, 2));
            set(obj.hAcc(3), 'XData', t, 'YData', obj.accBuffer(validIdx, 3));

            set(obj.hAccFilt(1), 'XData', t, 'YData', obj.accFilterBuffer(validIdx,1));
            set(obj.hAccFilt(2), 'XData', t, 'YData', obj.accFilterBuffer(validIdx,2));
            set(obj.hAccFilt(3), 'XData', t, 'YData', obj.accFilterBuffer(validIdx,3));
            
            % Gyro Plot (Umrechnung von rad/s zu °/s)
            set(obj.hGyro(1), 'XData', t, 'YData', rad2deg(obj.gyroBuffer(validIdx, 1)));
            set(obj.hGyro(2), 'XData', t, 'YData', rad2deg(obj.gyroBuffer(validIdx, 2)));
            set(obj.hGyro(3), 'XData', t, 'YData', rad2deg(obj.gyroBuffer(validIdx, 3)));

            set(obj.hGyroFilt(1), 'XData', t, 'YData', rad2deg(obj.gyroFilterBuffer(validIdx,1)));
            set(obj.hGyroFilt(2), 'XData', t, 'YData', rad2deg(obj.gyroFilterBuffer(validIdx,2)));
            set(obj.hGyroFilt(3), 'XData', t, 'YData', rad2deg(obj.gyroFilterBuffer(validIdx,3)));
            
            % Magnetometer Plot
            set(obj.hMag(1), 'XData', t, 'YData', obj.magBuffer(validIdx, 1));
            set(obj.hMag(2), 'XData', t, 'YData', obj.magBuffer(validIdx, 2));
            set(obj.hMag(3), 'XData', t, 'YData', obj.magBuffer(validIdx, 3));

            set(obj.hMagFilt(1), 'XData', t, 'YData', obj.magFilterBuffer(validIdx,1));
            set(obj.hMagFilt(2), 'XData', t, 'YData', obj.magFilterBuffer(validIdx,2));
            set(obj.hMagFilt(3), 'XData', t, 'YData', obj.magFilterBuffer(validIdx,3));

            % 3D Körper zeichnen
            obj.draw3DBody();
        end
        
        function draw3DBody(obj)
            cla(obj.ax3D);
            axes(obj.ax3D);
            
            % Quaternion aus EKF extrahieren
            q = obj.ekf_state(1:4);
            q = q / norm(q);
            
            % Quaternion zu Euler-Winkel für Anzeige
            [roll, pitch, yaw] = obj.quatToEuler(q);
            
            % Rotationsmatrix aus Quaternion
            R = obj.quatToRotMat(q);
            
            % Körper-Koordinatensystem (Quader)
            vertices = [
                -1 -0.5 -0.2; 1 -0.5 -0.2; 1 0.5 -0.2; -1 0.5 -0.2;
                -1 -0.5  0.2; 1 -0.5  0.2; 1 0.5  0.2; -1 0.5  0.2
            ];
            
            % Rotation anwenden
            vertices_rot = (R * vertices')';
            
            % Flächen definieren
            faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
            
            % Quader zeichnen
            patch('Vertices', vertices_rot, 'Faces', faces, ...
                'FaceColor', [0.3 0.5 0.8], 'FaceAlpha', 0.7, 'EdgeColor', 'k');
            
            % Achsen zeichnen
            scale = 1.5;
            axes_body = R * eye(3) * scale;
            hX = quiver3(0, 0, 0, axes_body(1,1), axes_body(2,1), axes_body(3,1), 'r', 'LineWidth', 3);
            hY = quiver3(0, 0, 0, axes_body(1,2), axes_body(2,2), axes_body(3,2), 'g', 'LineWidth', 3);
            hZ = quiver3(0, 0, 0, axes_body(1,3), axes_body(2,3), axes_body(3,3), 'b', 'LineWidth', 3);
            
            % Legende für Achsen (Beschleunigungssensor-Koordinatensystem)
            legend([hX, hY, hZ], {'X-Achse (Acc)', 'Y-Achse (Acc)', 'Z-Achse (Acc)'}, ...
                'Location', 'northeast', 'FontSize', 9);
            
            % Winkel anzeigen
            title(sprintf('Roll: %.1f° | Pitch: %.1f° | Yaw: %.1f°', ...
                rad2deg(roll), rad2deg(pitch), rad2deg(yaw)));
            
            axis equal;
            xlim([-2 2]); ylim([-2 2]); zlim([-2 2]);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            grid on;
            view(45, 30);
        end
        
        function startCalibration(obj)
            obj.isCalibrating = true;
            obj.calibData = [];
            obj.calibCoverage = [0; 0; 0];
            
            set(obj.txtInstructions, 'Visible', 'on');
            set(obj.txtInstructions, 'String', sprintf([...
                'MAGNETOMETER KALIBRIERUNG GESTARTET\n\n' ...
                'Anweisungen:\n' ...
                '1. Drehen Sie das Board langsam um alle 3 Achsen\n' ...
                '2. Führen Sie mindestens 3 vollständige Rotationen um jede Achse durch\n' ...
                '3. Bewegen Sie das Board dabei so, dass es alle Orientierungen einnimmt\n' ...
                '4. Halten Sie das Board von magnetischen Störquellen fern\n' ...
                '5. Beobachten Sie die Fortschrittsbalken - alle sollten 100%% erreichen\n' ...
                '6. Die Kalibrierung dauert ca. 30-60 Sekunden\n\n' ...
                'Klicken Sie erneut auf den Button um die Kalibrierung abzuschließen']));
            
            set(obj.btnCalibrate, 'String', 'Kalibrierung beenden', ...
                'BackgroundColor', [1.0 0.5 0.3]);
            
            % Fortschrittsbalken und Labels sichtbar machen
            obj.showCalibrationProgress(true);
            
            obj.addLogMessage('Magnetometer-Kalibrierung gestartet');
            obj.addLogMessage('Bitte Board um alle Achsen drehen...');
            
            % Timer für automatisches Beenden nach 60 Sekunden
            t = timer('TimerFcn', @(~,~)obj.finishCalibration(), ...
                'StartDelay', 60, 'ExecutionMode', 'singleShot');
            start(t);
        end
        
        function finishCalibration(obj)
            obj.isCalibrating = false;
            
            % Fortschrittsbalken ausblenden
            obj.showCalibrationProgress(false);
            
            if size(obj.calibData, 1) > 100
                % Ellipsoid-Fitting für Hard-Iron und Soft-Iron Kompensation
                % Vereinfacht: Offset und Skalierung berechnen
                obj.magOffset = mean(obj.calibData, 1)';
                
                centered = obj.calibData - obj.magOffset';
                obj.magScale = 1 ./ std(centered, 0, 1)';
                obj.magScale = obj.magScale / max(obj.magScale);
                
                set(obj.txtStatus, 'String', sprintf(...
                    'Kalibrierung erfolgreich! Offset: [%.1f, %.1f, %.1f]', ...
                    obj.magOffset(1), obj.magOffset(2), obj.magOffset(3)));
                
                obj.addLogMessage(sprintf('Kalibrierung erfolgreich abgeschlossen (%d Datenpunkte)', size(obj.calibData, 1)));
                obj.addLogMessage(sprintf('Mag Offset: [%.2f, %.2f, %.2f]', obj.magOffset(1), obj.magOffset(2), obj.magOffset(3)));
                obj.addLogMessage(sprintf('Mag Scale: [%.3f, %.3f, %.3f]', obj.magScale(1), obj.magScale(2), obj.magScale(3)));
                obj.addLogMessage(sprintf('Abdeckung: X=%.0f%%, Y=%.0f%%, Z=%.0f%%', ...
                    obj.calibCoverage(1), obj.calibCoverage(2), obj.calibCoverage(3)));
            else
                obj.addLogMessage('WARNUNG: Nicht genügend Daten gesammelt - Kalibrierung wiederholen!');
            end
            
            set(obj.txtInstructions, 'Visible', 'off');
            set(obj.btnCalibrate, 'String', 'Magnetometer kalibrieren', ...
                'BackgroundColor', [0.3 0.7 1.0]);
        end
        
        function saveCalibration(obj)
            % Kalibrierungsdaten speichern
            magOffset = obj.magOffset;
            magScale = obj.magScale;
            poseOffset = obj.poseOffset;
            calibDate = datestr(now);
            
            try
                save(obj.calibFileName, 'magOffset', 'magScale', 'poseOffset', 'calibDate');
                set(obj.txtStatus, 'String', sprintf('Kalibrierung gespeichert: %s', obj.calibFileName));
                obj.addLogMessage(sprintf('Kalibrierungsdaten gespeichert in: %s', obj.calibFileName));
            catch e
                obj.addLogMessage(sprintf('FEHLER beim Speichern: %s', e.message));
            end
        end
        
        function loadCalibration(obj)
            % Kalibrierungsdaten laden
            if exist(obj.calibFileName, 'file')
                try
                    data = load(obj.calibFileName);
                    obj.magOffset = data.magOffset;
                    obj.magScale = data.magScale;
                    
                    if isfield(data, 'poseOffset')
                        obj.poseOffset = data.poseOffset;
                    end
                    
                    calibDate = '';
                    if isfield(data, 'calibDate')
                        calibDate = data.calibDate;
                    end
                    
                    set(obj.txtStatus, 'String', sprintf('Kalibrierung geladen vom: %s', calibDate));
                    obj.addLogMessage(sprintf('Kalibrierung geladen (Datum: %s)', calibDate));
                    obj.addLogMessage(sprintf('Mag Offset: [%.2f, %.2f, %.2f]', obj.magOffset(1), obj.magOffset(2), obj.magOffset(3)));
                    obj.addLogMessage(sprintf('Mag Scale: [%.3f, %.3f, %.3f]', obj.magScale(1), obj.magScale(2), obj.magScale(3)));
                catch e
                    obj.addLogMessage(sprintf('FEHLER beim Laden: %s', e.message));
                end
            else
                obj.addLogMessage(sprintf('Keine Kalibrierungsdatei gefunden: %s', obj.calibFileName));
            end
        end
        
        function resetPose(obj)
            % Aktuelle Pose als Nullpunkt setzen
            if obj.dataCount > 0
                % Aktuelles Quaternion als Offset speichern (als Euler für Kompatibilität)
                q = obj.ekf_state(1:4);
                [roll, pitch, yaw] = obj.quatToEuler(q);
                
                obj.poseOffset(1) = roll;
                obj.poseOffset(2) = pitch;
                obj.poseOffset(3) = yaw;
                
                set(obj.txtStatus, 'String', sprintf(...
                    'Pose auf 0° gesetzt | Offset: R=%.1f° P=%.1f° Y=%.1f°', ...
                    rad2deg(obj.poseOffset(1)), rad2deg(obj.poseOffset(2)), rad2deg(obj.poseOffset(3))));
                
                obj.addLogMessage('Pose auf 0° zurückgesetzt');
                obj.addLogMessage(sprintf('Roll Offset: %.1f°', rad2deg(obj.poseOffset(1))));
                obj.addLogMessage(sprintf('Pitch Offset: %.1f°', rad2deg(obj.poseOffset(2))));
                obj.addLogMessage(sprintf('Yaw Offset: %.1f°', rad2deg(obj.poseOffset(3))));
            else
                obj.addLogMessage('WARNUNG: Keine Sensordaten verfügbar - warten Sie auf Datenempfang');
            end
        end
        
        function stopAcquisition(obj)
            obj.isRunning = false;
            
            if ~isempty(obj.serialPort)
                delete(obj.serialPort);
                obj.serialPort = [];
            end
            
            set(obj.btnConnect, 'Enable', 'on');
            set(obj.btnStop, 'Enable', 'off');
            set(obj.txtStatus, 'String', 'Verbindung getrennt');
            obj.addLogMessage('Verbindung getrennt');
        end
        
        function addLogMessage(obj, message)
            % Nachricht mit Zeitstempel zum Log hinzufügen
            timestamp = datestr(now, 'HH:MM:SS.FFF');
            logEntry = sprintf('[%s] %s', timestamp, message);
            
            % Aktuelle Log-Einträge abrufen
            currentLog = get(obj.txtLog, 'String');
            
            % Neuen Eintrag hinzufügen
            if isempty(currentLog)
                newLog = {logEntry};
            else
                newLog = [currentLog; {logEntry}];
            end
            
            % Log auf maximal 500 Einträge begrenzen
            if length(newLog) > 500
                newLog = newLog(end-499:end);
            end
            
            % Log aktualisieren
            set(obj.txtLog, 'String', newLog);
            
            % Automatisch zum neuesten Eintrag scrollen
            set(obj.txtLog, 'Value', length(newLog));
        end
        
        function updateCalibrationProgress(obj)
            % Berechne Abdeckung der Kalibrierungsdaten für jede Achse
            if size(obj.calibData, 1) < 10
                return;
            end
            
            % Für jede Achse: Berechne Min/Max und Spannweite
            for axis = 1:3
                data = obj.calibData(:, axis);
                minVal = min(data);
                maxVal = max(data);
                range = maxVal - minVal;
                
                % Erwartete Spannweite (basierend auf typischen Magnetfeldstärken)
                % Typisch: ±50 µT pro Achse, also ~100 µT Spannweite
                expectedRange = 100;
                
                % Fortschritt als Prozentsatz (min 0%, max 100%)
                coverage = min(100, (range / expectedRange) * 100);
                obj.calibCoverage(axis) = coverage;
            end
            
            % Fortschrittsbalken aktualisieren
            obj.drawProgressBar(obj.progressBarX, obj.calibCoverage(1), obj.txtProgressX);
            obj.drawProgressBar(obj.progressBarY, obj.calibCoverage(2), obj.txtProgressY);
            obj.drawProgressBar(obj.progressBarZ, obj.calibCoverage(3), obj.txtProgressZ);
        end
        
        function drawProgressBar(~, axHandle, percentage, txtHandle)
            % Zeichne Fortschrittsbalken
            cla(axHandle);
            axes(axHandle);
            
            % Hintergrund (grau)
            rectangle('Position', [0 0 100 1], 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none');
            
            % Fortschritt (grün bis rot je nach Wert)
            if percentage < 50
                color = [1 0.5 0];  % Orange
            elseif percentage < 80
                color = [1 1 0];    % Gelb
            else
                color = [0 0.8 0];  % Grün
            end
            
            rectangle('Position', [0 0 percentage 1], 'FaceColor', color, 'EdgeColor', 'none');
            
            % Text aktualisieren
            set(txtHandle, 'String', sprintf('%.0f%%', percentage));
            
            % Farbe des Textes anpassen
            if percentage >= 100
                set(txtHandle, 'ForegroundColor', [0 0.6 0], 'FontWeight', 'bold');
            else
                set(txtHandle, 'ForegroundColor', [0 0 0], 'FontWeight', 'normal');
            end
        end
        
        function showCalibrationProgress(obj, visible)
            % Zeige oder verstecke alle Kalibrierungs-UI-Elemente
            if visible
                visStr = 'on';
                % Fortschrittsbalken zurücksetzen
                obj.calibCoverage = [0; 0; 0];
                obj.drawProgressBar(obj.progressBarX, 0, obj.txtProgressX);
                obj.drawProgressBar(obj.progressBarY, 0, obj.txtProgressY);
                obj.drawProgressBar(obj.progressBarZ, 0, obj.txtProgressZ);
            else
                visStr = 'off';
            end
            
            % Alle Elemente mit Tag 'calibProgressBar' oder 'calibProgressLabel'
            set(obj.progressBarX, 'Visible', visStr);
            set(obj.progressBarY, 'Visible', visStr);
            set(obj.progressBarZ, 'Visible', visStr);
            set(obj.txtProgressX, 'Visible', visStr);
            set(obj.txtProgressY, 'Visible', visStr);
            set(obj.txtProgressZ, 'Visible', visStr);
            
            % Alle Labels
            labels = findall(obj.fig, 'Tag', 'calibProgressLabel');
            set(labels, 'Visible', visStr);
        end
        
        function closeApp(obj)
            obj.stopAcquisition();
            delete(obj.fig);
        end
    end
end