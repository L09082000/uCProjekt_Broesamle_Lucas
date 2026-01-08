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
        btnConnect
        btnStop
        txtStatus
        txtLog  % Neues Log-Textfeld

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

        % Plot Handles
        hAcc
        hGyro
        hMag
        hAccFilt
        hGyroFilt
        hMagFilt

        
        % Laufzeitvariablen
        isRunning = false
        dataCount = 0
        dt = 0.1   % 100ms Abtastrate
    end
    
    methods
        function obj = MATLAB_Skript_Broesamle_Lucas()
            obj.initGUI();
            obj.initBuffers();
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
            obj.hAcc = plot(obj.axAcc, 0, zeros(1,3), 'LineWidth', 1.5);
            obj.hAccFilt = plot(obj.axAcc, 0, zeros(1,3), '--', 'LineWidth', 1.2);
            legend('X roh','Y roh','Z roh','X gef.','Y gef.','Z gef.');
            
            % Gyroskop Plot (oben mitte)
            obj.axGyro = subplot(2,3,2);
            title('Winkelgeschwindigkeit [°/s]');
            xlabel('Zeit [s]');
            ylabel('Winkelgeschwindigkeit');
            hold on; grid on;
            obj.hGyro = plot(obj.axGyro, 0, zeros(1,3), 'LineWidth', 1.5);
            obj.hGyroFilt = plot(obj.axGyro, 0, zeros(1,3), '--', 'LineWidth', 1.2);
            legend('X roh','Y roh','Z roh','X gef.','Y gef.','Z gef.');
            
            % Magnetometer Plot (oben rechts)
            obj.axMag = subplot(2,3,3);
            title('Magnetfeld [µT]');
            xlabel('Zeit [s]');
            ylabel('Magnetfeld');
            hold on; grid on;
            obj.hMag = plot(obj.axMag, 0, zeros(1,3), 'LineWidth', 1.5);
            obj.hMagFilt = plot(obj.axMag, 0, zeros(1,3), '--', 'LineWidth', 1.2);
            legend('X roh','Y roh','Z roh','X gef.','Y gef.','Z gef.');
            
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
            
            % Statustext
            obj.txtStatus = uicontrol('Style', 'text', ...
                'String', 'Bereit zum Verbinden', ...
                'Position', [850 20 740 40], ...
                'HorizontalAlignment', 'left', ...
                'FontSize', 10);
        end
        
        function initBuffers(obj)
            obj.timeBuffer = zeros(obj.maxBufferSize, 1);
            obj.accBuffer = zeros(obj.maxBufferSize, 3);
            obj.gyroBuffer = zeros(obj.maxBufferSize, 3);
            obj.magBuffer = zeros(obj.maxBufferSize, 3);

            obj.accFilterBuffer = zeros(obj.maxBufferSize, 3);
            obj.gyroFilterBuffer = zeros(obj.maxBufferSize, 3);
            obj.magFilterBuffer = zeros(obj.maxBufferSize, 3);
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
            
            % Daten in Buffer speichern
            idx = mod(obj.dataCount - 1, obj.maxBufferSize) + 1;
            obj.timeBuffer(idx) = obj.dataCount * obj.dt;
            obj.accBuffer(idx, :) = acc';
            obj.gyroBuffer(idx, :) = gyro';
            obj.magBuffer(idx, :) = mag';

            obj.accFilterBuffer(idx, :)  = accFilter';
            obj.gyroFilterBuffer(idx, :) = gyroFilter';
            obj.magFilterBuffer(idx, :)  = magFilter';
            
            % Plots aktualisieren (alle 10 Samples)
            if mod(obj.dataCount, 10) == 0
                obj.updatePlots();
            end
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

        function closeApp(obj)
            obj.stopAcquisition();
            delete(obj.fig);
        end
    end
end