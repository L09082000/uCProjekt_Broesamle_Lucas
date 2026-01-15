classdef MATLAB_Skript_Broesamle_Lucas < handle
    properties
        % Serielle Verbindung
        serialPort
        portName = 'COM4'  % Anpassen an persönlichen Port
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
        delimiter  = uint32(3735928559);   % oder hex2dec('DEADBEEF')
        dataFloats = 18;
        frameBytes = 76;  % 4 + 18*4
        
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

        % Statistik
        receivedBytes = 0;       % insgesamt empfangene Bytes
        txtByteCount   % UI-Feld für die Byte-Anzeige
    end
    
    methods
        function obj = MATLAB_Skript_Broesamle_Lucas()
            % Alle vorherigen Fenster und Daten löschen
            close all;
            clc;

            obj.initGUI();
            obj.initBuffers();

            % Initiale Log-Nachrichten
            obj.addLogMessage('Messdaten-Visualisierung gestartet');
            obj.addLogMessage('COM-Port aktuell: "COM4" – bei Bedarf anpassen');
            obj.addLogMessage('Klicken Sie auf "Verbinden" um Datenerfassung zu starten');
        end
        
        function initGUI(obj)
            % Hauptfenster erstellen
            obj.fig = figure('Name', 'Messdaten-Visualisierung', ...
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
                'Position', [150 120 1300 300], ...
                'FontName', 'Courier New', ...
                'FontSize', 8, ...
                'HorizontalAlignment', 'left', ...
                'Max', 2, ...
                'Min', 0);
            
            uicontrol('Style', 'text', ...
                'String', 'System-Log:', ...
                'Position', [150 430 100 20], ...
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
                'Position', [280 20 740 40], ...
                'HorizontalAlignment', 'left', ...
                'FontSize', 10);

            % Byte-Zähler Textfeld (rechts oben / neben Status)
            obj.txtByteCount = uicontrol('Style', 'text', ...
                'String', 'Empfangene Bytes: 0', ...
                'Position', [280 0 740 40], ...
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
                obj.serialPort.Timeout = 1;

                % Byte-Counter zurücksetzen
                obj.dataCount     = 0;
                obj.receivedBytes = 0;

                % Log hinzufügen: Counter zurückgesetzt
                obj.addLogMessage('Byte-Counter auf 0 gesetzt');
                
                % UI initialisieren
                set(obj.txtByteCount, 'String', 'Empfangene Bytes: 0');
                
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

        function frame = readFrame(obj)
            frame = [];
        
            while obj.serialPort.NumBytesAvailable >= obj.frameBytes
        
                % 1) 4 Byte Header lesen
                hdr = read(obj.serialPort, 4, 'uint8');
                hdrVal = typecast(uint8(hdr), 'uint32');

                % Byte-Counter
                obj.receivedBytes = obj.receivedBytes + obj.frameBytes;
                
                % Direkt UI aktualisieren
                if isvalid(obj.txtByteCount)
                    set(obj.txtByteCount, 'String', sprintf('Empfangene Bytes: %d', obj.receivedBytes));
                end
        
                if hdrVal == obj.delimiter
                    % 2) Payload lesen (18 floats)
                    payload = read(obj.serialPort, obj.dataFloats*4, 'uint8');
                    frame = typecast(uint8(payload), 'single');
                    return;
                else
                    % 3) Resynchronisation
                    read(obj.serialPort, 1, 'uint8');
                end
            end
        end

        function acquireData(obj)
            while obj.isRunning && isvalid(obj.fig)
                try
                    frame = obj.readFrame();
        
                    if ~isempty(frame)
                        obj.processData(frame);
                    end
        
                    drawnow;
        
                catch e
                    disp(['Fehler beim Lesen: ' e.message]);
                end
            end
        end
        
        function processData(obj, frame)
            obj.dataCount = obj.dataCount + 1;
            
            % Daten extrahieren
            % Rohwerte
            acc  = frame(1:3);
            gyro = frame(4:6);
            mag  = frame(7:9);
        
            % Gefilterte Werte
            accFilter  = frame(10:12);
            gyroFilter = frame(13:15);
            magFilter  = frame(16:18);
            
            % Anpassung des Magnetometer-Koordinatensystems (X und Y invertiert)
            % Hintergrund:
            % Das Magnetometer (z. B. LIS3MDL) besitzt aufgrund seiner internen
            % Achsdefinition bzw. der mechanischen Montage auf dem PCB ein gegenüber
            % dem Fahrzeug- bzw. IMU-Koordinatensystem um 180° gedrehtes X-Y-System.
            % Durch die Invertierung der X- und Y-Achse wird eine Rotation um die
            % Z-Achse um 180° realisiert, sodass das Magnetometer-Koordinatensystem
            % mit dem Koordinatensystem von Beschleunigungs- und Drehratensensor
            % übereinstimmt.
            mag(1) = -mag(1);  % X-Achse invertieren
            mag(2) = -mag(2);  % Y-Achse invertieren
            magFilter(1) = -magFilter(1);
            magFilter(2) = -magFilter(2);
            
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