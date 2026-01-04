classdef MATLAB_Skript_Broesamle_Lucas < handle
    properties
        % Serielle Verbindung
        serialPort
        portName = 'COM4'
        baudRate = 115200
        
        % GUI Elemente
        fig
        axAcc
        axGyro
        axMag
        hAccFilter
        hGyroFilter
        hMagFilter
        txtLog
        txtStatus
        btnConnect
        btnStop
        
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
        
        % Laufzeitvariablen
        isRunning = false
        dataCount = 0
        dt = 0.1
        dataSize = 19  % 1 Delimiter + 18 Werte
        delimiter = 0xDEADBEEF
    end
    
    methods
        function obj = MATLAB_Skript_Broesamle_Lucas()
            obj.initGUI();
            obj.initBuffers();
        end
        
        function initGUI(obj)
            obj.fig = figure('Name', 'IMU Visualisierung', 'NumberTitle', 'off', 'Position', [50 50 1600 900], 'CloseRequestFcn', @(~,~)obj.closeApp());
            
            % Beschleunigungsplot
            obj.axAcc = subplot(2,3,1);
            title('Beschleunigung [m/s²]');
            xlabel('Zeit [s]'); ylabel('Beschleunigung');
            hold on; grid on;
            obj.hAcc = plot(0, [0 0 0]);
            obj.hAccFilter = plot(0, [0 0 0], '--');
            legend('X','Y','Z','X filter','Y filter','Z filter');
            
            % Gyro Plot
            obj.axGyro = subplot(2,3,2);
            title('Winkelgeschwindigkeit [°/s]');
            xlabel('Zeit [s]'); ylabel('Winkelgeschwindigkeit');
            hold on; grid on;
            obj.hGyro = plot(0, [0 0 0]);
            obj.hGyroFilter = plot(0, [0 0 0], '--');
            legend('X','Y','Z','X filter','Y filter','Z filter');
            
            % Magnetometer Plot
            obj.axMag = subplot(2,3,3);
            title('Magnetfeld [µT]');
            xlabel('Zeit [s]'); ylabel('Magnetfeld');
            hold on; grid on;
            obj.hMag = plot(0, [0 0 0]);
            obj.hMagFilter = plot(0, [0 0 0], '--');
            legend('X','Y','Z','X filter','Y filter','Z filter');
            
            % Log-Feld
            obj.txtLog = uicontrol('Style','listbox', 'String',{}, 'Position',[150 150 1300 250], 'FontName','Courier New','FontSize',8,'Max',2,'Min',0);
            
            % Status
            obj.txtStatus = uicontrol('Style','text', 'String','Bereit zum Verbinden', 'Position',[250 50 300 30], 'HorizontalAlignment','left','FontSize',10);
            
            % Buttons
            obj.btnConnect = uicontrol('Style','pushbutton', 'String','Verbinden','Position',[20 50 100 40], 'Callback',@(~,~)obj.connectSerial());
            obj.btnStop = uicontrol('Style','pushbutton', 'String','Stop','Position',[130 50 100 40], 'Enable','off','Callback',@(~,~)obj.stopAcquisition());
        end
        
        function initBuffers(obj)
            obj.timeBuffer = zeros(obj.maxBufferSize,1);
            obj.accBuffer = zeros(obj.maxBufferSize,3);
            obj.gyroBuffer = zeros(obj.maxBufferSize,3);
            obj.magBuffer = zeros(obj.maxBufferSize,3);

            obj.accFilterBuffer = zeros(obj.maxBufferSize,3);
            obj.gyroFilterBuffer = zeros(obj.maxBufferSize,3);
            obj.magFilterBuffer = zeros(obj.maxBufferSize,3);
        end
        
        function connectSerial(obj)
            try
                obj.serialPort = serialport(obj.portName, obj.baudRate);
                configureTerminator(obj.serialPort,"LF");
                obj.serialPort.Timeout = 1;
                
                obj.isRunning = true;
                set(obj.btnConnect,'Enable','off');
                set(obj.btnStop,'Enable','on');
                set(obj.txtStatus,'String','Verbunden - Datenempfang läuft');
                obj.addLogMessage('Serielle Verbindung hergestellt');
                
                obj.acquireData();
            catch e
                obj.addLogMessage(['FEHLER: ' e.message]);
                set(obj.txtStatus,'String','Verbindung fehlgeschlagen');
            end
        end
        
        function acquireData(obj)
            while obj.isRunning && isvalid(obj.fig)
                try
                    if obj.serialPort.NumBytesAvailable >= obj.dataSize*4
                        data = read(obj.serialPort,obj.dataSize,'single');
                        if abs(data(1)-obj.delimiter)<1e6
                            obj.processData(data);
                        end
                    end
                    drawnow;
                catch e
                    obj.addLogMessage(['Fehler beim Lesen: ' e.message]);
                end
            end
        end
        
        function processData(obj, data)
            obj.dataCount = obj.dataCount + 1;
            
        % Rohwerte
        acc = [data(2); data(3); data(4)];
        gyro = [data(5); data(6); data(7)];
        mag = [data(14); data(15); data(16)];
    
        % Gefilterte Werte
        accFilter = [data(8); data(9); data(10)];
        gyroFilter = [data(11); data(12); data(13)];
        magFilter = [data(17); data(18); data(19)];
            
            idx = mod(obj.dataCount-1,obj.maxBufferSize)+1;
            obj.timeBuffer(idx) = obj.dataCount*obj.dt;

            % speichern
            obj.accBuffer(idx,:) = acc';
            obj.gyroBuffer(idx,:) = gyro';
            obj.magBuffer(idx,:) = mag';
            
            obj.accFilterBuffer(idx,:) = accFilter';
            obj.gyroFilterBuffer(idx,:) = gyroFilter';
            obj.magFilterBuffer(idx,:) = magFilter';
            
            if mod(obj.dataCount,10)==0
                obj.updatePlots();
            end
        end
        
        function updatePlots(obj)
            validIdx = obj.timeBuffer>0;
            t = obj.timeBuffer(validIdx);
            if isempty(t), return; end
            
            % Beschleunigung
            set(obj.hAcc(1),'XData',t,'YData',obj.accBuffer(validIdx,1));
            set(obj.hAcc(2),'XData',t,'YData',obj.accBuffer(validIdx,2));
            set(obj.hAcc(3),'XData',t,'YData',obj.accBuffer(validIdx,3));
            
            % Gyro in °/s
            set(obj.hGyro(1),'XData',t,'YData',rad2deg(obj.gyroBuffer(validIdx,1)));
            set(obj.hGyro(2),'XData',t,'YData',rad2deg(obj.gyroBuffer(validIdx,2)));
            set(obj.hGyro(3),'XData',t,'YData',rad2deg(obj.gyroBuffer(validIdx,3)));
            
            % Magnetometer
            set(obj.hMag(1),'XData',t,'YData',obj.magBuffer(validIdx,1));
            set(obj.hMag(2),'XData',t,'YData',obj.magBuffer(validIdx,2));
            set(obj.hMag(3),'XData',t,'YData',obj.magBuffer(validIdx,3));

            % Beschleunigung gefiltert
            set(obj.hAccFilter(1),'XData',t,'YData',obj.accFilterBuffer(validIdx,1));
            set(obj.hAccFilter(2),'XData',t,'YData',obj.accFilterBuffer(validIdx,2));
            set(obj.hAccFilter(3),'XData',t,'YData',obj.accFilterBuffer(validIdx,3));
            
            % Gyro gefiltert
            set(obj.hGyroFilter(1),'XData',t,'YData',rad2deg(obj.gyroFilterBuffer(validIdx,1)));
            set(obj.hGyroFilter(2),'XData',t,'YData',rad2deg(obj.gyroFilterBuffer(validIdx,2)));
            set(obj.hGyroFilter(3),'XData',t,'YData',rad2deg(obj.gyroFilterBuffer(validIdx,3)));
            
            % Magnetometer gefiltert
            set(obj.hMagFilter(1),'XData',t,'YData',obj.magFilterBuffer(validIdx,1));
            set(obj.hMagFilter(2),'XData',t,'YData',obj.magFilterBuffer(validIdx,2));
            set(obj.hMagFilter(3),'XData',t,'YData',obj.magFilterBuffer(validIdx,3));
        end
        
        function addLogMessage(obj,message)
            timestamp = datestr(now,'HH:MM:SS.FFF');
            logEntry = sprintf('[%s] %s',timestamp,message);
            currentLog = get(obj.txtLog,'String');
            set(obj.txtLog,'String',[currentLog; {logEntry}]);
            drawnow;
        end
        
        function stopAcquisition(obj)
            obj.isRunning = false;
            if ~isempty(obj.serialPort)
                delete(obj.serialPort);
                obj.serialPort = [];
            end
            set(obj.btnConnect,'Enable','on');
            set(obj.btnStop,'Enable','off');
            set(obj.txtStatus,'String','Datenempfang gestoppt');
            obj.addLogMessage('Datenempfang gestoppt');
        end
        
        function closeApp(obj)
            obj.stopAcquisition();
            delete(obj.fig);
        end
    end
end