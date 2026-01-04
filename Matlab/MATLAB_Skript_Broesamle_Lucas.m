function STM32_LivePlot
%% =========================================================
% Live Plot STM32 Sensor Data (LSM6DSL + LIS3MDL)
% Autor: Lucas Brösamle
% Abtastrate: 100 Hz
% =========================================================

%% ----------------- Serielle Schnittstelle ----------------
port = "COM4";
baudrate = 115200;
serialPort = [];

%% ----------------- Protokollparameter -------------------
DELIMITER = single(123456.0);   
NUM_FLOATS = 1 + 6 + 6 + 3 + 3;  
PACKET_BYTES = NUM_FLOATS * 4;   

%% ----------------- Speicher für Daten -------------------
fs = 100;  
t = [];
sample_idx = 0;

acc_raw  = [];
acc_filt = [];
gyro_raw  = [];
gyro_filt = [];
mag_raw  = [];
mag_filt = [];

isRunning = false;

%% ----------------- GUI erstellen ------------------------
fig = figure('Name','STM32 Sensor Live Plot','NumberTitle','off',...
             'Position',[50 50 1600 900]);

% Subplots
axAcc  = subplot(2,3,1); hold(axAcc,'on'); grid(axAcc,'on');
title(axAcc,'Beschleunigung [m/s²]'); xlabel(axAcc,'Zeit [s]'); ylabel(axAcc,'Beschleunigung');
axGyro = subplot(2,3,2); hold(axGyro,'on'); grid(axGyro,'on');
title(axGyro,'Winkelgeschwindigkeit [°/s]'); xlabel(axGyro,'Zeit [s]'); ylabel(axGyro,'Winkelgeschwindigkeit');
axMag  = subplot(2,3,3); hold(axMag,'on'); grid(axMag,'on');
title(axMag,'Magnetfeld [µT]'); xlabel(axMag,'Zeit [s]'); ylabel(axMag,'Magnetfeld');

colors = ['b','r','g'];
hAcc = gobjects(6,1); hGyro = gobjects(6,1); hMag = gobjects(6,1);

for i=1:3
    hAcc(i)   = plot(axAcc, nan, nan, colors(i));
    hAcc(i+3) = plot(axAcc, nan, nan, colors(i), 'LineWidth', 1.5);
    hGyro(i)   = plot(axGyro, nan, nan, colors(i));
    hGyro(i+3) = plot(axGyro, nan, nan, colors(i), 'LineWidth', 1.5);
    hMag(i)   = plot(axMag, nan, nan, colors(i));
    hMag(i+3) = plot(axMag, nan, nan, colors(i), 'LineWidth', 1.5);
end

legend(axAcc, {'Acc X raw','Acc Y raw','Acc Z raw','Acc X filt','Acc Y filt','Acc Z filt'});
legend(axGyro, {'Gyro X raw','Gyro Y raw','Gyro Z raw','Gyro X filt','Gyro Y filt','Gyro Z filt'});
legend(axMag, {'Mag X raw','Mag Y raw','Mag Z raw','Mag X filt','Mag Y filt','Mag Z filt'});

% Log-Feld
txtLog = uicontrol('Style','listbox','Position',[150 150 1300 250],'String',{});
txtStatus = uicontrol('Style','text','Position',[250 50 300 30],'String','Status: nicht verbunden');

% Buttons
btnConnect = uicontrol('Style','pushbutton','String','Verbinden',...
                       'Position',[20 50 100 40]);
btnDisconnect = uicontrol('Style','pushbutton','String','Trennen',...
                          'Position',[130 50 100 40],...
                          'Enable','off');

% Callback-Funktionen
btnConnect.Callback = @(~,~) connectSerial();
btnDisconnect.Callback = @(~,~) stopAcquisition();

%% ----------------- Nested Functions --------------------
    function addLogMessage(message)
        timestamp = datestr(now, 'HH:MM:SS.FFF');
        logEntry = sprintf('[%s] %s', timestamp, message);
        currentLog = get(txtLog,'String');
        if isempty(currentLog)
            newLog = {logEntry};
        else
            newLog = [currentLog; {logEntry}];
        end
        if length(newLog) > 500
            newLog = newLog(end-499:end);
        end
        set(txtLog,'String',newLog);
        set(txtLog,'Value',length(newLog));
        disp(logEntry);
    end

    function connectSerial()
        try
            serialPort = serialport(port, baudrate);
            configureTerminator(serialPort, "LF");
            serialPort.Timeout = 1;

            isRunning = true;
            set(btnConnect,'Enable','off');
            set(btnDisconnect,'Enable','on');
            set(txtStatus,'String','Verbunden - Datenempfang läuft');

            addLogMessage('Serielle Verbindung hergestellt');
            addLogMessage(sprintf('Port: %s, Baudrate: %d', port, baudrate));

            acquireData();
        catch e
            addLogMessage(sprintf('FEHLER: Verbindung fehlgeschlagen - %s', e.message));
            set(txtStatus,'String','Verbindung fehlgeschlagen');
        end
    end

    function stopAcquisition()
        isRunning = false;
        if ~isempty(serialPort)
            delete(serialPort);
            serialPort = [];
        end
        set(btnConnect,'Enable','on');
        set(btnDisconnect,'Enable','off');
        set(txtStatus,'String','Verbindung getrennt');
        addLogMessage('Verbindung getrennt');
    end

    function acquireData()
        while isRunning
            if serialPort.NumBytesAvailable < PACKET_BYTES
                pause(0.001);
                continue;
            end

            raw_bytes = read(serialPort, PACKET_BYTES, "uint8");
            data = typecast(uint8(raw_bytes),'single');

            if data(1) ~= DELIMITER
                continue;
            end

            acc_r   = data(2:4);
            gyro_r  = data(5:7);
            acc_f   = data(8:10);
            gyro_f  = data(11:13);
            mag_r   = data(14:16);
            mag_f   = data(17:19);

            sample_idx = sample_idx + 1;
            t(sample_idx) = sample_idx / fs;

            acc_raw(:,sample_idx) = acc_r;
            acc_filt(:,sample_idx) = acc_f;
            gyro_raw(:,sample_idx) = gyro_r;
            gyro_filt(:,sample_idx) = gyro_f;
            mag_raw(:,sample_idx) = mag_r;
            mag_filt(:,sample_idx) = mag_f;

            for i=1:3
                set(hAcc(i),'XData',t,'YData',acc_raw(i,:));
                set(hAcc(i+3),'XData',t,'YData',acc_filt(i,:));
                set(hGyro(i),'XData',t,'YData',gyro_raw(i,:));
                set(hGyro(i+3),'XData',t,'YData',gyro_filt(i,:));
                set(hMag(i),'XData',t,'YData',mag_raw(i,:));
                set(hMag(i+3),'XData',t,'YData',mag_filt(i,:));
            end
            drawnow limitrate
        end
    end
end