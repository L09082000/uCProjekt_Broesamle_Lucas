function STM32_GUI_Layout_Fixed
%% =========================================================
% GUI Layout für STM32 Live Plot (ohne Funktionalität)
% Plots oben, Log und Buttons unten
% =========================================================

%% ----------------- GUI erstellen ------------------------
fig = figure('Name','STM32 Sensor Live Plot','NumberTitle','off',...
             'Position',[100 100 1200 900]);

%% ----------------- Plots -----------------------------
% Manuelle Achsenpositionen: [left bottom width height] in Normalized units (0-1)
axHeight = 0.18;  % Höhe pro Plot
axMargin = 0.02;  % Abstand zwischen Plots

axAcc  = axes('Parent',fig,'Position',[0.08 0.76 0.85 axHeight]); hold(axAcc,'on'); grid(axAcc,'on');
title(axAcc,'Accelerometer'); xlabel(axAcc,'Zeit [s]'); ylabel(axAcc,'m/s²');

axGyro = axes('Parent',fig,'Position',[0.08 0.55 0.85 axHeight]); hold(axGyro,'on'); grid(axGyro,'on');
title(axGyro,'Gyroscope'); xlabel(axGyro,'Zeit [s]'); ylabel(axGyro,'°/s');

axMag  = axes('Parent',fig,'Position',[0.08 0.34 0.85 axHeight]); hold(axMag,'on'); grid(axMag,'on');
title(axMag,'Magnetometer'); xlabel(axMag,'Zeit [s]'); ylabel(axMag,'µT');

colors = ['b','r','g'];
for i=1:3
    plot(axAcc, nan, nan, colors(i));
    plot(axAcc, nan, nan, colors(i), 'LineWidth', 1.5);
    plot(axGyro, nan, nan, colors(i));
    plot(axGyro, nan, nan, colors(i), 'LineWidth', 1.5);
    plot(axMag, nan, nan, colors(i));
    plot(axMag, nan, nan, colors(i), 'LineWidth', 1.5);
end

legend(axAcc, {'Acc X raw','Acc Y raw','Acc Z raw','Acc X filt','Acc Y filt','Acc Z filt'});
legend(axGyro, {'Gyro X raw','Gyro Y raw','Gyro Z raw','Gyro X filt','Gyro Y filt','Gyro Z filt'});
legend(axMag, {'Mag X raw','Mag Y raw','Mag Z raw','Mag X filt','Mag Y filt','Mag Z filt'});

%% ----------------- Log-Feld -----------------------------
txtLog = uicontrol('Style','listbox', ...
                   'Units','normalized', ...
                   'Position',[0.08 0.01 0.85 0.15], ... % unten 22% für Log
                   'String',{}, ...
                   'FontSize',10);

txtStatus = uicontrol('Style','text', ...
                      'Units','normalized', ...
                      'Position',[0.08 0.31 0.3 0.03], ... % Status über Log
                      'String','Status: nicht verbunden', ...
                      'FontSize',10);

%% ----------------- Buttons -----------------------------
btnConnect = uicontrol('Style','pushbutton','String','Verbinden',...
                       'Units','normalized', ...
                       'Position',[0.45 0.31 0.1 0.04]);

btnDisconnect = uicontrol('Style','pushbutton','String','Trennen',...
                          'Units','normalized', ...
                          'Position',[0.56 0.31 0.1 0.04],...
                          'Enable','off');

end
