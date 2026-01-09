% Hauptskript zum Starten der Anwendung
function Start_MATLAB_Skript_Broesamle_Lucas()
    % Alle vorherigen Fenster und Daten löschen
    close all;
    clear classes;
    clc;
    
    % Erstelle und starte die Visualisierung
    app = MATLAB_Skript_Broesamle_Lucas();
    
    % Initiale Log-Nachricht
    app.addLogMessage('IMU Visualisierung gestartet');
    app.addLogMessage('Bitte COM-Port in Zeile 5 anpassen, aktuell auf "COM4" eingestellt.');
    app.addLogMessage('Klicken Sie auf "Verbinden" um Datenerfassung zu starten');
end