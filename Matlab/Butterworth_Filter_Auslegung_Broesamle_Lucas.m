clear; clc;

% User parameters
fs = 100;              % Sampling frequency in Hz
fc = 10;               % Cutoff frequency in Hz

% Normalized cutoff frequency
% -----------------------------
Wn = fc / (fs/2);      % Normalized cutoff for MATLAB IIR functions

% Butterworth filter design
% (Direct Form coefficients)
% b = numerator coefficients
% a = denominator coefficients
[b, a] = butter(5, Wn);    

% Convert to Second-Order Sections
% Required for STM32 implementation (CTF partitioning)
sos = tf2sos(b, a);    % Each row: [b0 b1 b2 a0 a1 a2]
g = 1;                 % Overall gain (usually = 1 for Butterworth)

% Display coefficients
disp('===== SOS Matrix for STM32 (second-order sections) =====');
disp(sos);

disp('===== Gain (g) =====');
disp(g);

disp('===== Filter Parameters =====');
fprintf('Sampling frequency fs = %d Hz\n', fs);
fprintf('Cutoff frequency fc = %d Hz\n', fc);
fprintf('Normalized cutoff Wn = %.4f\n', Wn);

% Optional: Plot the filter response
fvtool(sos, 'Analysis', 'freq');
title('Butterworth 5th Order - SOS Implementation');