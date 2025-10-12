%% FIR Low-pass Filter Coefficient Generator
clc; clear;

% ---- Filter Parameters ----
Fs = 22050;      % Sample Rate (Hz)
Fc = 1000;       % Cut-off Frequency (Hz)
N  = 128;        % Number of taps (filter order + 1)
windowType = 'hamming';  % Window type: 'hamming', 'blackman', or 'kaiser'
beta = 5;        % Only used for Kaiser window

% ---- Calculate Coefficients ----
Wn = Fc / (Fs/2);   % Normalize cutoff frequency to [0,1]

% Generate window array
switch lower(windowType)
    case 'hamming'
        win = hamming(N);
    case 'blackman'
        win = blackman(N);
    case 'kaiser'
        win = kaiser(N, beta);
    otherwise
        error('Window type not recognized');
end

b = fir1(N-1, Wn, win);  % Design filter using the window array

% ---- Normalization (optional) ----
b = b / sum(b);  % Ensure unity gain at DC

% ---- Display coefficients in the command window ----
fprintf('\n=== FIR Low-pass Filter Coefficients (%d taps) ===\n', N);
for i = 1:N
    fprintf('%3d: %+1.10f\n', i, b(i));
end

% ---- Display frequency response ----
figure;
freqz(b, 1, 1024, Fs);
title('Frequency Response of FIR Low-pass Filter');
grid on;
