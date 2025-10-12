%% IIR Low-pass Butterworth Coefficients Printer (b,a + SOS + CMSIS)
clc; clear;

% ---- Filter specs ----
fs    = 22050;     % Sample rate (Hz)
fc    = 2000;      % Cutoff frequency (Hz)
order = 6;         % Butterworth order (even -> 3 biquads)

Wn = fc/(fs/2);    % Normalize cutoff to Nyquist

% ---- Direct-form (b,a) coefficients ----
% That is, what percentage of the frequency does Wn pass between 0 and (Fs/2)?
[b, a] = butter(order, Wn, 'low');   % digital IIR Butterworth (bilinear)

% Optional: ensure unity DC gain (usually already ~1 for lowpass Butter)
% gdc = sum(b)/sum(a);  b = b/gdc;

% ---- Pretty print (b,a) ----
fprintf('\n=== Butterworth Low-pass (order=%d, fc=%g Hz, fs=%g Hz) ===\n', order, fc, fs);
fprintf('--- Direct-Form Coefficients (b) ---\n');
for i = 1:numel(b)
    fprintf('b(%d) = %+1.15f\n', i, b(i));
end

fprintf('\n--- Direct-Form Coefficients (a) ---\n');
for i = 1:numel(a)
    fprintf('a(%d) = %+1.15f\n', i, a(i));
end

% ---- SOS (biquads) for stable implementation ----
% Get zeros, poles, gain then convert to SOS
[z, p, k] = butter(order, Wn, 'low');
[sos, g]  = zp2sos(z, p, k);     % sos: [b0 b1 b2 a0 a1 a2], gain g

% Apply the overall gain g to the FIRST section numerator
sos(1,1:3) = sos(1,1:3) * g;

fprintf('\n--- SOS (biquads) [b0 b1 b2 a0 a1 a2], gain applied to first section ---\n');
for s = 1:size(sos,1)
    fprintf('SOS%u: ', s);
    fprintf('%+1.15f ', sos(s,:));
    fprintf('\n');
end

% ---- CMSIS-DSP biquad coeffs (DF1/DF2): {b0, b1, b2, a1, a2} per section ----
% Note: CMSIS expects a1,a2 with SIGN NEGATED (difference eqn has minus signs).
numStages = size(sos,1);
cmsisCoeffs = zeros(numStages, 5);

for s = 1:numStages
    b0 = sos(s,1); b1 = sos(s,2); b2 = sos(s,3);
    a0 = sos(s,4); a1 = sos(s,5); a2 = sos(s,6);

    % Normalize to a0 = 1 (safety; usually already 1)
    b0 = b0/a0; b1 = b1/a0; b2 = b2/a0; a1 = a1/a0; a2 = a2/a0;

    % CMSIS format: {b0, b1, b2, -a1, -a2}
    cmsisCoeffs(s,:) = [b0, b1, b2, -a1, -a2];
end

fprintf('\n--- CMSIS-DSP biquad coeffs per stage: {b0, b1, b2, a1, a2} with a1,a2 NEGATED ---\n');
for s = 1:numStages
    fprintf('Stage %d: { %+1.15ff, %+1.15ff, %+1.15ff, %+1.15ff, %+1.15ff }\n', ...
        s, cmsisCoeffs(s,1), cmsisCoeffs(s,2), cmsisCoeffs(s,3), cmsisCoeffs(s,4), cmsisCoeffs(s,5));
end

% ---- (Optional) Quick frequency response check in MATLAB (not required) ----
% figure; freqz(b,a, 2048, fs); grid on; title('Butterworth LPF freq response');
