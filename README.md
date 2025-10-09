# Low Pass Filter in STM32 & CMSIS-DSP

In the NoiseReduction_speech_STM32F4 repository, our main goal is to eliminate constant background noise from speech signals. However, at this stage—which is considered the initial phase of the project—our focus is on implementing a low-pass filter on the STM32F411 microcontroller using the CMSIS-DSP library.

The project scenario is defined as follows: a .wav audio file is stored on an SD card. The data from this file is then read, processed through the filter, and the filtered output is written back to the same SD card.

In this project, we use the STM32F411CEU6 microcontroller, which features a CORTEX-M4 processor, 512 KB of flash memory, and 128 KB of SRAM. One of the main challenges in this project is the limited SRAM, which does not allow the entire file to be read at once and processed through the filter in a single step.
To address this issue, the data is read from the SD card in frames of 1024 samples, with each sample represented as a 16-bit value. After processing and filtering each frame, the output is stored in a new file in the same format. This process is then repeated for subsequent frames until all the samples in the file have been processed.

It is worth noting that the data in the WAVE file is stored as 8-bit values, and for processing, two consecutive values must be combined to form a single 16-bit sample. Therefore, to create a frame of 1024 samples for processing, 2048 bytes must be read from the SD card. This process is handled by the WAV file-related functions implemented in the project.
Another important point is that, for filter implementation, it is preferable to change the data format by normalizing the sample values to the range [-1, 1]. This improves processing accuracy and filter performance and is also applied within the project’s WAV functions.

I will provide a set of explanations regarding the code of this program, which will contribute to a better understanding of the project.
<br></br>



>In this project, the implementation of the low-pass filter is carried out using two methods: FIR and IIR. By defining FILTER_M, the desired method can be selected and applied.<
```c
// Select Low Pass Filter with FIR OR IIR
#define FIR_METHOD		1
#define IIR_METHOD		0
#define FILTER_M		FIR_METHOD
}
```

<br></br>
        
The implementation of FIR and IIR methods requires the corresponding filter coefficients. In this project, the FIR low-pass filter coefficients are generated using the script Generate_FIR_LPF_Coeff.m, while the IIR low-pass filter coefficients are produced using Generate_IIR_LPF_Coeff.m. Both scripts are executable in MATLAB and provide the necessary coefficient values for implementing the filters on the microcontroller.



![MATLAB Badge](https://img.shields.io/badge/language-MATLAB-blue)
>This program is designed for generating FIR filter coefficients. By appropriately setting the parameters Fs (sampling frequency), Fc (cutoff frequency), N (number of taps), and windowType (type of window), and, if necessary, beta (for the Kaiser window), the required filter coefficients can be obtained from the program’s output.<
```c
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

```
<br></br>
>In the file Generate_IIR_LPF_Coeff.m, by correctly setting the parameters Fs, Fc, and order, and running the program in the MATLAB environment, the required IIR low-pass filter coefficients can be obtained from the program’s output.<
```c
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

```
<br></br>

By comparing the Fc parameters in the two programs, it can be observed that the cutoff frequency of the FIR filter is 1 kHz, while that of the IIR filter is 2 kHz. The corresponding filter coefficients are stored in the project’s header files under the name firCoeffs.h.

```c
const float firCoeffs[FIR_TAP_NUM] = {
    -0.0002753720f, -0.0003545342f, -0.0004136985f, -0.0004480195f,
    -0.0004520794f, -0.0004203027f, -0.0003477614f, -0.0002312985f,
    -0.0000708224f,  0.0001294268f,  0.0003598754f,  0.0006051102f,
     0.0008440715f,  0.0010510784f,  0.0011977249f,  0.0012555736f,
     0.0011994532f,  0.0010110557f,  0.0006824417f,  0.0002190132f,
    -0.0003584915f, -0.0010133768f, -0.0016940359f, -0.0023368088f,
    -0.0028706148f, -0.0032231193f, -0.0033279977f, -0.0031326825f,
    -0.0026058531f, -0.0017438612f, -0.0005752954f,  0.0008370138f,
     0.0023970771f,  0.0039801975f,  0.0054407720f,  0.0066231039f,
     0.0073745636f,  0.0075601446f,  0.0070772439f,  0.0058693573f,
     0.0039373604f,  0.0013471419f, -0.0017674278f, -0.0052068858f,
    -0.0087142088f, -0.0119883096f, -0.0147023432f, -0.0165257167f,
    -0.0171483145f, -0.0163051699f, -0.0137996730f, -0.0095234125f,
    -0.0034709164f,  0.0042521231f,  0.0134281049f,  0.0237346755f,
     0.0347596035f,  0.0460225868f,  0.0570026430f,  0.0671692675f,
     0.0760152065f,  0.0830885227f,  0.0880216379f,  0.0905552316f,
     0.0905552316f,  0.0880216379f,  0.0830885227f,  0.0760152065f,
     0.0671692675f,  0.0570026430f,  0.0460225868f,  0.0347596035f,
     0.0237346755f,  0.0134281049f,  0.0042521231f, -0.0034709164f,
    -0.0095234125f, -0.0137996730f, -0.0163051699f, -0.0171483145f,
    -0.0165257167f, -0.0147023432f, -0.0119883096f, -0.0087142088f,
    -0.0052068858f, -0.0017674278f,  0.0013471419f,  0.0039373604f,
     0.0058693573f,  0.0070772439f,  0.0075601446f,  0.0073745636f,
     0.0066231039f,  0.0054407720f,  0.0039801975f,  0.0023970771f,
     0.0008370138f, -0.0005752954f, -0.0017438612f, -0.0026058531f,
    -0.0031326825f, -0.0033279977f, -0.0032231193f, -0.0028706148f,
    -0.0023368088f, -0.0016940359f, -0.0010133768f, -0.0003584915f,
     0.0002190132f,  0.0006824417f,  0.0010110557f,  0.0011994532f,
     0.0012555736f,  0.0011977249f,  0.0010510784f,  0.0008440715f,
     0.0006051102f,  0.0003598754f,  0.0001294268f, -0.0000708224f,
    -0.0002312985f, -0.0003477614f, -0.0004203027f, -0.0004520794f,
    -0.0004480195f, -0.0004136985f, -0.0003545342f, -0.0002753720f
};

const float32_t biquadCoeffs[NUM_STAGES * 5] = {
    // Stage 1
    +0.000206046069855f, +0.000412092139711f, +0.000206046069855f, +1.106983833903043f, -0.314780900552466f,
    // Stage 2
    +1.000000000000000f, +2.000000000000000f, +1.000000000000000f, +1.218879336445586f, -0.447680826545009f,
    // Stage 3
    +1.000000000000000f, +2.000000000000000f, +1.000000000000000f, +1.477569487213168f, -0.754930904616197f
};
```
<br></br>

In the main.c file, which contains the core implementation of the program, a set of modules and functions are utilized, including CMSIS-DSP, WAVE File, and MATLAB Interface. Their roles are defined as follows:
CMSIS-DSP: Performs digital signal processing operations and applies the designed filters.
WAVE File: Manages data read and write operations on the SD card and performs the necessary conversions to ensure data format compatibility.
MATLAB Interface: Transfers the output signals from the STM32 microcontroller to the MATLAB environment for analysis, evaluation, and validation of the results.
In the following sections, each of these modules will be examined and explained in greater detail.
![C Badge](https://img.shields.io/badge/language-C-blue)



>This section of the code corresponds to the initial configuration required for the filter functions. As shown, by assigning the appropriate value to the variable FILTER_M, the filter type can be switched between IIR and FIR. The primary role of these functions is to receive the address of the first element of the filter coefficient array and prepare them for use in the signal processing workflow.<
```c
  // Filter initialization
  #if FILTER_M == FIR_METHOD
  	  arm_fir_init_f32(&S, FIR_TAP_NUM, (float32_t *)firCoeffs, firState, BLOCK_SIZE);
  #else
  	  arm_biquad_cascade_df2T_init_f32(&IIR, NUM_STAGES, (float32_t *)biquadCoeffs, (float32_t *)biquadState);
  #endif
```


>In this section, the functions related to WAV file handling are used to open the file Recording_8.wav, thereby enabling access to the data contained within it.<
```c
        sd_mount();
        // Open recording file
        ResultWave = WAVFIL_Start_Read( "Recording_8.wav");
        if(ResultWave != WAV_OK){
          break;
          FlagKey = false;
        }
```



>In this section, depending on the selected filter type, a new .wav file is created to store the data after the final processing is completed.<
```c
          // Create a new wave file
          #if FILTER_M == FIR_METHOD
                  ResultWave = WAVFIL_Start_Write("LPF_FIR.wav", R_WavremainData, 22050);
                if(ResultWave != WAV_OK){
                        break;
                        FlagKey = false;
                }
          #else
                  ResultWave = WAVFIL_Start_Write("LPF_IIR.wav", R_WavremainData, 22050);
                if(ResultWave != WAV_OK){
                        break;
                        FlagKey = false;
                }
          #endif
```


>In this section, a frame consisting of 1024 16-bit samples is read from the SD card to perform the processing operations.<
```c
          // Give data from wave file in the sd card
          WAVFIL_Catch_Data(wav_in, &NumByteRead);
          if(ResultWave != WAV_OK){
                  break;
                  FlagKey = false;
          }
```


>The purpose of the MAT_Connect function is to transmit the required information to the MATLAB environment and prepare it for receiving the signal samples.<
```c
          // Trying to connect to MATLAB on the PC
          ResultMat = MAT_Connect(22050, 32767, NumByteRead);
          if(ResultMat != MAT_OK){
                  break;
                  FlagKey = false;
          }
```

>In this section, the selected filter is optionally applied to the data prepared for processing, and the resulting output is stored in the wav_out array.<
```c
          // Apply Low Pass Filter
          #if FILTER_M == FIR_METHOD
                  arm_fir_f32(&S, wav_in, wav_out, NumByteRead);
          #else
                  arm_biquad_cascade_df2T_f32(&IIR, wav_in, wav_out, BLOCK_SIZE);
          #endif
```

>In this section, the processed data stored in the wav_out array is written to a new WAV file for storage on the SD card.<
```c
          // Storing Filtered data in the sd card
          ResultWave = WAVFIL_Give_Write(wav_out);
          if(ResultWave != WAV_OK){
                  break;
                  FlagKey = false;
          }
```


>In this section, the MAT_SendSamples function transmits the data contained in the wav_out array to the MATLAB environment via the USB protocol.<
```c
          // Send Filtered data to MATLAB
          ResultMat = MAT_SendSamples(wav_out, NumByteRead, 32767.5f);
          if(ResultMat != MAT_OK){
                  break;
                  FlagKey = false;
          }
```


>Initially, these steps are executed once to obtain the necessary information, such as the remaining data to be processed (R_WavremainData) and other related values. Subsequently, the process is repeated in a loop until R_WavremainData reaches zero, indicating that there are no more samples to read and that the processing is complete.<
```c
          // We need to continue this until the end of the wavw recording file.
          while(R_WavremainData != 0){

                  // Give data from wave file in the sd card
                  ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead);
                  if(ResultWave != WAV_OK){
                          break;
                          FlagKey = false;
                  }

                  // Apply Low Pass Filter
                  #if FILTER_M == FIR_METHOD
                          arm_fir_f32(&S, wav_in, wav_out, NumByteRead);
                  #else
                          arm_biquad_cascade_df2T_f32(&IIR, wav_in, wav_out, BLOCK_SIZE);
                  #endif

                  // Send Filtered data to MATLAB
                  ResultMat = MAT_SendSamples(wav_out, NumByteRead, 32767.5f);
                  if(ResultMat != MAT_OK){
                          break;
                          FlagKey = false;
                  }

                  // Storing Filtered data in the sd card
                  ResultWave = WAVFIL_Give_Write(wav_out);
                  if(ResultWave != WAV_OK){
                          break;
                          FlagKey = false;
                  }
          }
```


>At this stage, all files opened on the SD card are closed, and a command is sent to MATLAB to indicate that the signal data has been fully processed, allowing MATLAB to perform the final analysis and evaluation.<
```c
          // Send to MATLAB end signal.
          MAT_EndSignal();
          // Close the wave recording file.
          ResultWave = WAVFIL_End_Read();
          // Close the new wave filtered file.
          ResultWave = WAVFIL_End_Write();
```

<br></br>


This figure shows the original (raw) audio signal, which contains environmental noise. As observed in the spectrogram, different frequencies are present in the signal with varying energy levels.
<p align="center">
  <img src="https://github.com/user-attachments/assets/ffe62523-c344-4b00-92df-7a6f7fef3eeb" width="1200"/>
</p>

<br></br>



This figure illustrates the performance of the low-pass filter implemented using the FIR method. The purpose of this filter is to attenuate frequencies above 1 kHz while allowing lower frequencies to pass. The filter design utilizes a Hamming window.

<p align="center">
   <img width="1915" height="662" alt="Screenshot 2025-09-05 130248" src="https://github.com/user-attachments/assets/08d9e142-13eb-4041-b010-69edcd7caae9" />
</p>

<br></br>


This figure depicts the effect of the low-pass filter implemented using the IIR method on the STM32 microcontroller. The filter’s cutoff frequency is set to 2 kHz, and frequencies above this value are attenuated or eliminated by the filter.
<p align="center">
   <img width="1918" height="643" alt="Screenshot 2025-09-05 131030" src="https://github.com/user-attachments/assets/c5c851d9-1fee-4592-85db-b8da2c6acee6" />
</p>

Although the results obtained from these filters indicate that their overall performance is correct, this does not imply complete noise removal from the original signal. Observations show that in certain critical speech-related frequencies, the original signal is also attenuated. Therefore, the outcome at this stage is not fully satisfactory in terms of the expected signal quality.



