# Spectral Subtraction Method in STM32 & CMSIS-DSP
This project focuses on implementing the Spectral Subtraction algorithm on an STM32F4 microcontroller using the CMSIS-DSP library. The main objective is to remove noise from an audio signal and extract clean speech from a recorded signal containing stationary environmental noise.

In this project, we estimate the average noise spectrum from the audio signal using the Spectral Subtraction method and then subtract this spectrum from the original signal to isolate the speech component with minimal residual noise.

This work is a continuation of a previous project titled LowPassFilter, where FIR and IIR low-pass filtering methods were applied to remove noise. However, the results were not satisfactory, as a significant portion of the speech signal shared the same frequency range as the noise, and excessive filtering led to the loss of important speech information.

In this project, our approach is more precise and scientific. By analyzing the spectral characteristics of the signal, we aim to accurately estimate the noise spectrum and remove it from the original signal without compromising speech quality.

The algorithm was previously implemented and tested in MATLAB, producing very promising results. The current goal is to replicate and optimize this performance on the STM32F411 platform using the digital signal processing (DSP) capabilities of the microcontroller.

<br></br>



















