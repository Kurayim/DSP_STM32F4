# Spectral Subtraction Method in STM32 & CMSIS-DSP
This project focuses on implementing the Spectral Subtraction algorithm on an STM32F4 microcontroller using the CMSIS-DSP library. The main objective is to remove noise from an audio signal and extract clean speech from a recorded signal containing stationary environmental noise.

In this project, we estimate the average noise spectrum from the audio signal using the Spectral Subtraction method and then subtract this spectrum from the original signal to isolate the speech component with minimal residual noise.

This work is a continuation of a previous project titled LowPassFilter, where FIR and IIR low-pass filtering methods were applied to remove noise. However, the results were not satisfactory, as a significant portion of the speech signal shared the same frequency range as the noise, and excessive filtering led to the loss of important speech information.

In this project, our approach is more precise and scientific. By analyzing the spectral characteristics of the signal, we aim to accurately estimate the noise spectrum and remove it from the original signal without compromising speech quality.

The algorithm was previously implemented and tested in MATLAB, producing very promising results. The current goal is to replicate and optimize this performance on the STM32F411 platform using the digital signal processing (DSP) capabilities of the microcontroller.


In this project, the audio file is provided to the microcontroller via an SD card.
The file is specifically designed so that within its total duration of 7 seconds, the first 4 seconds contain only environmental noise with no speech signal present. From the fourth second onward, the speech segment begins.

In the first stage, a Fast Fourier Transform (FFT) is applied to the noise-only portion (the first 4 seconds).
By averaging the resulting spectra, the mean noise spectrum is estimated.
Next, an FFT is also performed on the entire audio file, and the averaged noise spectrum is subtracted from the signal spectrum.
This operation is carried out in a windowed manner to preserve the temporal variations of the signal.
Finally, by applying the Inverse FFT (IFFT), the enhanced (noise-reduced) signal is reconstructed in the time domain.

However, this process is not as straightforward in practice as it appears in theory, due to significant hardware limitations of the microcontroller.
The STM32F411 provides only 128 KB of RAM, whereas the size of the input audio file exceeds 300 KB.
As a result, it is not possible to load the entire file into memory at once.

To overcome this limitation, the audio data must be processed incrementally.
The samples are read from the SD card in segments, and both FFT and spectral processing are performed in a block-processing fashion.
This approach enables efficient memory usage while maintaining the integrity and continuity of the audio signal during spectral analysis and reconstruction.

Another major challenge in this project is the processing speed limitation.
The STM32F411 microcontroller operates at a maximum frequency of 100 MHz, which is not sufficient for performing computationally intensive tasks such as Fast Fourier Transform (FFT) processing and real-time implementation of the Spectral Subtraction algorithm.

During development, it became evident that this microcontroller is not inherently designed for heavy audio processing or high-rate spectral computation.
Expecting real-time performance comparable to that of a PC or a dedicated DSP processor would therefore be unrealistic.

Nevertheless, the main objective of this project is to demonstrate the implementation process of the Spectral Subtraction algorithm and to gain a deeper understanding of the steps involved in noise reduction on a resource-constrained platform.
Instead of focusing solely on execution speed, the emphasis was placed on algorithmic correctness, structured implementation, and logical optimization within the system’s hardware limits.
<br></br>



















