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

In this project, the audio data is processed in frames of 1024 samples with a 512-sample overlap between consecutive frames.
Applying a window function and dividing the data into overlapping frames offers two essential advantages:

It prevents artificial discontinuities or spectral breaks at frame boundaries, resulting in a smoother and more natural frequency spectrum after the FFT.

It enables progressive reading of data from the SD card instead of loading the entire file into memory at once.
This approach significantly optimizes RAM utilization and serves as the primary solution for overcoming the microcontroller’s memory limitations.

At each processing step, after reading one frame, a Fast Fourier Transform (FFT) is applied.
The FFT output, which consists of complex values, is decomposed into its magnitude and phase components.
Since the goal in this stage is to estimate the average noise spectrum, the phase information is not used — only the magnitude values are considered for computation.

All magnitude spectra corresponding to the noise frames are accumulated, and at the end of the process, the total is divided by the number of frames.
The final result is a averaged noise spectrum frame, representing the estimated mean noise characteristics extracted from the first four seconds of the input audio file.

<br></br>


## Calculation of Noise Frame Count
In this part of the code, the number of processable frames within the first 4 seconds of the audio file (the noise-only segment) is calculated based on the total number of available samples:
```c
  uint32_t NoiseNumSam = NOISE_SEC * SAMP_RATE;
  float32_t NumFrame = ((NoiseNumSam - FRAME_LEN) / HALF_FRAME) + 1;
  float32_t IndexFrame   = NumFrame;
```
Explanation of variables:

NOISE_SEC – duration of the noise segment in seconds (in this project, 4 seconds).

SAMP_RATE – sampling rate of the input audio signal.

FRAME_LEN – frame length in samples (here, 1024 samples per frame).

HALF_FRAME – half of the frame length (512 samples), used to create a 50% overlap between consecutive frames.

The above formula determines how many overlapping frames can be extracted from the noise portion of the signal.
Finally, the variable IndexFrame is used as a counter to track the total number of frames available for reading and processing from the noise segment.

<br></br>

## Estimation of the Average Noise Spectrum
In this part of the code, the goal is to read environmental noise data from the SD card and compute the average noise spectrum by performing multiple Fast Fourier Transforms (FFT) on consecutive overlapping frames.

```c
ResultWave = WAVFIL_Start_Read( "Record_8.wav");
  ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead, HALF_FRAME);

  CopyFloatArray(wav_in, Frame, HALF_FRAME);
  ResetArray(NoiseSpectrum, HALF_FRAME);

  while(IndexFrame != 0){

  	  ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead, HALF_FRAME);

  	  CopyFloatArray(wav_in, &Frame[512], HALF_FRAME);

  	  WindowApply(Frame, HammingWin, FRAME_LEN);

  	  arm_rfft_fast_f32(&S, Frame, FFTOut, 0);

   	  arm_cmplx_mag_f32(FFTOut, fftMagnitude, HALF_FRAME);

   	  SumArray(NoiseSpectrum, fftMagnitude, HALF_FRAME);

   	  CopyFloatArray(wav_in, Frame, HALF_FRAME);

   	  IndexFrame--;
  }

  DivisionArray(NoiseSpectrum, NumFrame, HALF_FRAME);

  ResultWave = WAVFIL_End_Read();
```
At the beginning, the function WAVFIL_Catch_Data reads 512 samples from the input audio file (Record_8.wav) and stores them in the array wav_in.
These samples are then copied into the first half of the Frame array (indices 0–511).
The array NoiseSpectrum is initialized with zeros to prepare it for accumulating the spectral data.

The while loop runs for as many iterations as the number of frames in the noise segment (IndexFrame).
During each iteration, the following steps are executed:

1. Reading new data

    The next 512 samples are read from the SD card and stored in the second half of the Frame array (indices 512–1023).
    At this point, a complete frame of 1024 continuous samples is formed, ready for processing.

2. Applying the Hamming window

      The Hamming window is applied to the frame using the function WindowApply() to minimize discontinuities at frame boundaries and reduce spectral artifacts.

3. FFT computation

    The Fast Fourier Transform (FFT) is then computed using the arm_rfft_fast_f32() function from the CMSIS-DSP library.
    The complex output is stored in the array FFTOut, where:

    FFTOut[0] contains the real part of the first complex sample,

    FFTOut[1] contains the imaginary part,
    and this pattern continues in pairs throughout the array.

4. Magnitude calculation

    To compute the magnitude spectrum, the function arm_cmplx_mag_f32() is used.
    It calculates the absolute magnitude of each complex sample from FFTOut and stores the results in the array fftMagnitude.

5. Spectrum accumulation

    The magnitude spectrum obtained from each frame (fftMagnitude) is accumulated with the spectra of previous frames using the function SumArray().
    The cumulative result is stored in the NoiseSpectrum array.

6. Preparing for the next frame

    The contents of wav_in are shifted to the beginning of Frame to form the next overlapping frame.
    The frame counter IndexFrame is decremented, and the loop continues until all noise frames are processed.

    At the end of the loop, the accumulated sum of all spectra is stored in NoiseSpectrum.
    To compute the average noise spectrum, the array values are divided by the total number of frames (NumFrame) using the function DivisionArray().

    The final result represents the mean noise spectrum estimated from the first four seconds of the input audio file.
    This averaged noise spectrum is later used in the spectral subtraction stage to remove background noise from the speech signal.

<br></br>

## Calculating the Number of Frames for the Entire 7-Second Audio Segment
In this part of the code, the total number of processable frames within the 7-second portion of the audio file is calculated as follows:

```c
NumFrame = (((R_WavremainData / 2) - FRAME_LEN) / HALF_FRAME) + 1;
IndexFrame = NumFrame;
```
Since each audio sample is 16 bits (2 bytes), dividing R_WavremainData by 2 yields the total number of samples.
By subtracting one frame length (FRAME_LEN, 1024 samples in this project) and dividing by the frame step size (HALF_FRAME, 512 samples, corresponding to a 50% overlap), and finally adding 1, the total number of frames that can be extracted from the 7-second segment is obtained.

The variable IndexFrame is then initialized with the same value to serve as a counter for tracking the remaining frames during the signal processing loop in subsequent stages.

















## Noise Reduction and Speech Reconstruction (Spectral Subtraction Process)

In this section of the code, the main objective is to utilize the previously obtained average noise spectrum (NoiseSpectrum) to suppress environmental noise and reconstruct the clean speech signal across the entire audio file.
The overall process is similar to the noise spectrum estimation stage but includes crucial differences in the signal restoration and time-domain reconstruction phases.

```c
  ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead, HALF_FRAME);

  CopyFloatArray(wav_in, Frame, HALF_FRAME);
  ResetArray(SignalMagnitud, HALF_FRAME);




  while(IndexFrame != 0){

	  ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead, HALF_FRAME);

	  CopyFloatArray(wav_in, &Frame[512], HALF_FRAME);

	  WindowApply(Frame, HammingWin, FRAME_LEN);

	  arm_rfft_fast_f32(&S, Frame, FFTOut, 0);

	  arm_cmplx_mag_f32(FFTOut, SignalMagnitud, HALF_FRAME);

	  ComputePhase(FFTOut, SignalPhase, HALF_FRAME);

	  CleanMagnitude(CleanSpectrum, SignalMagnitud, NoiseSpectrum, HALF_FRAME);

	  ApplySpectralFloor(CleanSpectrum, NoiseSpectrum, HALF_FRAME);

//---------------------------------------------

	  ComplexConvertPolarToAlgebraic(IFFTIn, CleanSpectrum, SignalPhase, HALF_FRAME);

	  arm_rfft_fast_f32(&S, IFFTIn, CleanSampleNew, 1);

	  WindowApply(CleanSampleNew, HammingWin, FRAME_LEN);


	  if(LetOverlapFrame){

		  SumArray(CleanSampleNew, CleanSampleOld, HALF_FRAME);
		  SimpleCalArray(2, CleanSampleNew, CleanSampleNew, &WinSum[512], HALF_FRAME);

	  }
	  else{

		  SimpleCalArray(2, CleanSampleNew, CleanSampleNew, WinSum, HALF_FRAME);

	  }

	  maxSample = 0.092862136481524f;
	  DivisionArray(CleanSampleNew, maxSample, HALF_FRAME);
	  GainApply(CleanSampleNew, Gain, HALF_FRAME);
	  LimitClipp(CleanSampleNew, HALF_FRAME);

	  ResultWave = WAVFIL_Give_Write(CleanSampleNew, HALF_FRAME);

	  CopyFloatArray(&CleanSampleNew[512], CleanSampleOld, HALF_FRAME);

	  IndexFrame--;
	  LetOverlapFrame = true;

	  CopyFloatArray(wav_in, Frame, HALF_FRAME);
  }
  if(IndexFrame == 0){

	  SimpleCalArray(2, CleanSampleNew, CleanSampleNew, &WinSum[1024], HALF_FRAME);

	  maxSample = 0.092862136481524f;
	  DivisionArray(CleanSampleNew, maxSample, HALF_FRAME);
	  GainApply(CleanSampleNew, Gain, HALF_FRAME);
	  LimitClipp(CleanSampleNew, HALF_FRAME);

	  ResultWave = WAVFIL_Give_Write(CleanSampleNew, HALF_FRAME);
  }

  ResultWave = WAVFIL_End_Read();
  ResultWave = WAVFIL_End_Write();
```

Before entering the processing loop, the first 512 samples are read from the SD card and placed in the first half of the Frame array (indices 0–511).
Then, the while loop iterates according to the number of available frames (IndexFrame).
At the beginning of each iteration, another 512 new samples are read from the audio file and stored in the second half of the Frame array (indices 512–1023), completing one full frame.

After the frame is formed, a Hamming window is applied using the WindowApply function to minimize discontinuities at frame boundaries.
Next, the Fast Fourier Transform (FFT) is computed using the arm_rfft_fast_f32 function from the CMSIS-DSP library, and the complex output is stored in the FFTOut array.
To extract spectral features, the arm_cmplx_mag_f32 and ComputePhase functions are used to calculate the magnitude and phase of the signal, respectively.
Preserving the original phase is critical at this stage to ensure accurate waveform reconstruction when converting back to the time domain.

In the next step, the average noise spectrum (NoiseSpectrum) is subtracted from the current signal spectrum using the CleanMagnitude function.
To prevent excessive suppression of speech components, the ApplySpectralFloor function enforces a spectral floor, maintaining a minimum spectral level and reducing artificial distortion in the output.

After spectral subtraction, the polar-domain data (magnitude and phase) are converted back to complex algebraic form using the ComplexConvertPolarToAlgebraic function.
The Inverse FFT (IFFT) is then performed via arm_rfft_fast_f32 (in inverse mode), reconstructing the signal in the time domain, and the result is stored in the CleanSampleNew array.
A Hamming window is applied again to smooth transitions between frames.

To handle frame overlap, the Overlap-Add method is employed: only the first half of each frame (512 samples) is written to the SD card, while the second half is retained in CleanSampleOld for overlap with the next frame.
This approach minimizes discontinuities between consecutive frames in the reconstructed audio signal.

At the end of each iteration, amplitude normalization is performed using the DivisionArray function to control dynamic range and prevent clipping.
If necessary, GainApply enhances the signal energy, and LimitClipp ensures that all sample values remain within valid limits, preventing signal saturation.

The loop continues until IndexFrame reaches zero, meaning all frames have been processed.
Finally, the reconstructed clean speech signal is saved to the SD card, and the WAVFIL_End_Read and WAVFIL_End_Write functions are called to properly close the read and write operations.




