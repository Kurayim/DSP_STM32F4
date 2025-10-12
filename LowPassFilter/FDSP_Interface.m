
clear
clc

s = serialport("COM8", 115200);
s.Timeout = 1000;
COM_START_SEND_SIGNAL = uint8('10');
ANS_MATLAB_READY_RX = uint8([0xAA, 0x55, 0x5A, 0xA5, 0xBB, 0xCC, ...
                             0x81, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, ...
                             0xFF, 0xFF, 0x0A, 0xFF]);

FlagExtracteSample = logical(false);
Signals = [];
ScalsePow = 0;
CountSignal = 0;
SampleRate = 0;
IndexTotalSample = 0;



while true
    
    if (s.NumBytesAvailable > 15 )
        PacketData = read(s, s.NumBytesAvailable, "uint8");
        indexPacket = length(PacketData);
        CheckSum = bitor( ...
                          bitor(bitshift(uint32(PacketData(indexPacket-4)), 24), bitshift(uint32(PacketData(indexPacket-3)), 16)), ...
                           bitor(bitshift(uint32(PacketData(indexPacket-2)), 8), uint32(PacketData(indexPacket-1))) ...
                           );
        CalCheckSum = uint32(0);
        for index = 1 : (length(PacketData) - 4)
           CalCheckSum = CalCheckSum + PacketData(index);
        end

        if(CalCheckSum == CheckSum)
            for i = 1 : 2 : (length(PacketData) - 5)
                
                IndexTotalSample = IndexTotalSample + 1;       
                value16 = typecast((bitor(bitshift(uint16(PacketData(i)), 8), uint16(PacketData(i+1)))), 'int16');
                Signals(IndexTotalSample) = double(value16) / double(ScalsePow);

                fprintf('Signals(%d) = %.12f\n', IndexTotalSample, Signals(IndexTotalSample));
              

            end
        end
    end

    if (s.NumBytesAvailable == 15)
        data = read(s, s.NumBytesAvailable, "uint8");

        if(data(6) == hex2dec('15'))
            
            byteCounter = 0;
            for i = 1:length(data)
                fprintf("Byte %d: 0x%s\n", byteCounter, dec2hex(data(i), 2));
                byteCounter = byteCounter + 1;
            end
         
            if(data(1) == hex2dec('AA')  &&  data(2) == hex2dec('55')  &&  data(3) == hex2dec('5A')  &&  ...
               data(4) == hex2dec('A5')) 

               CheckSum = uint32(0);
               for index = 1 : (length(data) - 2)
                   CheckSum = CheckSum + data(index);
               end
               CalCheckSum = bitor(bitshift(uint16(data(14)), 8), uint16(data(15)));
               if(CheckSum == CalCheckSum)
                   if(data(7) == hex2dec('10'))

                        ScalsePow = bitor(bitshift(uint16(data(10)), 8), uint16(data(11)));
                        SampleRate = bitor(bitshift(uint16(data(8)), 8), uint16(data(9)));

                        if(data(7) == hex2dec(COM_START_SEND_SIGNAL))
                            IndexTotalSample = 0;
                            write(s, ANS_MATLAB_READY_RX, "uint8");
                        end
                   end
                   if(data(7) == hex2dec('11'))
                        FlagExtracteSample = true;
                        CountSignal = CountSignal + 1;
                   end     
               end 
            end
        end
    end
    
    if(FlagExtracteSample == true)

        FlagExtracteSample = false;
        
        % ==== Input Parameters ====
        % Signals: Your signal array
        % IndexTotalSample: Total number of samples
        % SampleRate: Sampling rate in Hz
        
        % Ensure data types are double
        Signals = double(Signals);
        SampleRate = double(SampleRate);
        
        % Create time axis in seconds
        t = (0:IndexTotalSample-1) / SampleRate;
        
        % Plot the signal
        figure('Name','Signal','NumberTitle','off');
        plot(t, Signals, 'b', 'LineWidth', 0.1);
        grid on;
        
        % Labels and title
        title('Time-Domain Signal', 'FontSize', 14, 'FontWeight', 'bold');
        xlabel('Time (seconds)', 'FontSize', 12);
        ylabel('Amplitude', 'FontSize', 12);
        
        % Adjust axis for better view
        xlim([0 max(t)]);
        set(gca, 'FontSize', 12, 'LineWidth', 1);
            
    
            
    
        % ==== Input Parameters ====
        % Signals: your signal array
        % SampleRate: sampling rate (Hz)
        
        % Convert signal to double
        Signals = double(Signals);
        SampleRate = double(SampleRate);
        
        % Define parameters
        window = hamming(256);   % Window length = 256 samples
        noverlap = 128;          % Overlap = 50%
        nfft = 512;              % FFT length
        
        % Plot spectrogram
        figure('Name','Spectrogram','NumberTitle','off');
        spectrogram(Signals, window, noverlap, nfft, SampleRate, 'yaxis');
        
        % Title and color bar
        title('Spectrogram', 'FontSize', 14, 'FontWeight', 'bold');
        colorbar;


    end



    pause(0.01);
end















