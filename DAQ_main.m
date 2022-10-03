%DAQ script, to be launched on a host computer controlling the experiment


clear all;

%% **************************************************************************
% LOAD MEASUREMENT PARAMETERS
%**************************************************************************
usDataFile = 'settings/m4_z1to35_fpp30_ppt3_hwn10_apod128'; %must be identical with the data file passed to VSX script in Verasonics

load(usDataFile,'npoints','fpCoordVec','fpCoord','framesPerPoint','nframes','pulsesPerTrigger');
%**************************************************************************


%% **************************************************************************
% EXTERNAL HARDWARE TRIGGER-TIMER SETUP
%**************************************************************************
mcfreq = 16e6;  %microcontroller clock frequency
timdiv = 1;     %timer/counter divider
endOfDataCode = 21845;
measDelays = zeros(nframes,pulsesPerTrigger);   %allocate memory for laser delay measurement results
availablePorts = serialportlist;
portName = availablePorts(3); %only arduino board connected in this version!
baudRate = 9600;
s = serialport(portName,baudRate,'Timeout',1);    %added timeout to avoid long waiting times in case of any error (10 secs by default)
pause(3);   %wait for the serial port to be ready!
disp('Connection with external trigger/counter established.');
%*************************************************************************



%% **************************************************************************
% CAMERA SETUP
%**************************************************************************
et=pulsesPerTrigger; %exposure time in ms - IN AOI4 IT DETERMINES THE NUMBER OF LASER PULSES PER FRAME!!!!

roi=[1042 1542 504 200];    %ROI setting no. 2, determined experimentally for new integrated probe

vid = videoinput('gentl', 1, 'Mono8');
vid.ROIPosition=roi;
%VID trigger settings:
triggerconfig(vid, 'hardware', 'DeviceSpecific', 'DeviceSpecific');
vid.FramesPerTrigger = 1;   %One frame per trigger downloaded and processed immidiately
vid.TriggerRepeat = nframes-1;  %no. of frames to capture

src = getselectedsource(vid);
src.TriggerSelector = 'FrameStart';
src.TriggerActivation = 'RisingEdge';
src.TriggerDelay = 0;   %delay in us
src.TriggerSource = 'Line0';
src.TriggerMode = 'On';
src.ExposureMode = 'Timed';
src.ExposureTime = et*1000;
%Some important defaults, explicitly configured for clarity, or to allow changes here:
src.ExposureAuto = 'Off'; %Dispabling automatic exposure:
src.Gain = 20;
src.GainAuto = 'Off';
start(vid);
disp('Camera ready, waiting for trigger...');
%*************************************************************************
















%% **************************************************************************
% MAIN PROGRAM LOOP
%**************************************************************************
sc = zeros(nframes,1);  %memory allocation for determined speckle contrast values
mi = zeros(nframes,1);  %memory allocation for determined mean intensity values
frameTimes = zeros(nframes,1);

for licz = 1:nframes    %for every frame to capture...
    %Start transmission my sending start code to external trigger/counter
    write(s,96,"uint8");    %96 is the start code
    write(s,et,"uint8");    %send texposure time = no, of pulses @ 1kHz laser freq
    
    %Capture frame, determine speckle contrast...
    [frame, tf] = getdata(vid);
    frame = double(frame(:));
    fsize = (roi(3)*roi(4));
    fmean = sum(frame)/fsize;
    sc(licz) = sqrt(sum((frame-fmean).^2)/(fsize-1))/fmean;
    mi(licz) = fmean;
    frameTimes(licz) = tf;

    
    %Mark end of the current frame, read delay values from timer/counter:
    write(s,69,"uint8");    %69 is the stop code
    dataH = 0;
    dataL = 0;
    nticks = 2^8 * dataH + dataL;
    liczdelay = 1;
    while nticks ~= endOfDataCode
        dataH = read(s,1,"uint8");
        dataL = read(s,1,"uint8");
        nticks = 2^8 * dataH + dataL;
        if nticks ~= endOfDataCode
            measDelays(licz,liczdelay) = nticks/(mcfreq/timdiv);
        end
        liczdelay = liczdelay + 1;
    end
    
end
%*************************************************************************



%% **************************************************************************
% CLEANING UP AND POST-PROCESSING
%**************************************************************************
stop(vid);
clear vid src s
clearvars availablePorts command dataH dataL endOfDataCode portName

scd = sc(1:2:end) - sc(2:2:end);

%Save the results:s
dataName = strsplit(usDataFile,'m4_');
dataName = dataName{2};
outfname = strcat('results_',dataName,'_',strrep(strrep(strrep(string(datetime('now')),' ','_'),':','_'),'-','_'));
command = strcat({'save '},outfname);
eval(command)
%*************************************************************************