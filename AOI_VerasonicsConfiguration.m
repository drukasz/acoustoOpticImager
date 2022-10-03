%% DESCRIPTION AND SETTINGS
%Cobolt TorXS Lasers running constantly, triggered by an external
%signal generator (NOT Verasonics!!!). Such an approach allows to mitigate the jitter-related issues.
%The Verasonics is triggered by an
%external microcontroller control system, which provides trigger delay
%calculation and adjusts for variable delay to give the predicted fixed
%delay of a given value. In this way the Verasonics system can assume that
%the laser pulse emission will occurr within a specified time period after
%the trigger is detected, calculate the required US delay and fire US at
%the desired moment. 





clear all;

c = 1700;   %Speed of sound, [m/s]
apod=128;    %number of active transmit elements
fpp=30;     %frames per point
hwn=10;
vpeak = 85;   %peak voltage (half Vpp) for driving the transducer

fixedDelay = 200e-6;    %assumed fixed delay between the Trigger 1 IN event and the laser pulse emission

pulsesPerTrigger=20;    %how many laser pulses per single captured frame
afterPulsePause=1e-3;   %pause after firing every laser pulse [s] - MUST BE AT LEAST 1 ms!!!! The hardware blocks faster attempts as the max laser trigger rfreq is 1 kHz!
framesPerPoint=fpp;  %how many frames are captured for a single focal point. I.e. how many times specific sequence should be repeated before switching to the next coordinates





%**************************************************************************************************************
%% GEOMETRY CALCULATION
%**************************************************************************************************************
%geometry:
zStart=1;   %start depth > 0
zStop=35;   %size of the scanned cross section, z axis (depth), in mm
zRes=34;    %how many bins along z axis 

xSteps = 1; %number of scans along X axis (lateral) with a give apodization number
xStepSize = 8;  %expressed in number of elements

 

zSpan=zStop-zStart;
nZpoints =(zRes+1);
fpCoord=struct('x',{},'z',{});
for liczx = 1:xSteps
    for liczz = 1:zRes+1
            fpCoord(liczx,liczz).z = zStart + (liczz-1)*zSpan/zRes;
            fpCoord(liczx,liczz).x =  -(xSteps-1)*xStepSize/2 + (liczx-1)*xStepSize ; 
    end    
end


apodizationMatrix = zeros(xSteps,nZpoints,128);
for liczx = 1:xSteps
    for liczz = 1:nZpoints
        xScanStart = 64-round(apod/2)+1 -floor((xSteps-1)*(xStepSize/2)) + (liczx-1)*xStepSize;
        xScanStop = 64+round(apod/2) -floor((xSteps-1)*(xStepSize/2)) + (liczx-1)*xStepSize;
        apodizationMatrix(liczx,liczz, xScanStart : xScanStop)=1;  
    end
end
%find(apodizationMatrix(1,1,:)==1)  %test - check the apodization



npoints = nZpoints * xSteps;       
nframes = npoints * framesPerPoint * 2; %times 2, because number of frames counts both for us and no us cases
disp(strcat('Total number of frames to capture: ',num2str(nframes)));
disp(strcat('Estimated time: ',num2str(nframes/600),'min'));

%Reshape matrices to vectors for further processing - line scans
fpCoordVec = reshape(fpCoord',npoints,1);
apodizationVec = zeros(npoints,128);
for liczx = 1:xSteps
    for liczz = 1:zRes+1
        apodizationVec((liczx-1)*nZpoints+liczz,:) = apodizationMatrix(liczx,liczz,:);
    end
end
%**************************************************************************************************************














%**************************************************************************************************************
%% System parameters:
%**************************************************************************************************************
% Specify system parameters
Resource.Parameters.numTransmit = 128; % number of transmit channels.
Resource.Parameters.numRcvChannels = 128; % number of receive channels.
Resource.Parameters.connector = 1; % transducer connector to use.
Resource.Parameters.speedOfSound = c; % speed of sound in m/sec
%Symulacja:
Resource.Parameters.simulateMode = 0; % run script in simulate mode (1)
%  Resource.Parameters.simulateMode = 1 forces simulate mode, even if hardware is present.
%  Resource.Parameters.simulateMode = 2 stops sequence and processes
%  RcvData continuously. 
% Specify Resources.
Resource.RcvBuffer(1).datatype = 'int16';
Resource.RcvBuffer(1).rowsPerFrame = 4096;   % this size allows for maximum range
Resource.RcvBuffer(1).colsPerFrame = Resource.Parameters.numRcvChannels;
Resource.RcvBuffer(1).numFrames = 100;       % 100 frames used for RF cineloop.
Resource.InterBuffer(1).numFrames = 1;  % one intermediate buffer needed.
Resource.ImageBuffer(1).numFrames = 10;

%**************************************************************************************************************
%TRANSDUCER SPECIFICATION:
%**************************************************************************************************************
% Specify Trans structure array.
Trans.name = 'L7-4';
Trans.units = 'mm'; % Explicit declaration avoids warning message when selected by default
% Trans.frequency = 5; % not needed if using default center frequency
Trans = computeTrans(Trans);  % L7-4 transducer is 'known' transducer so we can use computeTrans.
Trans.maxHighVoltage = 90;  % set maximum high voltage limit for pulser supply.

%**************************************************************************************************************
%WAVEFORM SPECIFICATION:
%**************************************************************************************************************
% Specify Transmit waveform structure.
%A - czestotliwosc, B - czesc okresu przez ktora wzmacniacz jest aktywny,
%pomiedzy 0.1 a 1, okresla energie sygnalu; C - ilosc polowek okresu fali;
%D - czy zaczynamy od dodatniej (1) polaryzacji czy ujemniej (-1)
TW(1).type = 'parametric';
TW(1).Parameters = [Trans.frequency,0.67,hwn,1]; % A, B, C, D
%Programming manual, strona 16 - dotyczy programowania niezaleznych
%sygnalow dla wszystkich przetwornikow:
% In the case where one really would like independent parametric waveforms on individual
% channels, the TW.Parameters array can be expanded to a 128 (or 256) row, two
% dimensional array, where each row specifies the waveform for the transmitter of the same
% number as the row index.

%**************************************************************************************************************
% TX objects - Plane wave
%**************************************************************************************************************
for licz=1:npoints
    TX(licz).waveform = 1; % use 1st TW structure.
    TX(licz).Origin=[0,0,0]; 
    TX(licz).Steer=[0,0]; 
    TX(licz).focus=0; 
    TX(licz).Apod = apodizationVec(licz,:); 
    TX(licz).Delay = zeros(1,Trans.numelements);  %Plane wave imaging
end



% Specify PData structure array.
P.startDepth = 5;   % Acquisition depth in wavelengths
P.endDepth = 256;   % This should preferrably be a multiple of 128 samples.
PData(1).PDelta = [Trans.spacing, 0, 0.5];
PData(1).Size(1) = ceil((P.endDepth-P.startDepth)/PData(1).PDelta(3)); % startDepth, endDepth and pdelta set PData(1).Size.
PData(1).Size(2) = ceil((Trans.numelements*Trans.spacing)/PData(1).PDelta(1));
PData(1).Size(3) = 1;      % single image page
PData(1).Origin = [-Trans.spacing*(Trans.numelements-1)/2,0,P.startDepth]; % x,y,z of upper lft crnr.
% No PData.Region specified, so a default Region for the entire PData array will be created by computeRegions.
%**************************************************************************************************************





%**************************************************************************************************************
%% COMPUTE PROPAGATION TIMES FROM TRANSDUCER TO DESIRED IMAGING DEPTH, INCLUDE LASER DELAY:
%**************************************************************************************************************
propagationTimes=zeros(npoints,1);  %memory allocation
for licz = 1:npoints
        propagationTimes(licz)=(fpCoord(licz).z *1e-3) /c;
end
propagationTimes = fixedDelay - propagationTimes;   %SUBTRACT the propagation delay from the laser delay (introduced in v.7)
lasDelay=round((propagationTimes)/200e-9);  %delay values, expressed in 200 ns multiples for 'noop' sequence object
%**************************************************************************************************************














%% ************************************************************************
% EVENT SPECIFICATION:
% *************************************************************************
waitPtPvalue = round(afterPulsePause / 200e-9);   %convert value given in us into 200 ns multiply that should be passed to the software


liczEvent = 1;
liczSeq = 1;


    
 
for liczpunkt=1:npoints
   
    Event(liczEvent).info = 'ResetFramesPerPointCounter';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0; 
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0;
    Event(liczEvent).seqControl = liczSeq;
        SeqControl(liczSeq).command = 'loopCnt';  %set loop counter
        SeqControl(liczSeq).condition = 'counter2';  %loop counter 2 - frames per point
        SeqControl(liczSeq).argument = framesPerPoint - 1; %how many loop iterations will be performed (minus 1, in order to compensate for the first iteration)
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;   
    
    Event(liczEvent).info = 'ReturnToMatlab';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0;
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0;
    Event(liczEvent).seqControl = liczSeq;
        SeqControl(liczSeq).command = 'returnToMatlab'; 
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;      
    
    %% Capture frame: laser only

    Event(liczEvent).info = 'Set laser pulse counter';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0;
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0;
    Event(liczEvent).seqControl = liczSeq;
        SeqControl(liczSeq).command = 'loopCnt';  %set loop counter
        SeqControl(liczSeq).condition = 'counter1';  %loop counter 1
        SeqControl(liczSeq).argument = pulsesPerTrigger-1; %how many loop iterations will be performed (minus 1, in order to compensate for the first iteration)
    jmpMarkNewFrame = liczEvent;    %frame loop   
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;
    
    Event(liczEvent).info = 'Wait for trigger';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0;
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0;
    Event(liczEvent).seqControl = liczSeq;
        SeqControl(liczSeq).command = 'pause';
        SeqControl(liczSeq).condition = 'extTrigger';
        SeqControl(liczSeq).argument = 17; % Trigger input 1, rising edge
    jmpMarkLaserTrigger = liczEvent;    %laser-only loop
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;

    %No laser triggering, as the laser is triggered by an external signal
    %generator. Thus - just wait...

    Event(liczEvent).info = 'waitBetweenLaserAndUS';    %no US tranmission here, this event just keeps the symmetry
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0; 
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0; 
    Event(liczEvent).seqControl = liczSeq; 
        SeqControl(liczSeq).command = 'noop';
        SeqControl(liczSeq).condition = 'Hw&Sw';
        SeqControl(liczSeq).argument = lasDelay(liczpunkt);
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;
      
        

    Event(liczEvent).info = 'jumpBackToLaser';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0; 
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0; 
    Event(liczEvent).seqControl = liczSeq;
        SeqControl(liczSeq).command = 'loopTst';
        SeqControl(liczSeq).argument = jmpMarkLaserTrigger;  % jump back to  'laserTrigger'
        SeqControl(liczSeq).condition = 'counter1';   
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;
    
    Event(liczEvent).info = 'ReturnToMatlab';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0;
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0;
    Event(liczEvent).seqControl = liczSeq;
        SeqControl(liczSeq).command = 'returnToMatlab';
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;      
    
    %% Next frame: laser + US
    
    Event(liczEvent).info = 'Set laser pulse counter';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0;
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0;
    Event(liczEvent).seqControl = liczSeq;
        SeqControl(liczSeq).command = 'loopCnt';  %set loop counter
        SeqControl(liczSeq).condition = 'counter1';  %loop counter 1
        SeqControl(liczSeq).argument = pulsesPerTrigger-1; %how many loop iterations will be performed (minus 1, in order to compensate for the first iteration)
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;
    
    Event(liczEvent).info = 'Wait for trigger';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0;
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0;
    Event(liczEvent).seqControl = liczSeq;
        SeqControl(liczSeq).command = 'pause';
        SeqControl(liczSeq).condition = 'extTrigger';
        SeqControl(liczSeq).argument = 17; % Trigger input 1, rising edge
    jmpMarklaserBeforeUS = liczEvent;
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;

  
    Event(liczEvent).info = 'waitBetweenLaserAndUS';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0; 
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0; 
    Event(liczEvent).seqControl = liczSeq; 
        SeqControl(liczSeq).command = 'noop';
        SeqControl(liczSeq).condition = 'Hw&Sw';
        SeqControl(liczSeq).argument = lasDelay(liczpunkt);  
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;
    
    
    Event(liczEvent).info = 'USafterLaser';
    Event(liczEvent).tx = liczpunkt; 
    Event(liczEvent).rcv = 0; 
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0; 
    Event(liczEvent).seqControl = 0;
    liczEvent = liczEvent + 1;

    
    Event(liczEvent).info = 'jumpBackToUS';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0; 
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0; 
    Event(liczEvent).seqControl = liczSeq; 
        SeqControl(liczSeq).command = 'loopTst';
        SeqControl(liczSeq).argument = jmpMarklaserBeforeUS;  % jump back to 'USandlaserTrigger'
        SeqControl(liczSeq).condition = 'counter1';  %test loop counter 1
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;   
    
    Event(liczEvent).info = 'ReturnToMatlab';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0;
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0;
    Event(liczEvent).seqControl = liczSeq;
        SeqControl(liczSeq).command = 'returnToMatlab';
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1;      
    
    
    Event(liczEvent).info = 'jumpBackToNextFrameForTheSamePoint';
    Event(liczEvent).tx = 0; 
    Event(liczEvent).rcv = 0; 
    Event(liczEvent).recon = 0; 
    Event(liczEvent).process = 0; 
    Event(liczEvent).seqControl = liczSeq; 
        SeqControl(liczSeq).command = 'loopTst';
        SeqControl(liczSeq).argument = jmpMarkNewFrame;  % jump back to sync before 'CaptureFrameLaserOnly' 
        SeqControl(liczSeq).condition = 'counter2';  %test loop counter 2 - frames per point counter 
    liczEvent = liczEvent + 1;
    liczSeq = liczSeq + 1; 
    
end

       
Event(liczEvent).info = 'BackToStart';
Event(liczEvent).tx = 0; 
Event(liczEvent).rcv = 0; 
Event(liczEvent).recon = 0; 
Event(liczEvent).process = 0; 
Event(liczEvent).seqControl = liczSeq; 
        SeqControl(liczSeq).command = 'jump';  
        SeqControl(liczSeq).argument = 1;  
       
        
        
        
        
        
        
        
        
        
        
        
        
 
    

%Initialize with the high voltage level set
UI(1).Statement =strcat('[result,hv] = setTpcProfileHighVoltage(',num2str(vpeak),',1);'); 
UI(2).Statement ='hv1Sldr = findobj(''Tag'',''hv1Sldr'');'; 
UI(3).Statement ='set(hv1Sldr,''Value'',hv);'; 
UI(4).Statement ='hv1Value = findobj(''Tag'',''hv1Value'');'; 
UI(5).Statement ='set(hv1Value,''String'',num2str(hv,''%.1f''));';    
    
    






    
    
    
    
%% ***********************************************************************
% SAVE CONFIGURATION DATA OF SPECIFIC MEASUREMENT TO FILE
path=cd;
save(strcat(path,'\MatFiles\m4_z',num2str(zStart),'to',num2str(zStop),'_fpp',num2str(fpp),'_ppt',num2str(pulsesPerTrigger),'_hwn',num2str(hwn),'_apod',num2str(apod)));