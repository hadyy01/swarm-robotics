%% Robot Trail Projection System
%  Active Matter Lab, Department of Physics, IIT Bombay
%
%  Author  : Hady Khan
%  Advisor : Prof. Dr. Nitin Kumar, IIT Bombay
%
%  Description:
%    Projects a persistent light trail onto the arena floor using an overhead
%    projector driven by a GigE camera feed. The system:
%      1. Captures frames from a GigE camera at 5 fps.
%      2. Detects robot centroids via blob analysis on thresholded grayscale frames.
%      3. Accumulates a rolling history of centroid positions (TrailLength frames).
%      4. Projects the trail as filled blue circles via a fullscreen video player.
%
%    Robots equipped with LDR sensors detect the projected light trail and
%    transition from free ABP motion into coordinated trail-following, inducing
%    emergent flocking without direct inter-robot communication.
%
%  Setup:
%    - Set the projector as the PRIMARY display before running.
%    - Calibrate `croprect` to match your arena boundaries.
%    - Adjust `vid.ExposureTime` and `vid.Gain` for your lighting conditions.
%
%  Hardware:
%    - GigE camera (index 1, Mono8 pixel format)
%    - Overhead projector (1920x1080, fullscreen)
%    - Arena with retroreflective markers on robot tops

%%%%%% MAKE PROJECTOR THE PRIMARY DISPLAY BEFORE RUNNING %%%%%
clc;
clear;
%%
% INPUT
croprect = [127.510000000000,59.5100000000000,1821.98000000000,1411.98000000000];
TrailLength = 300;

% Read the camera
vid = gigecam(1, 'PixelFormat', 'Mono8');
% vid = gigecam(1, 'PixelFormat', 'BayerRG8');

% Create a BlobAnalysis system object
hBlob = vision.BlobAnalysis('OrientationOutputPort',true,'BoundingBoxOutputPort',false,'MinimumBlobArea',270,'ExcludeBorderBlobs',true);

% Video player
depVideoPlayer = vision.DeployableVideoPlayer('Size','Full-screen');

%% Change camera properties
vid.PacketSize = 8000;
vid.PacketDelay = 8000;
vid.Timeout = 20;
vid.AcquisitionFrameRateAuto = 'off';
vid.ExposureAuto = 'off';
vid.GainAuto = 'off';
vid.AcquisitionFrameRate = 5;
vid.ExposureTime = 50000;
vid.Gain = 2;

%% Create background image
I = zeros(1080,1920);

for i = 1:(1080)
    for j = 240:(1920-290)
        I(i,j) = 1;
    end
end
%%
%USER INPUT
% prompt = "Put trail length ";
% TrailLength = input(prompt);


%% Video Projection Loop
cont = 1;
p = 1;
M = [0 0 0];
% Make screen white
Imask = insertShape(I,'FilledCircle',[0 0 0],'Color','yellow');
depVideoPlayer(Imask);
pause(1);

%%
while cont
    % Snapshot
    X = vid.snapshot;
    %Crop image
    crop = imcrop(X,croprect);
    % Segmentation
    BW = crop < 70;
    % Morphological operation
    disk = strel('disk',4);
    BWerode = imerode(BW,disk);
    BWerode = imerode(BWerode,disk);
    BWerode = imdilate(BWerode,disk);
    BWerode = imdilate(BWerode,disk);
    % Position detection
    [~, objCentroid, ~] = step(hBlob,BWerode);
%     % Shift axes for GigE image
%     objCentroid(:,1) = objCentroid(:,1) - croprect(1);
%     objCentroid(:,2) = objCentroid(:,2) - croprect(2);
    % Unit change pixel to mm
    OnePixel = 1080/croprect(4);
    objCentroid = objCentroid .* OnePixel;
    % Shift axes for projector
    objCentroid(:,1) = objCentroid(:,1) + 240;
    objCentroid(:,2) = objCentroid(:,2) + 0;
    % Trail creation
    r = ones(length(objCentroid(:,1)),1);
    r(:,:) = 20; % Trail size
    M = [M; [objCentroid r]];
    % Trail length
    L = TrailLength;%*length(objCentroid(:,1));
    if length(M(:,1)) > L
        M = M(end-L:end,:);
    else
        M = M;
    end
    % Insert trail
    Imask = insertShape(I,'FilledCircle',M,'Color','blue','Opacity',0.6);
    % Display video
    depVideoPlayer(Imask);
    % Check videoPlayer is open or not
    cont = isOpen(depVideoPlayer);
end

% Release videoPlayer
release(depVideoPlayer);

% % Position Data
% M = M(2:end,1:2);
% t = [1:length(M(:,1))]'/vid.AcquisitionFrameRate;
% Pos = [t M];
