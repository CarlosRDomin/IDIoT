addpath(genpath('.'));	% Make sure all folders and subfolders are added to the path
cdToThisScriptsDirectory();	% Change directory to the folder containing this script
DATA_FOLDER = '../DataCollection/data';
experimentStartT = '2018-12-06 14-39-39';
imuPrefix = 'BNO055_1';
camPrefix = 'cam_1';

%% Load h5 files and process wrist positions from camera
if ~exist('dataIMU', 'var')
	dataIMU = loadOurExperimentIMU([DATA_FOLDER '/' experimentStartT '/' imuPrefix '_' experimentStartT '.h5']);
end

if ~exist('dataCam', 'var')
	dataCam = getPosFromCam([DATA_FOLDER '/' experimentStartT '/' camPrefix '_' experimentStartT '.h5']);
	dataCam.params.cam.intrinsicMat = [1400 0 909; 0 1400 490; 0 0 1];  % Manually update the intrinsic matrix
end

dt = diff(dataCam.t_frames);
dataIMU.params.t = dataIMU.params.t; % Manual time-sync
t = mean(dt).*(0:length(dt)-1); %t = 0:mean(dt):40;%dataCam.t_frames(end)-dataCam.t_frames(1);
t = dataCam.t_frames-dataCam.t_frames(1);
[dataCam, dataIMU] = resampleCamAndIMU(t, dataCam, dataIMU);
return;

%% 2D tracked position animation
% Setup 2D position figure
figure('Name', 'Wrist positions');
h = scatter(0,0, 'r', 'SizeData', 500);
xlim([0 dataCam.width]); ylim([0 dataCam.height]);

% Animate the position over time
for i = 1:length(pCam)
	set(h, 'XData', pCam(i,1)+1);
	set(h, 'YData', dataCam.height-pCam(i,2));
	pause(dt(i));
end

%% 2D tracked position animation
% Setup 2D position figure
figure('Name', 'Wrist positions'); hold on;
hWrist = scatter(0,0, 'SizeData', 500);
hElbow = scatter(0,0, 'SizeData', 500);
axis([0 dataCam.width 0 dataCam.height]);

% Animate the position over time
for i = 1:length(pCam)
	set(hWrist, 'XData', pCam(i,1)+1);
	%set(hWrist, 'YData', dataCam.height-pCam(i,2));
	set(hWrist, 'YData', pCam(i,2)+1);
	set(hElbow, 'XData', pElbowCam(i,1)+1);
	set(hElbow, 'YData', pElbowCam(i,2)+1);
	pause(dt(i));
end

%% Find 3D rotation and animate orientations
% Find best time shift and rotation (align frames of coordinates)
joints = {'rWrist', 'rElbow'};
personID = 4;

[score, R, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted, deltaT] = find_shift_and_alignment([dataCam.camPos(personID).(joints{1}).pos2Dresamp dataCam.camPos(personID).(joints{2}).pos2Dresamp], dataIMU.(IMUlocation).forwardResamp, dataCam.params.cam.intrinsicMat);
tShifted = shiftSignalBy(deltaT, t, false, 2);
limbLengthInCam = sqrt(sum((pCamResamp.(joints{1})-pCamResamp.(joints{2})).^2, 2));
limbLengthInCamShifted = shiftSignalBy(deltaT, limbLengthInCam);
proj = dot(nCamOrientationShifted, orientationIoTtoCamShifted, 2);

figure('Name', 'CamPlane vs IoT orientation angle');
subplot(2,1,1); plot(tShifted, asind(dot(nCamOrientationShifted, orientationIoTtoCamShifted, 2))); hold on; plot(tShifted, asind(abs(dot(nCamOrientationShifted, orientationIoTtoCamShifted, 2))), '--'); stem(tShifted, 90.*(limbLengthInCamShifted<100), '--'); title(['Score: ' num2str(score)]);
subplot(2,1,2); plot(tShifted, limbLengthInCamShifted);

% Setup 3D orientation figure
figure('Name', 'IMU orientation');

subplot(1,2,1); hold on; grid on;
hIoT3D = plot3(0,0,0, 'LineWidth', 5);
hCam3D = plot3(0,0,0, 'LineWidth', 5);
axis([-1 1 -1 1 -1 1]);

subplot(1,2,2); hold on; grid on;
hIoT2D = plot([0 1],[0 0], 'LineWidth', 5);
hCam2D = plot(0,0, 'LineWidth', 5);
axis([-1 1 -1 1]);

% Animate the orientation over time
for i = 1:length(orientationIoTtoCamShifted)
	set(hIoT3D, 'XData', [0 orientationIoTtoCamShifted(i,1)]);
	set(hIoT3D, 'YData', [0 orientationIoTtoCamShifted(i,2)]);
	set(hIoT3D, 'ZData', [0 orientationIoTtoCamShifted(i,3)]);
	set(hCam3D, 'XData', [0 nCamOrientationShifted(i,1)]);
	set(hCam3D, 'YData', [0 nCamOrientationShifted(i,2)]);
	set(hCam3D, 'ZData', [0 nCamOrientationShifted(i,3)]);
	set(hCam2D, 'XData', [0 proj(i)]);
	set(hCam2D, 'YData', [0 sqrt(1 - proj(i).^2)]);
	pause(dt(i));
end

%%
% Setup 3D skeleton figure
figure('Name', '3D skeleton');
hCam3D = plot3(0,0,0, '.-', 'MarkerSize',20, 'LineWidth', 2);
hCam3Ddata = zeros(length(jointNames), 3);
drawingOrder = [19, 17, 1, 16, 18, 16, 1, 2, 3, 4, 5, 4, 3, 9, 6, 2, 6, 7, 8, 7, 6, 9, 10, 11, 12, 25, 12, 23, 24, 23, 12, 11, 10, 9, 13, 14, 15, 22, 15, 20, 21];
jointNamesTest = {'nose1', 'neck2', 'rShoulder3', 'rElbow4', 'rWrist5', 'lShoulder6', 'lElbow7',...
	'lWrist8', 'midHip9', 'rHip10', 'rKnee11', 'rAnkle12', 'lHip13', 'lKnee14', 'lAnkle15', ...
	'rEye16', 'lEye17', 'rEar18', 'lEar19', 'lBigToe20', 'lSmallToe21', 'lHeel22', 'rBigToe23', 'rSmallToe24', 'rHeel25'};
axis(1500*repmat([-1 1], 1,3));

% Animate estimated 3D skeleton
for i = 1:length(t)
	for j = 1:length(jointNames)
		hCam3Ddata(j,:) = pCam3Dresamp.(jointNames{j})(i,:);
	end
	set(hCam3D, 'XData', hCam3Ddata(drawingOrder,1));
	set(hCam3D, 'YData', hCam3Ddata(drawingOrder,2));
	set(hCam3D, 'ZData', hCam3Ddata(drawingOrder,3));
	pause(dt(i));
end

%% Load video into memory
v = VideoReader([DATA_FOLDER '/' experimentStartT '/' camPrefix '_' experimentStartT '.mp4']);
totalFrames = int64(v.Duration*v.FrameRate);
maxFrames = 1000;  % Only process the first 1000 frames
imgCam = zeros(maxFrames, v.Height/4,v.Width/4,3,'uint8');
i = 1;
while i <= maxFrames %v.hasFrame
	imgCam(i, :,:,:) = imresize(v.readFrame, 1/4);
	i = i+1;
	if mod(i,100) == 0, fprintf('Read: %d frames\r', i); end
end

%%
personID = 1;
[scoreIMUtoCamBodyPart, bodyPartToJointsAssoc] = computeSimilarityMatrix(dataIMU, dataCam, personID, true);
a = assignDetectionsToTracks(scoreIMUtoCamBodyPart, 1);
assignedCamBodyPart = zeros(1,length(a)); assignedCamBodyPart(a(:,1))=a(:,2);
figure; imshow(scoreIMUtoCamBodyPart, 'InitialMagnification',10000); colormap(jet);
figure; for i = 1:length(bodyPartToJointsAssoc), [m,j]=min(scoreIMUtoCamBodyPart(i,:)); subplot(length(bodyPartToJointsAssoc),1,i); plot(rad2deg(scoreIMUtoCamBodyPart(i,:))); hold on; plot(i,0, 'g*', 'MarkerSize',20); plot(j,m, 'r*', 'MarkerSize',20); plot(assignedCamBodyPart(i),0, 'b*', 'MarkerSize',20); ylim([0 15]); xlim([0 size(scoreIMUtoCamBodyPart,2)+1]); title(bodyPartToJointsAssoc(i).bodyPart); end

%%
joints = {'rElbow', 'rWrist'};
IMUlocation = 'R_LowArm'; personID = 1;
dataIMU.(IMUlocation).forwardResamp = dataIMU.(IMUlocation).forwardResamp./sqrt(sum(dataIMU.(IMUlocation).forwardResamp.^2,2));
[score, R, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted, deltaT] = find_shift_and_alignment([dataCam.camPos(personID).(joints{1}).pos2D dataCam.camPos(personID).(joints{2}).pos2D], dataIMU.(IMUlocation).forwardResamp, dataCam.params.cam.intrinsicMat);

%% Visualize orientation together with video
% Setup 3D orientation figure
figure('Name', 'IMU orientation');

IMUlocation = 'R_LowArm';
% v = VideoReader([DATA_FOLDER '/' experimentStartT '/' camPrefix '_' experimentStartT '.mp4']);
imgCam = [];

subplot(3,6,[1,14]); hold on; grid on; S = [5 2 3]/5; % Cube dimensions
[hIoT3D, Pcube] = plotCube(S, -S/2, 0.8, [0 1 0]); view(0,-90);
% hIoT3D = plot3(0,0,0, 'LineWidth', 5);
% hCam3D = plot3(0,0,0, 'LineWidth', 5);
axis([-1 1 -1 1 -1 1]); xlabel('x'); ylabel('y'); zlabel('z');

subplot(3,6,3); hold on; grid on;
hIoT3Dx = plot([0 1],[0 0], 'LineWidth', 5);
axis([-1 1 -1 1]); xlabel('x');

subplot(3,6,9); hold on; grid on;
hIoT3Dy = plot([0 1],[0 0], 'LineWidth', 5);
axis([-1 1 -1 1]); xlabel('y');

subplot(3,6,15); hold on; grid on;
hIoT3Dz = plot([0 1],[0 0], 'LineWidth', 5);
axis([-1 1 -1 1]); xlabel('z');

subplot(3,6,4); hold on; grid on;
hIoT3Dxy = plot([0 1],[0 0], 'LineWidth', 5);
axis([-1 1 -1 1]); xlabel('x'); ylabel('y');

subplot(3,6,10); hold on; grid on;
hIoT3Dyz = plot([0 1],[0 0], 'LineWidth', 5);
axis([-1 1 -1 1]); xlabel('z'); ylabel('y');

subplot(3,6,16); hold on; grid on;
hIoT3Dxz = plot([0 1],[0 0], 'LineWidth', 5);
axis([-1 1 -1 1]); xlabel('x'); ylabel('z');

timerViz = timer('BusyMode','drop', 'ExecutionMode','fixedRate', 'Period',round(1000*mean(dt))/1000);
[~, hSlider, hPlayPause] = addAnimationTimeControls(t, @(tInd)updateFigAtT(tInd, hIoT3D, hIoT3Dx, hIoT3Dy, hIoT3Dz, hIoT3Dxy, hIoT3Dyz, hIoT3Dxz, imgCam, orientationIoTtoCamShifted, dataIMU.(IMUlocation).quat(1:4:end), Pcube, R), @(shouldPlay)onPlayPause(shouldPlay, timerViz));
timerViz.TimerFcn = @(src,event)vizTick(src, hSlider, hPlayPause);
hPlayPause.notify('Action');  % 'Play' animation

function updateFigAtT(tInd, hIoT3D, hIoT3Dx, hIoT3Dy, hIoT3Dz, hIoT3Dxy, hIoT3Dyz, hIoT3Dxz, imgCam, v, q, Pcube, R)
% 	v = dataIMU.(IMUlocation).forwardResamp;
	%subplot(3,6,[5,18]); if false, imshow(imresize(readFrame(v, tInd), 1/4)); else, imshow(squeeze(imgCam(tInd,:,:,:))); end
% 	set(hIoT3D, 'XData', [0 v(tInd,1)]);
% 	set(hIoT3D, 'YData', [0 v(tInd,2)]);
% 	set(hIoT3D, 'ZData', [0 v(tInd,3)]);
	v(tInd,:) = rotatepoint(q(tInd), [1 0 0]);
	v(tInd,:) = (R*rotatepoint(q(tInd), [1 0 0])')';
	for i = 1:length(hIoT3D)
		P = rotatepoint(q(tInd), Pcube(:,:,i));
		P = (R*rotatepoint(q(tInd), Pcube(:,:,i))')';
		set(hIoT3D(i), 'XData', P(:,1));
		set(hIoT3D(i), 'YData', P(:,2));
		set(hIoT3D(i), 'ZData', P(:,3));
	end
	set(hIoT3Dx, 'XData', [0 v(tInd,1)]);
	set(hIoT3Dy, 'XData', [0 v(tInd,2)]);
	set(hIoT3Dz, 'XData', [0 v(tInd,3)]);
	set(hIoT3Dxy, 'XData', [0 v(tInd,1)]);
	set(hIoT3Dxy, 'YData', [0 v(tInd,2)]);
	set(hIoT3Dyz, 'XData', [0 v(tInd,3)]);
	set(hIoT3Dyz, 'YData', [0 v(tInd,2)]);
	set(hIoT3Dxz, 'XData', [0 v(tInd,1)]);
	set(hIoT3Dxz, 'YData', [0 v(tInd,3)]);
end

function vizTick(timerViz, hSlider, hPlayPause)
	if hSlider.Value < hSlider.Max  % Show next frame
		hSlider.Value = hSlider.Value + 1;
		hSlider.notify('Action');
	end
	if hSlider.Value >= hSlider.Max  % Stop the timer if we got to the end
		stop(timerViz);
		hPlayPause.notify('Action');
	end
end

function onPlayPause(shouldPlay, timerViz)
	if shouldPlay == true
		timerViz.start();
	else
		stop(timerViz);
	end
end

function img = readFrame(v, frameNumber)
	v.CurrentTime = (frameNumber-1)/v.FrameRate;
	img = v.readFrame;
end
