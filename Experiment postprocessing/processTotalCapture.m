addpath(genpath('.'));	% Make sure all folders and subfolders are added to the path
cdToThisScriptsDirectory();	% Change directory to the folder containing this script
DATA_FOLDER = '../TotalCapture';
FIGURE_FOLDER = '../Paper';
dt = 1/60;  % The dataset is fully synchronized at 60fps
bodyPartToJointsAssoc = cell2struct([{'Head', 'rEye','lEye'}; {'Sternum', 'rShoulder','lShoulder'}; {'Pelvis', 'rHip','lHip'}; {'L_UpArm', 'lShoulder','lElbow'}; {'R_UpArm', 'rShoulder','rElbow'}; {'L_LowArm', 'lElbow','lWrist'}; {'R_LowArm', 'rElbow','rWrist'}; {'L_UpLeg', 'lHip','lKnee'}; {'R_UpLeg', 'rHip','rKnee'}; {'L_LowLeg', 'lKnee','lAnkle'}; {'R_LowLeg', 'rKnee','rAnkle'}; {'L_Foot', 'lAnkle','lBigToe'}; {'R_Foot', 'rAnkle','rBigToe'}], {'bodyPart', 'joint1', 'joint2'}, 2);
camCalib = readTotalCaptureCamCalib([DATA_FOLDER '/camCalibration.txt']);
reloadData = false;

%% Load h5 files and process each person's joint positions from camera and IMU orientation
for iSubj = 1:5
	folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];  % Folder where all the data for subject s_i is
	activities = {'acting1', 'acting2', 'acting3', 'freestyle1', 'freestyle2', 'freestyle3', 'rom1', 'rom2', 'rom3', 'walking1', 'walking2', 'walking3'};
	for iActivity = 1:length(activities)
		activity = activities{iActivity};
		activityIMUfile = [folderPrefix '/' activity '_Xsens_AuxFields.sensors'];
		activityProcessedFile = [folderPrefix '/' activity '_processed.mat'];
		if exist([folderPrefix '/' activity], 'file')~=7  || exist(activityIMUfile, 'file')~=2  % Make sure folder exists (folder=7) as well as IMU data (file=2)
			fprintf('Couldn''t find folder %s or file %s_Xsens_AuxFields.sensors for subject s%d\n', activity, activity, iSubj);
			continue;
		end
		
		needToProcessData = (exist(activityProcessedFile, 'file')~=2);  % Determine whether this activity of the dataset has already been processed and saved as a .mat
		% Load or process IMU data
		if ~needToProcessData
			load(activityProcessedFile);
		else
			dataIMU = readXsens([folderPrefix '/' activity '_Xsens_AuxFields.sensors']);
			clear dataCams;
		end
		namesIMUs = setdiff(fieldnames(dataIMU), 'params', 'stable');
		t = 0 : dt : (length(dataIMU.(namesIMUs{1}).quat)-1)*dt;

		for iCam = 1:8
			% Process cam data if needed (otherwise it has already been loaded)
			activityCamFile = [folderPrefix '/' activity '/TC_S' num2str(iSubj) '_' activity '_cam' num2str(iCam) '.h5'];
			if needToProcessData || length(dataCams)<iCam
				if exist(activityCamFile, 'file')~=2  % Make sure cam file exists (I haven't processed some camera angles [on purpose]) otherwise just ignore this
					fprintf('Couldn''t find file %s, skipping!\n', activityCamFile);
					continue;
				end
				aux = getPosFromCam(activityCamFile);
				aux.fps = 1/dt;
				aux.params.cam = camCalib(iCam);
				dataCams(iCam) = aux;
				save(activityProcessedFile, 'dataIMU', 'dataCams');
			end
			continue;  % TEMPORARY, DELETE AFTER PROCESSING DATASET

			[scoreIMUtoCamBodyPart, bodyPartToJointsAssoc] = computeSimilarityMatrix(dataIMU, dataCams(iCam), [], [], 1);
			normalizedScore = sinkhornKnopp(scoreIMUtoCamBodyPart(1:13,1:13));
			a = assignDetectionsToTracks(scoreIMUtoCamBodyPart, max(scoreIMUtoCamBodyPart(:))+1);
			assignedCamBodyPart = zeros(1,length(a)); assignedCamBodyPart(a(:,1))=a(:,2);
			figure; imshow((max(scoreIMUtoCamBodyPart(:))-scoreIMUtoCamBodyPart)./(max(scoreIMUtoCamBodyPart(:))-min(scoreIMUtoCamBodyPart(:))), 'InitialMagnification',10000); colormap('bone');
			figure; imshow((max(normalizedScore(:))-normalizedScore)./(max(normalizedScore(:))-min(normalizedScore(:))), 'InitialMagnification',10000); colormap('bone');
			figure; for i = 1:length(bodyPartToJointsAssoc), [m,j]=min(scoreIMUtoCamBodyPart(i,:)); subplot(length(bodyPartToJointsAssoc),1,i); plot(scoreIMUtoCamBodyPart(i,:)); hold on; plot(i,0, 'g*', 'MarkerSize',20); plot(j,m, 'r*', 'MarkerSize',20); plot(assignedCamBodyPart(i),0, 'b*', 'MarkerSize',20); ylim([0,1]); xlim([0 size(scoreIMUtoCamBodyPart,2)+1]); title(bodyPartToJointsAssoc(i).bodyPart); end
		end
	end
end
return;

%% Plot example confusion matrices
legendStr = {'IDIoT', '3D accel matching', '3D orientation matching'};
outputFigFilenames = {'confusionIDIoT', 'confusion3Daccel', 'confusion3Dorient'};
for activity = {'rom1', 'walking1', 'acting1', 'freestyle1'}
	load([DATA_FOLDER '/s1/' activity{:} '_processed.mat']);
	windowLengths = 5:5:30;
	personID = 1;
	bodyPartToJointsAssocInds = 4:7;  % {'L_UpArm', 'R_UpArm', 'L_LowArm', 'R_LowArm'}
	t = (0:(length(dataCams(1).camPos(personID).rWrist.pos2D)-1))./dataCams(1).fps;
	for iW = 1:length(windowLengths)
		tEnd = windowLengths(iW);
		tInds = (t < tEnd);
		for iMethod = 1:3
			outputFigFilename = [outputFigFilenames{iMethod} '_' activity{:} '_t' num2str(tEnd)];
			xLabelStr = sprintf('Confusion matrix after t=%ds for %s', tEnd, legendStr{iMethod});
			confusionMatrix = computeSimilarityMatrix(dataIMU, dataCams(1), personID, false, iMethod, bodyPartToJointsAssocInds, bodyPartToJointsAssocInds, tInds);
			h = figure('Name', xLabelStr); imshow(confusionMatrix, 'InitialMagnification',10000); colormap(flipud(gray));
			caxis([0, max(0.1, max(confusionMatrix(:)))]); colorbar;
			savefig(h, [FIGURE_FOLDER '/' outputFigFilename '.fig']);
			saveas(h, [FIGURE_FOLDER '/' outputFigFilename '.eps'], 'epsc');
		end
	end
end
return;

%% 2D tracked position animation
% Setup 2D position figure
figure('Name', 'Wrist positions');
h = scatter(0,0, 'r', 'SizeData', 500);
xlim([0 hfCam.width]); ylim([0 hfCam.height]);

% Animate the position over time
for i = 1:length(pCam)
	set(h, 'XData', pCam(i,1)+1);
	set(h, 'YData', hfCam.height-pCam(i,2));
	pause(dt(i));
end

%% 2D tracked position animation
% Setup 2D position figure
figure('Name', 'Wrist positions'); hold on;
hWrist = scatter(0,0, 'SizeData', 500);
hElbow = scatter(0,0, 'SizeData', 500);
axis([0 hfCam.width 0 hfCam.height]);

% Animate the position over time
for i = 1:length(pCam)
	set(hWrist, 'XData', pCam(i,1)+1);
	%set(hWrist, 'YData', hfCam.height-pCam(i,2));
	set(hWrist, 'YData', pCam(i,2)+1);
	set(hElbow, 'XData', pElbowCam(i,1)+1);
	set(hElbow, 'YData', pElbowCam(i,2)+1);
	pause(dt(i));
end

%% Find 3D rotation and animate orientations
joints = {'rWrist', 'rElbow'}; IMUlocation = 'R_LowArm'; personID=1; iIMU=7;iBodyJointPair=7; dataCam = dataCams(1);
[score, R, nCamOrientationShifted, ~, orientationIoTtoCamShifted] = find_shift_and_alignment([dataCams(iCam).camPos(personID).(joints{1}).pos2D dataCams(iCam).camPos(personID).(joints{2}).pos2D], rotatepoint(dataIMU.(IMUlocation).quat, [1 0 0]), dataCams(iCam).params.cam.intrinsicMat, 0);

limbLengthInCam = sqrt(sum((dataCams(iCam).camPos(personID).(joints{1}).pos2D-dataCams(iCam).camPos(personID).(joints{2}).pos2D).^2, 2));
proj = dot(nCamOrientationShifted, orientationIoTtoCamShifted, 2);

figure('Name', 'CamPlane vs IoT orientation angle');
subplot(2,1,1); plot(t, asind(dot(nCamOrientationShifted, orientationIoTtoCamShifted, 2))); hold on; plot(t, asind(abs(dot(nCamOrientationShifted, orientationIoTtoCamShifted, 2))), '--'); stem(t, 90.*(limbLengthInCam<100), '--'); title(['Score: ' num2str(score)]);
subplot(2,1,2); plot(t, limbLengthInCam);

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
	pause(dt);
end

%%
% Setup 3D skeleton figure
figure('Name', '3D skeleton');
hCam3D = plot3(0,0,0, '.-', 'MarkerSize',20, 'LineWidth', 2);
hCam3Ddata = zeros(length(jointNames), 3);
drawingOrder = [19, 17, 1, 16, 18, 16, 1, 2, 3, 4, 5, 4, 3, 9, 6, 2, 6, 7, 8, 7, 6, 9, 10, 11, 12, 25, 12, 23, 24, 23, 12, 11, 10, 9, 13, 14, 15, 22, 15, 20, 21];
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

%% Visualize orientation together with video
personID=1; iCam=1;iIMU=7;iBodyJointPair=8;
posCamJoints = [dataCams(iCam).camPos(personID).(bodyPartToJointsAssoc(iBodyJointPair).joint1).pos2D dataCams(iCam).camPos(personID).(bodyPartToJointsAssoc(iBodyJointPair).joint2).pos2D];
[score, qOffset, vAxis, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted, deltaT] = find_shift_and_alignment(posCamJoints, dataIMU.(namesIMUs{iIMU}).quat, dataCams(iCam).params.cam.intrinsicMat, 0);
IMUlocation = 'R_LowArm';
visualizeCamIMUpair(t, orientationIoTtoCamShifted, nCamOrientationShifted, posCamJoints, [], vAxis);
return;

% Setup 3D orientation figure
figure('Name', 'IMU orientation');
v = VideoReader([folderPrefix '/' activity '/TC_S' num2str(iSubj) '_' activity '_cam' num2str(iCam) '_rendered.mp4']);

subplot(3,6,[1,14]); hold on; grid on; S = [5 2 3]/5; % Cube dimensions
[hIoT3D, Pcube] = plotCube(S, -S/2, 0.8, [0 1 0]);
% hIoT3D = plot3(0,0,0, 'LineWidth', 5);
% hCam3D = plot3(0,0,0, 'LineWidth', 5);
axis([-1 1 -1 1 -1 1]); xlabel('x'); ylabel('y'); zlabel('z'); view(0,-90);

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
[~, hSlider, hPlayPause] = addAnimationTimeControls(t, @(tInd)updateFigAtT(tInd, hIoT3D, hIoT3Dx, hIoT3Dy, hIoT3Dz, hIoT3Dxy, hIoT3Dyz, hIoT3Dxz, v, dataIMU.(IMUlocation).quat, Pcube, R), @(shouldPlay)onPlayPause(shouldPlay, timerViz));
timerViz.TimerFcn = @(src,event)vizTick(src, hSlider, hPlayPause);
% hPlayPause.notify('Action');  % 'Play' animation

function updateFigAtT(tInd, hIoT3D, hIoT3Dx, hIoT3Dy, hIoT3Dz, hIoT3Dxy, hIoT3Dyz, hIoT3Dxz, v, q, Pcube, R)
 	%subplot(3,6,[5,18]); if true, imshow(imresize(readFrame(v, tInd), 1/4)); else, imshow(squeeze(imgCam(tInd,:,:,:))); end
	calib_imu_bone = quaternion(circshift(str2double(strsplit('0.00848931 -0.987702 0.154796 -0.020277', ' ')), 1));
	calib_imu_ref = quaternion(circshift(str2double(strsplit('-0.698693 -0.000693276 0.030256 0.714781', ' ')), 1));
	qq = calib_imu_ref*q(tInd)*calib_imu_bone.conj;
	vector = rotatepoint(qq, [1 0 0]);
	vector = (R*rotatepoint(q(tInd), [1 0 0])')';
	for i = 1:length(hIoT3D)
		P = rotatepoint(qq, Pcube(:,:,i));
		P = (R*rotatepoint(q(tInd), Pcube(:,:,i))')';
		set(hIoT3D(i), 'XData', P(:,1));
		set(hIoT3D(i), 'YData', P(:,2));
		set(hIoT3D(i), 'ZData', P(:,3));
	end
	set(hIoT3Dx, 'XData', [0 vector(1)]);
	set(hIoT3Dy, 'XData', [0 vector(2)]);
	set(hIoT3Dz, 'XData', [0 vector(3)]);
	set(hIoT3Dxy, 'XData', [0 vector(1)]);
	set(hIoT3Dxy, 'YData', [0 vector(2)]);
	set(hIoT3Dyz, 'XData', [0 vector(3)]);
	set(hIoT3Dyz, 'YData', [0 vector(2)]);
	set(hIoT3Dxz, 'XData', [0 vector(1)]);
	set(hIoT3Dxz, 'YData', [0 vector(3)]);
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
