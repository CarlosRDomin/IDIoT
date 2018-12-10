addpath(genpath('.'));	% Make sure all folders and subfolders are added to the path
cdToThisScriptsDirectory();	% Change directory to the folder containing this script
DATA_FOLDER = '../DataCollection/data';
rePreProcessData = false;

%% Preprocess experiment: varying amplitude in straight line at fixed speed (120 bpm)
outFilename = [DATA_FOLDER '/varyingLinearAmplitude.mat'];
experimentVariable = 'motionLinearAmplitude';
if rePreProcessData || exist(outFilename, 'file')~=2
	[t, dataCam, dataIMU] = loadExperiment(DATA_FOLDER, '2018-12-06 13-52-19');
	experimentStruct = applyTrimToExperiment(t, dataCam, dataIMU, experimentVariable, [3725,4725, 10, 2; 6113,7195, 20, 3; 7475,8590, 30, 3; 9020,10040, 40, 3; 10450,11517, 50, 3; 12011,13090, 60, 3; 13481,15065, 70, 3; 15551,16691, 80, 3; 16920,17940, 90, 3; 18200,19240, 100, 3]);
	save(outFilename, 'experimentStruct', 'experimentVariable', 'dataCam', 'dataIMU', 't');
end

%% Preprocess experiment: varying speed in straight line at fixed amplitude (50 cm)
outFilename = [DATA_FOLDER '/varyingLinearSpeed.mat'];
experimentVariable = 'motionLinearSpeed';
if rePreProcessData || exist(outFilename, 'file')~=2
	[t, dataCam, dataIMU] = loadExperiment(DATA_FOLDER, '2018-12-06 14-08-07');
	experimentStruct = applyTrimToExperiment(t, dataCam, dataIMU, experimentVariable, [2110,4073, 60, 1; 4589,5933, 90, 1; 6425,7500, 120, 1; 8015,8803, 150, 1; 9255,10032, 180, 1; 10517,11117, 210, 1; 11555,12079, 240, 1]);
	save(outFilename, 'experimentStruct', 'experimentVariable', 'dataCam', 'dataIMU', 't');
end

%% Preprocess experiment: varying circular angle at fixed speed (120 bpm)
outFilename = [DATA_FOLDER '/varyingCircularAmplitude.mat'];
experimentVariable = 'motionCircularAmplitude';
if rePreProcessData || exist(outFilename, 'file')~=2
	[t, dataCam, dataIMU] = loadExperiment(DATA_FOLDER, '2018-12-06 14-29-17');
	experimentStruct = applyTrimToExperiment(t, dataCam, dataIMU, experimentVariable, [1445,2455, 30, 5; 2765,3797, 60, 5; 4093,5094, 90, 5; 5615,6630, 120, 5; 7980,9029, 150, 5; 9277,10325, 180, 5]);
	save(outFilename, 'experimentStruct', 'experimentVariable', 'dataCam', 'dataIMU', 't');
end

%% Preprocess experiment: varying speed at fixed circular amplitude (90 deg)
outFilename = [DATA_FOLDER '/varyingCircularSpeed.mat'];
experimentVariable = 'motionCircularSpeed';
if rePreProcessData || exist(outFilename, 'file')~=2
	[t, dataCam, dataIMU] = loadExperiment(DATA_FOLDER, '2018-12-06 14-39-39');
	experimentStruct = applyTrimToExperiment(t, dataCam, dataIMU, experimentVariable, [1130,3149, 60, 1; 3701,5020, 90, 1; 5935,6959, 120, 1; 7470,8322, 150, 1; 8830,9523, 180, 1; 9948,10555, 210, 1; 11027,11548, 240, 1]);
	save(outFilename, 'experimentStruct', 'experimentVariable', 'dataCam', 'dataIMU', 't');
end

%% Preprocess experiment: varying camera angle at fixed speed (120 bpm) and amplitude (50 cm, 90 deg)
outFilenameLinear = [DATA_FOLDER '/varyingLinearCamAngle.mat'];
outFilenameCircular = [DATA_FOLDER '/varyingCircularCamAngle.mat'];
if rePreProcessData || exist(outFilenameLinear, 'file')~=2 || exist(outFilenameCircular, 'file')~=2
	[t, dataCam, dataIMU] = loadExperiment(DATA_FOLDER, '2018-12-07 20-17-41');
	[t2, dataCam2, dataIMU2] = loadExperiment(DATA_FOLDER, '2018-12-07 20-33-34');
	
	% Fixed LINEAR amplitude 50cm
	experimentVariable = 'motionLinearCamAngle';
	experimentStruct = applyTrimToExperiment(t, dataCam, dataIMU, experimentVariable, [1090,2085, 90, 1; 6665,7658, 60, 5; 12490,13510, 30, 5]);
	experimentStruct = applyTrimToExperiment(t2, dataCam2, dataIMU2, experimentVariable, [965,1970, 0, 8], experimentStruct);
	save(outFilenameLinear, 'experimentStruct', 'experimentVariable', 'dataCam', 'dataIMU', 't', 'dataCam2', 'dataIMU2', 't2');
	
	% Fixed CIRCULAR amplitude 90deg
	experimentVariable = 'motionCircularCamAngle';
	experimentStruct = applyTrimToExperiment(t, dataCam, dataIMU, experimentVariable, [2368,3380, 90, 1; 7943,8954, 60, 5; 13880,14902, 30, 5]);
	experimentStruct = applyTrimToExperiment(t2, dataCam2, dataIMU2, experimentVariable, [2355,3358, 0, 8], experimentStruct);
	save(outFilenameCircular, 'experimentStruct', 'experimentVariable', 'dataCam', 'dataIMU', 't', 'dataCam2', 'dataIMU2', 't2');
end

%% Preprocess experiment: varying camera distance at fixed speed (120 bpm) and amplitude (50 cm, 90 deg)
outFilenameLinear = [DATA_FOLDER '/varyingLinearCamDist.mat'];
outFilenameCircular = [DATA_FOLDER '/varyingCircularCamDist.mat'];
if rePreProcessData || exist(outFilenameLinear, 'file')~=2 || exist(outFilenameCircular, 'file')~=2
	[t, dataCam, dataIMU] = loadExperiment(DATA_FOLDER, '2018-12-07 20-56-19');
	[t2, dataCam2, dataIMU2] = loadExperiment(DATA_FOLDER, '2018-12-07 21-13-29');
	
	% Fixed LINEAR amplitude 50cm
	experimentVariable = 'motionLinearCamDist';
	experimentStruct = applyTrimToExperiment(t, dataCam, dataIMU, experimentVariable, [590,1600, 1, 1; 4560,5580, 2, 24; 8515,9611, 3, 3; 12820,13937, 4, 3; 16876,17867, 5, 5]);
	experimentStruct = applyTrimToExperiment(t2, dataCam2, dataIMU2, experimentVariable, [435,1459, 6, 3; 3665,4813, 7, 15; 7440,8463, 8, 15; 12125,13120, 9, 15; 16211,17198, 10, 15], experimentStruct);
	save(outFilenameLinear, 'experimentStruct', 'experimentVariable', 'dataCam', 'dataIMU', 't', 'dataCam2', 'dataIMU2', 't2');
	
	% Fixed CIRCULAR amplitude 90deg
	experimentVariable = 'motionCircularCamDist';
	experimentStruct = applyTrimToExperiment(t, dataCam, dataIMU, experimentVariable, [1980,2993, 1, 2; 5875,6973, 2, 24; 9995,11104, 3, 3; 14183,15177, 4, 3; 18059,19070, 5, 5]);
	experimentStruct = applyTrimToExperiment(t2, dataCam2, dataIMU2, experimentVariable, [1649,2656, 6, 3; 5379,6401, 7, 15; 8651,9662, 8, 15; 13391,14415, 9, 15; 17447,18449, 10, 15], experimentStruct);
	save(outFilenameCircular, 'experimentStruct', 'experimentVariable', 'dataCam', 'dataIMU', 't', 'dataCam2', 'dataIMU2', 't2');
end

%% Helper functions

function [t, dataCam, dataIMU] = loadExperiment(DATA_FOLDER, experimentStartT, manualDeltaT)
	if nargin<3 || isempty(manualDeltaT), manualDeltaT = 0; end  % Manual offset between IMU and cam time-sync, in seconds

	imuPrefix = 'BNO055_1';
	camPrefix = 'cam_1';

	dataIMU = loadOurExperimentIMU([DATA_FOLDER '/' experimentStartT '/' imuPrefix '_' experimentStartT '.h5']);
	dataCam = getPosFromCam([DATA_FOLDER '/' experimentStartT '/' camPrefix '_' experimentStartT '.h5']);
	dataCam.params.cam.intrinsicMat = [1400 0 909; 0 1400 490; 0 0 1];  % Manually update the intrinsic matrix
	
	%dt = diff(dataCam.t_frames);
	dataIMU.params.t = dataIMU.params.t + manualDeltaT; % Manual time-sync
	t = dataCam.t_frames-dataCam.t_frames(1);
end

function experimentStruct = applyTrimToExperiment(t, dataCam, dataIMU, experimentVariableName, experimentTrimTimes, experimentStruct)
% experimentTrimTimes is an Nx4 matrix where 1st column is tStart, 2nd is tEnd, 3rd is experiment variable value (e.g. the amplitude in cm of each experiment), 4th is personID (ignore data from other people)
	if nargin<6 || isempty(experimentStruct), experimentStruct = struct(experimentVariableName,{}, 'dataCam',{}, 'dataIMU',{}); end

	w = waitbar(0, sprintf('Processing %s experiments...', experimentVariableName), 'Name', experimentVariableName);
	for i = 1:size(experimentTrimTimes,1)
		tt = t(experimentTrimTimes(i,1):experimentTrimTimes(i,2));  % Select the time indices specified by experimentTrimTimes
		[dataCamTrimmed, dataIMUtrimmed] = resampleCamAndIMU(tt, dataCam, dataIMU, true);
		dataCamTrimmed.camPos = dataCamTrimmed.camPos(experimentTrimTimes(i,4));  % Only keep info of specified personID (experimentTrimTimes(i,4))
		dataCamTrimmed.t_frames = tt; dataIMUtrimmed.params.t = tt; dataIMUtrimmed.params.tStart = tt(1);  % Update times so they're all trimmed too
		experimentStruct(end+1,:) = struct(experimentVariableName,experimentTrimTimes(i,3), 'dataCam',dataCamTrimmed, 'dataIMU',dataIMUtrimmed);
		waitbar(i/size(experimentTrimTimes,1), w, sprintf('Processing %s experiments... %2d/%d', experimentVariableName, i, size(experimentTrimTimes,1)));
		
		if false
			scoreIMUtoCamBodyPart = computeSimilarityMatrix(dataIMUtrimmed, dataCamTrimmed, 1, true);
			figure; imshow(scoreIMUtoCamBodyPart, 'InitialMagnification',10000); colormap(gray);
			figure; [m,j]=min(scoreIMUtoCamBodyPart); plot(scoreIMUtoCamBodyPart); hold on; plot(j,m, 'r*', 'MarkerSize',20); xlim([1 size(scoreIMUtoCamBodyPart,2)]);
		end
	end
	close(w);
end