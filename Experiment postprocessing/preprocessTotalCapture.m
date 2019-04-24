addpath(genpath('.'));	% Make sure all folders and subfolders are added to the path
cdToThisScriptsDirectory();	% Change directory to the folder containing this script
DATA_FOLDER = '../TotalCapture';
FIGURE_FOLDER = '../Paper';
dt = 1/60;  % The dataset is fully synchronized at 60fps
camCalib = readTotalCaptureCamCalib([DATA_FOLDER '/camCalibration.txt']);
tWindowOverlap = 0.5;  % Percent (0-1) of overlap between consecutive windows
bodyPartToJointsAssoc = cell2struct([{'Head', 'lEar','rEar', 0.5}; {'Sternum', 'neck','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
rePreProcessData = false;

%% Load h5 files and process each person's joint positions from camera and IMU orientation
for iSubj = 1:5
	folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];  % Folder where all the data for subject s_i is
	activities = {'acting1', 'acting2', 'acting3', 'freestyle1', 'freestyle2', 'freestyle3', 'rom1', 'rom2', 'rom3', 'walking1', 'walking2', 'walking3'};
	for iActivity = 1:length(activities)
		activity = activities{iActivity};
		activityIMUfile = [folderPrefix '/' activity '_Xsens_AuxFields.sensors'];
		activityPreProcessedFile = [folderPrefix '/' activity '_preprocessed.mat'];
		if exist([folderPrefix '/' activity], 'file')~=7  || exist(activityIMUfile, 'file')~=2  % Make sure folder exists (folder=7) as well as IMU data (file=2)
			fprintf('Couldn''t find folder %s or file %s_Xsens_AuxFields.sensors for subject s%d\n', activity, activity, iSubj);
			continue;
		end
		
		needToProcessData = rePreProcessData || (exist(activityPreProcessedFile, 'file')~=2);  % Determine whether this activity of the dataset has already been processed and saved as a .mat
		% Load or process IMU data
		if ~needToProcessData
			load(activityPreProcessedFile);
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
				aux = getPosFromCam(activityCamFile, [], true);
				aux.fps = 1/dt;
				aux.params.cam = camCalib(iCam);
				dataCams(iCam) = aux;
				save(activityPreProcessedFile, 'dataIMU', 'dataCams');
			end
		end
	end
end

%% Take the preprocessed files and compute similarity matrices over different sliding window lengths
windowLengths = [20 10 5];
for iWin = 1:length(windowLengths)
	tW = windowLengths(iWin);
	for iSubj = 1:5
		folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];  % Folder where all the data for subject s_i is
		activities = {'acting1', 'acting2', 'acting3', 'freestyle1', 'freestyle2', 'freestyle3', 'rom1', 'rom2', 'rom3', 'walking1', 'walking2', 'walking3'};
		for iActivity = 1:length(activities)
			activity = activities{iActivity};
			activityProcessedFile = [folderPrefix '/' activity '_processed.mat'];
			activityPreProcessedFile = [folderPrefix '/' activity '_preprocessed.mat'];
			if exist(activityPreProcessedFile, 'file')~=2  % Make sure dataset has been preprocessed
				fprintf('Couldn''t find file %s for subject s%d\n', activityPreProcessedFile, iSubj);
				continue;
			end
			dataPreProc = load(activityPreProcessedFile);
	
			% Load or initialize processed
			if iWin == 1
				processed = struct('tW',{}, 'scores',{});
			else
				dataProc = load(activityProcessedFile);
			end
			processed(iWin).tW = tW;
			processed(iWin).scores = cell(length(dataPreProc.dataCams),3);

			for iCam = 1 %1:length(dataPreProc.dataCams)
				t = (0:(length(dataPreProc.dataCams(iCam).camPos(1).rWrist.pos2D)-1))./dataPreProc.dataCams(iCam).fps;
				
				iSegment = 1;
				while true
					tS = (iSegment-1)*tW*(1-tWindowOverlap);
					tE = tS + tW;
					if tE > t(end), break; end

					tInds = (tS <= t) & (t < tE);
					for iMethod = 1:3
						[processed(iWin).scores{iCam,iMethod}{iSegment}, bodyPartToJointsAssoc] = computeSimilarityMatrix(dataPreProc.dataIMU, dataPreProc.dataCams(iCam), 1:length(dataPreProc.dataCams(iCam).camPos), false, iMethod, [], [], tInds);
					end
					fprintf('\tDone processing segment %d of s%d/%s (cam %d) tW=%d\n', iSegment, iSubj, activity, iCam, tW);
					iSegment = iSegment+1;
				end
				fprintf('Done processing s%d/%s (cam %d) tW=%d\n', iSubj, activity, iCam, tW);
			end
			
			% Save temp results
			save(activityProcessedFile, 'processed', 'activity', 'tWindowOverlap', 'bodyPartToJointsAssoc');
		end
	end
end

%% Take the preprocessed files and compute similarity matrices (spawn multiple instances to run in parallel)
w = [];
activities = {'acting1', 'acting2', 'acting3', 'freestyle1', 'freestyle2', 'freestyle3', 'rom1', 'rom2', 'rom3', 'walking1', 'walking2', 'walking3'};
windowLengths = [20 10 5];
methods = 1:3;
cams = 1;

% Initialize struct inside mat files that will store results
for iActivity = 1:length(activities)
	activity = activities{iActivity};
	for iSubj = 1:5
		activityProcessedFile = [DATA_FOLDER '/s' num2str(iSubj) '/' activity '_processed.mat'];
		if rePreProcessData || (exist(activityProcessedFile, 'file') ~= 2)
			out = struct('processed',struct('tW',num2cell(windowLengths), 'scores',{cell(length(cams),length(methods))}), 'activity',activity, 'tWindowOverlap',tWindowOverlap, 'bodyPartToJointsAssoc',bodyPartToJointsAssoc);
			save(activityProcessedFile, '-struct', 'out');
		end
	end
end

% Process each activity and subject in parallel
for iWin = 1:length(windowLengths)
	for iSubj = 1:5
		w(end+1) = parfeval(@processPreProcessedData, 0, activities, iSubj, iWin, methods, cams, DATA_FOLDER, tWindowOverlap, bodyPartToJointsAssoc);
	end
end
