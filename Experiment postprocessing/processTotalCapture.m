addpath(genpath('.'));	% Make sure all folders and subfolders are added to the path
cdToThisScriptsDirectory();	% Change directory to the folder containing this script
DATA_FOLDER = '../TotalCapture';
FIGURE_FOLDER = '../Paper';
bodyPartToJointsAssoc = cell2struct([{'Head', 'nose','neck', -1}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
lgndActivities = {' Acting', ' Freestyle', ' RoM', ' Walking'};
lgndMethods = {'IDIoT', '3D accel matching', '3D orientation matching'};
methodNames = {'IDIoT', 'baseline3Daccel', 'baseline3Dori'};
jointsToConsider = [4,5,6,7,8,9,10,11];
bwIntensity = [0.25, 0.75];
baseColor = 1.5.*[0 0.4470 0.7410];

%% Plot figure 10: device location vs accuracy, different tW (repeat for each activity type)
activities = {'acting', 'freestyle', 'rom', 'walking'};
avgAccuracy = cell(length(activities), 3);
meanAvgAccuracy = zeros(size(avgAccuracy)); stdAvgAccuracy = zeros(size(avgAccuracy));
iCam = 1;
iMethod = 1;
subjects = 1:5;
for iActivity = 1:length(activities)
	for iActivityNum = 1:3
		activity = [activities{iActivity} num2str(iActivityNum)];
		for iSubj = subjects
			folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];  % Folder where all the data for subject s_i is
			activityProcessedFile = [folderPrefix '/' activity '_processed.mat'];
			if exist(activityProcessedFile, 'file')~=2  % Make sure dataset has been preprocessed
				fprintf('Couldn''t find file %s for subject s%d\n', activityProcessedFile, iSubj);
				continue;
			end
			load(activityProcessedFile);

			% For each window length Tw, look at the similarity matrices, assign the most likely, and determine whether the assignment was correct or not
			for iTw = 1:length(processed)
				matrices = processed(iTw).scores{iCam,iMethod};
				for iMat = 1:length(matrices)
					a = assignDetectionsToTracks(matrices{iMat}(jointsToConsider, ismember((1+mod((1:size(matrices{iMat},2))-1, 13)), jointsToConsider)), max(matrices{iMat}(:))+1);
					assignedCamBodyPart = zeros(1,length(a)); assignedCamBodyPart(a(:,1))=a(:,2);
					correct = (1+mod(assignedCamBodyPart-1, length(assignedCamBodyPart)) == 1:length(assignedCamBodyPart));
					avgAccuracy{iActivity,iTw}(end+1) = mean(correct);
				end
			end
		end
	end
end

% Compute mean and std
for i = 1:size(avgAccuracy,1)
	for j = 1:size(avgAccuracy,2)
		meanAvgAccuracy(i,j) = mean(avgAccuracy{i,j});
		stdAvgAccuracy(i,j) = std(avgAccuracy{i,j});
	end
end
	
% And plot the figure
h = figure('Name', 'Activity type vs Avg. Accuracy, tW comparison', 'Position', [0,0, 840,420]); 
if false
	colOrder = get(gca, 'ColorOrder');
	plotBoxPlot(avgAccuracy, lgndActivities, strcat("T_w = ", string({processed.tW}), "s"), 1.43*colOrder(iMethod,:).*linspace(bwIntensity(1),bwIntensity(2),3)', 0.18, 0.24);
else
	b = bar(categorical(lgndActivities), meanAvgAccuracy, 'FaceAlpha',0.65, 'LineWidth',1.5, 'EdgeColor','flat', 'FaceColor','flat');
	col = baseColor.*linspace(bwIntensity(1),bwIntensity(2),length(b))'; for iB = 1:length(b), b(iB).CData = col(iB,:); end
	legend(b, strcat("T_w = ", string({processed.tW}), "s"));
end
xlabel('Activity type'); ylabel('Avg. Accuracy'); ylim([0 1]); set(gca, 'YGrid','on'); legend('Location', 'NorthEast');
save([FIGURE_FOLDER '/activityTypeVsAvgAccuracy_winLengthComparison_figData.mat'], 'avgAccuracy', 'meanAvgAccuracy', 'stdAvgAccuracy', 'iCam', 'iMethod', 'subjects');
savefig(h, [FIGURE_FOLDER '/activityTypeVsAvgAccuracy_winLengthComparison.fig']);
saveas(h, [FIGURE_FOLDER '/activityTypeVsAvgAccuracy_winLengthComparison.eps'], 'epsc');

if false
	activities = {'acting', 'freestyle', 'rom', 'walking'};
	assignmentsProcessed = struct('activity',activities, 'correctGuesses',{cell(length(bodyPartToJointsAssoc),3)}, 'meanCorrectGuesses',zeros(length(bodyPartToJointsAssoc),3), 'stdCorrectGuesses',zeros(length(bodyPartToJointsAssoc),3));
	iCam = 1;
	iMethod = 1;
	subjects = 1:5;
	for iActivity = 1:length(activities)
		for iActivityNum = 1:3
			activity = [activities{iActivity} num2str(iActivityNum)];
			for iSubj = subjects
				folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];  % Folder where all the data for subject s_i is
				activityProcessedFile = [folderPrefix '/' activity '_processed.mat'];
				if exist(activityProcessedFile, 'file')~=2  % Make sure dataset has been preprocessed
					fprintf('Couldn''t find file %s for subject s%d\n', activityProcessedFile, iSubj);
					continue;
				end
				load(activityProcessedFile);

				% For each method, look at the similarity matrices, assign the most likely, and determine whether the assignment was correct or not
				for iTw = 1:length(processed)
					matrices = processed(iTw).scores{iCam,iMethod};
					for iMat = 1:length(matrices)
						a = assignDetectionsToTracks(matrices{iMat}, max(matrices{iMat}(:))+1);
						assignedCamBodyPart = zeros(1,length(a)); assignedCamBodyPart(a(:,1))=a(:,2);
						for iBodyPart = 1:length(assignedCamBodyPart)
							assignmentsProcessed(iActivity).correctGuesses{iBodyPart,iTw}(end+1) = (1+mod(assignedCamBodyPart(iBodyPart)-1, length(assignedCamBodyPart)) == iBodyPart);
						end
					end
				end
			end
		end

		% Compute mean and std
		for i = 1:size(assignmentsProcessed(iActivity).correctGuesses,1)
			for j = 1:size(assignmentsProcessed(iActivity).correctGuesses,2)
				assignmentsProcessed(iActivity).meanCorrectGuesses(i,j) = mean(assignmentsProcessed(iActivity).correctGuesses{i,j});
				assignmentsProcessed(iActivity).stdCorrectGuesses(i,j) = std(assignmentsProcessed(iActivity).correctGuesses{i,j});
			end
		end

		% And plot the figure
		h = figure('Name', ['Device location vs Accuracy [' activities{iActivity} '] tW comparison'], 'Position', [0,0, 840,420]);
		b = bar(categorical(1:13,1:13,{bodyPartToJointsAssoc.bodyPart}), assignmentsProcessed(iActivity).meanCorrectGuesses, 'FaceAlpha',0.65, 'LineWidth',1.5, 'EdgeColor','flat', 'FaceColor','flat');
		col = baseColor.*linspace(bwIntensity(1),bwIntensity(2),length(b))'; for iB = 1:length(b), b(iB).CData = col(iB,:); end
		xlabel('IMU location'); ylabel('Accuracy'); legend(b, strcat("T_w = ", string({processed.tW}), "s")); ylim([0 1]); set(gca, 'YGrid','on', 'TickLabelInterpreter','none');
		save([FIGURE_FOLDER '/deviceLocationVsAccuracy_winLengthComparison_' activities{iActivity} '_figData.mat'], 'assignmentsProcessed', 'iCam', 'iMethod', 'subjects');
		savefig(h, [FIGURE_FOLDER '/deviceLocationVsAccuracy_winLengthComparison_' activities{iActivity} '.fig']);
		saveas(h, [FIGURE_FOLDER '/deviceLocationVsAccuracy_winLengthComparison_' activities{iActivity} '.eps'], 'epsc');
	end
end

%% Plot figure 10b: device location vs accuracy, baseline comparison (repeat for each activity type)
activities = {'acting', 'freestyle', 'rom', 'walking'};
assignmentsProcessed = struct('activity',activities, 'correctGuesses',{cell(length(bodyPartToJointsAssoc),3)}, 'meanCorrectGuesses',zeros(length(bodyPartToJointsAssoc),3), 'stdCorrectGuesses',zeros(length(bodyPartToJointsAssoc),3));
iCam = 1;
iTw = 1;
subjects = 1:5;
for iActivity = 1:length(activities)
	for iActivityNum = 1:3
		activity = [activities{iActivity} num2str(iActivityNum)];
		for iSubj = subjects
			folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];  % Folder where all the data for subject s_i is
			activityProcessedFile = [folderPrefix '/' activity '_processed.mat'];
			if exist(activityProcessedFile, 'file')~=2  % Make sure dataset has been preprocessed
				fprintf('Couldn''t find file %s for subject s%d\n', activityProcessedFile, iSubj);
				continue;
			end
			load(activityProcessedFile);

			% For each method, look at the similarity matrices, assign the most likely, and determine whether the assignment was correct or not
			for iMethod = 1:3
				matrices = processed(iTw).scores{iCam,iMethod};
				for iMat = 1:length(matrices)
					a = assignDetectionsToTracks(matrices{iMat}, max(matrices{iMat}(:))+1);
					assignedCamBodyPart = zeros(1,length(a)); assignedCamBodyPart(a(:,1))=a(:,2);
					for iBodyPart = 1:length(assignedCamBodyPart)
						assignmentsProcessed(iActivity).correctGuesses{iBodyPart,iMethod}(end+1) = double(1+mod(assignedCamBodyPart(iBodyPart)-1, length(assignedCamBodyPart)) == iBodyPart);
					end
				end
			end
		end
	end

	% Compute mean and std
	for i = 1:size(assignmentsProcessed(iActivity).correctGuesses,1)
		for j = 1:size(assignmentsProcessed(iActivity).correctGuesses,2)
			assignmentsProcessed(iActivity).meanCorrectGuesses(i,j) = mean(assignmentsProcessed(iActivity).correctGuesses{i,j});
			assignmentsProcessed(iActivity).stdCorrectGuesses(i,j) = std(assignmentsProcessed(iActivity).correctGuesses{i,j});
		end
	end
	
	% And plot the figure
	h = figure('Name', ['Device location vs Accuracy [' activities{iActivity} '] baseline comparison'], 'Position', [0,0, 840,420]);
	b = bar(categorical(1:13,1:13,{bodyPartToJointsAssoc.bodyPart}), assignmentsProcessed(iActivity).meanCorrectGuesses, 'FaceAlpha',0.65, 'LineWidth',1.5, 'EdgeColor','flat');
	xlabel('IMU location'); ylabel('Accuracy'); legend(b, lgndMethods); ylim([0 1]); set(gca, 'YGrid','on', 'TickLabelInterpreter','none');
	save([FIGURE_FOLDER '/deviceLocationVsAccuracy_baselineComparison_' activities{iActivity} '_figData.mat'], 'assignmentsProcessed', 'iCam', 'iTw', 'subjects');
	savefig(h, [FIGURE_FOLDER '/deviceLocationVsAccuracy_baselineComparison_' activities{iActivity} '.fig']);
	saveas(h, [FIGURE_FOLDER '/deviceLocationVsAccuracy_baselineComparison_' activities{iActivity} '.eps'], 'epsc');
end

%% Plot figure 11: device location vs accuracy, different activity types
activities = {'acting', 'freestyle', 'rom', 'walking'};
methods = 1:3;
avgAccuracy = cell(length(activities), length(methods));
meanAvgAccuracy = zeros(size(avgAccuracy)); stdAvgAccuracy = zeros(size(avgAccuracy));
iCam = 1;
iTw = 1;
subjects = 1:5;
for iActivity = 1:length(activities)
	for iActivityNum = 1:3
		activity = [activities{iActivity} num2str(iActivityNum)];
		for iSubj = subjects
			folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];  % Folder where all the data for subject s_i is
			activityProcessedFile = [folderPrefix '/' activity '_processed.mat'];
			if exist(activityProcessedFile, 'file')~=2  % Make sure dataset has been preprocessed
				fprintf('Couldn''t find file %s for subject s%d\n', activityProcessedFile, iSubj);
				continue;
			end
			load(activityProcessedFile);

			% For each method, look at the similarity matrices, assign the most likely, and determine whether the assignment was correct or not
			for iMethod = methods
				matrices = processed(iTw).scores{iCam,iMethod};
				for iMat = 1:length(matrices)
					a = assignDetectionsToTracks(matrices{iMat}(jointsToConsider, ismember((1+mod((1:size(matrices{iMat},2))-1, 13)), jointsToConsider)), max(matrices{iMat}(:))+1);
					assignedCamBodyPart = zeros(1,length(a)); assignedCamBodyPart(a(:,1))=a(:,2);
					correct = (1+mod(assignedCamBodyPart-1, length(assignedCamBodyPart)) == 1:length(assignedCamBodyPart));
					avgAccuracy{iActivity,iMethod}(end+1) = mean(correct);
				end
			end
		end
	end
end

% Compute mean and std
for i = 1:size(avgAccuracy,1)
	for j = 1:size(avgAccuracy,2)
		meanAvgAccuracy(i,j) = mean(avgAccuracy{i,j});
		stdAvgAccuracy(i,j) = std(avgAccuracy{i,j});
	end
end

% And plot the figure
h = figure('Name', 'Activity vs Avg. Accuracy, method comparison', 'Position', [0,0, 840,420]);
if false
	colOrder = get(gca, 'ColorOrder');
	plotBoxPlot(avgAccuracy, lgndActivities, lgndMethods, [], 0.18, 0.24);
else
	b = bar(categorical(lgndActivities), meanAvgAccuracy, 'FaceAlpha',0.65, 'LineWidth',1.5, 'EdgeColor','flat', 'FaceColor','flat');
% 	col = baseColor.*linspace(bwIntensity(1),bwIntensity(2),length(b))'; for iB = 1:length(b), b(iB).CData = col(iB,:); end
	legend(b, lgndMethods);
end
xlabel('Activity type'); ylabel('Avg. Accuracy'); ylim([0 1]); set(gca, 'YGrid','on'); legend('Location', 'NorthEast');
save([FIGURE_FOLDER '/activityTypeVsAvgAccuracy_figData.mat'], 'avgAccuracy', 'meanAvgAccuracy', 'stdAvgAccuracy', 'iCam', 'iMethod', 'iTw', 'subjects');
savefig(h, [FIGURE_FOLDER '/activityTypeVsAvgAccuracy.fig']);
saveas(h, [FIGURE_FOLDER '/activityTypeVsAvgAccuracy.eps'], 'epsc');


if false
	activities = {'acting', 'freestyle', 'rom', 'walking'};
	assignmentsProcessed = struct('activities',{activities}, 'correctGuesses',{cell(length(bodyPartToJointsAssoc),4)}, 'meanCorrectGuesses',zeros(length(bodyPartToJointsAssoc),4), 'stdCorrectGuesses',zeros(length(bodyPartToJointsAssoc),4));
	iCam = 1;
	iTw = 1;
	iMethod = 1;
	subjects = 1:5;
	for iActivity = 1:length(activities)
		for iActivityNum = 1:3
			activity = [activities{iActivity} num2str(iActivityNum)];
			for iSubj = subjects
				folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];  % Folder where all the data for subject s_i is
				activityProcessedFile = [folderPrefix '/' activity '_processed.mat'];
				if exist(activityProcessedFile, 'file')~=2  % Make sure dataset has been preprocessed
					fprintf('Couldn''t find file %s for subject s%d\n', activityProcessedFile, iSubj);
					continue;
				end
				load(activityProcessedFile);

				% For each method, look at the similarity matrices, assign the most likely, and determine whether the assignment was correct or not
				matrices = processed(iTw).scores{iCam,iMethod};
				for iMat = 1:length(matrices)
					a = assignDetectionsToTracks(matrices{iMat}, max(matrices{iMat}(:))+1);
					assignedCamBodyPart = zeros(1,length(a)); assignedCamBodyPart(a(:,1))=a(:,2);
					for iBodyPart = 1:length(assignedCamBodyPart)
						assignmentsProcessed.correctGuesses{iBodyPart,iActivity}(end+1) = (1+mod(assignedCamBodyPart(iBodyPart)-1, length(assignedCamBodyPart)) == iBodyPart);
					end
				end
			end
		end

		% Compute mean and std
		for i = 1:size(assignmentsProcessed.correctGuesses,1)
			for j = 1:size(assignmentsProcessed.correctGuesses,2)
				assignmentsProcessed.meanCorrectGuesses(i,j) = mean(assignmentsProcessed.correctGuesses{i,j});
				assignmentsProcessed.stdCorrectGuesses(i,j) = std(assignmentsProcessed.correctGuesses{i,j});
			end
		end
	end

	% And plot the figure
	h = figure('Name', 'Device location vs Accuracy, activity comparison', 'Position', [0,0, 840,420]);
	b = bar(categorical(1:13,1:13,{bodyPartToJointsAssoc.bodyPart}), assignmentsProcessed.meanCorrectGuesses, 'FaceAlpha',0.65, 'LineWidth',1.5, 'EdgeColor','flat', 'FaceColor','flat');
	col = baseColor.*linspace(bwIntensity(1),bwIntensity(2),length(b))'; for iB = 1:length(b), b(iB).CData = col(iB,:); end
	xlabel('IMU location'); ylabel('Accuracy'); legend(b, lgndActivities); ylim([0 1]); set(gca, 'YGrid','on', 'TickLabelInterpreter','none');
	save([FIGURE_FOLDER '/deviceLocationVsAccuracy_winLengthComparison_figData.mat'], 'assignmentsProcessed', 'iCam', 'iMethod', 'iTw', 'subjects');
	savefig(h, [FIGURE_FOLDER '/deviceLocationVsAccuracy_winLengthComparison.fig']);
	saveas(h, [FIGURE_FOLDER '/deviceLocationVsAccuracy_winLengthComparison.eps'], 'epsc');
end

%% Plot figure 12: subject ID vs avg accuracy, different activity types (repeat for each method)
activities = {'acting', 'freestyle', 'rom', 'walking'};
subjects = 1:5;
methods = 1:3;
avgAccuracy = cell(length(subjects), length(activities));
totalAvgAccuracy = cell(length(methods));
totalCorrect = zeros(length(activities), length(methods));
totalAttempted = zeros(length(activities), length(methods));
iCam = 1;
iTw = 1;
for iMethod = methods
	for iActivity = 1:length(activities)
		for iActivityNum = 1:3
			activity = [activities{iActivity} num2str(iActivityNum)];
			for iSubj = subjects
				folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];  % Folder where all the data for subject s_i is
				activityProcessedFile = [folderPrefix '/' activity '_processed.mat'];
				if exist(activityProcessedFile, 'file')~=2  % Make sure dataset has been preprocessed
					%fprintf('Couldn''t find file %s for subject s%d\n', activityProcessedFile, iSubj);
					continue;
				end
				load(activityProcessedFile);

				% For each method, look at the similarity matrices, assign the most likely, and determine whether the assignment was correct or not
				matrices = processed(iTw).scores{iCam,iMethod};
				for iMat = 1:length(matrices)
					a = assignDetectionsToTracks(matrices{iMat}(jointsToConsider, ismember((1+mod((1:size(matrices{iMat},2))-1, 13)), jointsToConsider)), max(matrices{iMat}(:))+1);
					assignedCamBodyPart = zeros(1,length(a)); assignedCamBodyPart(a(:,1))=a(:,2);
					correct = (1+mod(assignedCamBodyPart-1, length(assignedCamBodyPart)) == 1:length(assignedCamBodyPart));
					avgAccuracy{iSubj, iActivity}(end+1) = mean(correct);
					totalAvgAccuracy{iMethod}(end+1) = mean(correct);
					totalCorrect(iActivity, iMethod) = totalCorrect(iActivity, iMethod) + sum(correct);
					totalAttempted(iActivity, iMethod) = totalAttempted(iActivity, iMethod) + length(correct);
				end
			end
		end
	end
	
	fprintf('Method %s has a mean avg. accuracy of %6.2f%% and std of %6.2f%% (total of %d windows)\n', methodNames{iMethod}, 100.*mean(totalAvgAccuracy{iMethod}), 100.*std(totalAvgAccuracy{iMethod}), length(totalAvgAccuracy{iMethod}));
	continue;

	% And plot the figure
	h = figure('Name', ['Subject vs Avg accuracy [' methodNames{iMethod} '] activity comparison'], 'Position', [0,0, 840,420]); colOrder = get(gca, 'ColorOrder');
	plotBoxPlot(avgAccuracy, strcat("Subject ", string(subjects)), lgndActivities, 1.43*colOrder(iMethod,:).*linspace(bwIntensity(1),bwIntensity(2),4)', 0.13,0.25);
	xlabel('Subject ID'); ylabel('Avg. Accuracy'); ylim([0 1]); set(gca, 'YGrid','on'); legend('Location', 'SouthEast');
	save([FIGURE_FOLDER '/subjectIDvsAvgAccuracy_activityComparison_' methodNames{iMethod} '_figData.mat'], 'avgAccuracy', 'iCam', 'iTw', 'iMethod', 'subjects', 'activities', 'totalAvgAccuracy');
	savefig(h, [FIGURE_FOLDER '/subjectIDvsAvgAccuracy_activityComparison_' methodNames{iMethod} '.fig']);
	saveas(h, [FIGURE_FOLDER '/subjectIDvsAvgAccuracy_activityComparison_' methodNames{iMethod} '.eps'], 'epsc');
end
100.*totalCorrect./totalAttempted,  % Each row is an activity, each column is the % accuracy for each method

%% Plot figure 14: device location vs accuracy, different camera angle (repeat for each activity type and method)
activities = {'acting', 'freestyle', 'rom', 'walking'};
iTw = 1;
iSubj = 1;
iMethod = 1;
iActivityNum = 1;
cams = 1:8;
assignmentsProcessed = struct('activity',{activities}, 'correctGuesses',{cell(length(cams),length(activities))}, 'meanCorrectGuesses',zeros(length(cams),length(activities)), 'stdCorrectGuesses',zeros(length(cams),length(activities)));
for iActivity = 1:length(activities)
	activity = [activities{iActivity} num2str(iActivityNum)];
	folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];  % Folder where all the data for subject s_i is
	activityProcessedFile = [folderPrefix '/' activity '_processed.mat'];
	if exist(activityProcessedFile, 'file')~=2  % Make sure dataset has been preprocessed
		fprintf('Couldn''t find file %s for subject s%d\n', activityProcessedFile, iSubj);
		continue;
	end
	load(activityProcessedFile);

	% For each method, look at the similarity matrices, assign the most likely, and determine whether the assignment was correct or not
	for iCam = cams
		matrices = processed(iTw).scores{iCam,iMethod};
		for iMat = 1:length(matrices)
			a = assignDetectionsToTracks(matrices{iMat}(jointsToConsider, ismember((1+mod((1:size(matrices{iMat},2))-1, 13)), jointsToConsider)), max(matrices{iMat}(:))+1);
			assignedCamBodyPart = zeros(1,length(a)); assignedCamBodyPart(a(:,1))=a(:,2);
			correct = (1+mod(assignedCamBodyPart-1, length(assignedCamBodyPart)) == 1:length(assignedCamBodyPart));
			assignmentsProcessed.correctGuesses{iCam,iActivity}(end+1) = mean(correct);
		end
	end
end

% Compute mean and std
for i = 1:size(assignmentsProcessed.correctGuesses,1)
	for j = 1:size(assignmentsProcessed.correctGuesses,2)
		assignmentsProcessed.meanCorrectGuesses(i,j) = mean(assignmentsProcessed.correctGuesses{i,j});
		assignmentsProcessed.stdCorrectGuesses(i,j) = std(assignmentsProcessed.correctGuesses{i,j});
	end
end

% And plot the figure
h = figure('Name', 'Camera ID vs Avg Accuracy, activity comparison', 'Position', [0,0, 840,420]);
b = bar(categorical(strcat("Camera ", string(cams))), assignmentsProcessed.meanCorrectGuesses, 'FaceAlpha',0.65, 'LineWidth',1.5, 'EdgeColor','flat', 'FaceColor','flat');
col = baseColor.*linspace(bwIntensity(1),bwIntensity(2),length(b))'; for iB = 1:length(b), b(iB).CData = col(iB,:); end
xlabel('Camera ID'); ylabel('Avg. Accuracy'); legend(b, lgndActivities); ylim([0 1]); set(gca, 'YGrid','on'); legend('Position', [0.5720 0.7810 0.0952 0.1179]);
save([FIGURE_FOLDER '/camIDVsAvgAccuracy_activityComparison_figData.mat'], 'assignmentsProcessed', 'cams', 'iMethod', 'iSubj', 'iTw', 'jointsToConsider');
savefig(h, [FIGURE_FOLDER '/camIDVsAvgAccuracy_activityComparison.fig']);
saveas(h, [FIGURE_FOLDER '/camIDVsAvgAccuracy_activityComparison.eps'], 'epsc');
return;

%% Load h5 files and process each person's joint positions from camera and IMU orientation
for iSubj = 1:5
	folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];  % Folder where all the data for subject s_i is
	activities = {'acting1', 'acting2', 'acting3', 'freestyle1', 'freestyle2', 'freestyle3', 'rom1', 'rom2', 'rom3', 'walking1', 'walking2', 'walking3'};
	for iActivity = 1:length(activities)
		activity = activities{iActivity};
		activityProcessedFile = [folderPrefix '/' activity '_processed.mat'];
		if exist(activityProcessedFile, 'file')~=2  % Make sure dataset has been preprocessed
			fprintf('Couldn''t find file %s for subject s%d\n', activityProcessedFile, iSubj);
			continue;
		end
		load(activityProcessedFile);

		processed = struct('tW',{}, 'scores',{});
		windowLengths = 5:5:20;
		for iWin = 1:length(windowLengths)
			tW = windowLengths(iWin);
			processed(iWin).tW = tW;
			processed(iWin).scores = cell(length(dataCams),3);
			for iCam = 1:length(dataCams)
				t = (0:(length(dataCams(iCam).camPos(1).rWrist.pos2D)-1))./dataCams(iCam).fps;
				
				iSegment = 1;
				while true
					tS = (iSegment-1)*tW*(1-tWindowOverlap);
					tE = tS + tW;
					if tE > t(end), break; end

					tInds = (tS <= t) & (t < tE);
					for iMethod = 1:3
						processed(iWin).scores{iCam,iMethod}{iSegment} = computeSimilarityMatrix(dataIMU, dataCams(iCam), 1:length(dataCams(iCam).camPos), false, iMethod, [], [], tInds);
					end
					iSegment = iSegment+1;
				end
				continue;  % TEMPORARY: Ignore assignment, etc.
				
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
