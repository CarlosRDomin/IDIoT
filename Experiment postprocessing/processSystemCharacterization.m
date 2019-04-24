addpath(genpath('.'));	% Make sure all folders and subfolders are added to the path
cdToThisScriptsDirectory();	% Change directory to the folder containing this script
DATA_FOLDER = '../DataCollection/data';
FIGURE_FOLDER = '../Paper';
if false, preprocessSystemCharacterization; end  % Re-preprocess our data if switch to true
joints = {'rElbow', 'rWrist'};
IMUlocation = 'R_LowArm';
baselineAccelAlphaJoint = 0.9;
legendStr = {'IDIoT', '3D accel matching', '3D orientation matching'};
tWindowOverlap = 0.5;
defaultTimeWindow = 10;  % 10s windows so we can have multiple data points and show boxplot

%% Process experiments
experimentFilenames = {'varyingLinearAmplitude', 'varyingLinearSpeed', 'varyingCircularAmplitude', 'varyingCircularSpeed', 'varyingLinearCamAngle', 'varyingCircularCamAngle', 'varyingLinearCamDist', 'varyingCircularCamDist'};
xLabelStrs = {'Motion amplitude [straight line] (cm)', 'Motion speed [straight line] (bpm/4)', 'Motion amplitude [circular arc] (deg)', 'Motion speed [circular arc] (bpm/4)', 'Camera angle to motion direction [straight line pattern] (deg)', 'Camera angle to motion direction [circular arc pattern] (deg)', 'Camera distance to IoT device [straight line pattern] (m)', 'Camera distance to IoT device [circular arc pattern] (m)'};

for iExperimentType = 1:length(experimentFilenames)
	experimentFilename = experimentFilenames{iExperimentType};
	xLabelStr = xLabelStrs{iExperimentType};

	load([DATA_FOLDER '/' experimentFilename '.mat']);
	score = cell(length(experimentStruct),3);
	for iExp=1:length(experimentStruct)
		posCamJoints2D = [experimentStruct(iExp).dataCam.camPos.(joints{1}).pos2D experimentStruct(iExp).dataCam.camPos.(joints{2}).pos2D];
		posCamJoints3D = [experimentStruct(iExp).dataCam.camPos.(joints{1}).pos3D experimentStruct(iExp).dataCam.camPos.(joints{2}).pos3D];
		t = experimentStruct(iExp).dataCam.t_frames - experimentStruct(iExp).dataCam.t_frames(1);

		iSegment = 1;
		while true
			tS = (iSegment-1)*defaultTimeWindow*(1-tWindowOverlap);
			tE = tS + defaultTimeWindow;
			if tE > t(end), break; end

			tInds = (tS <= t) & (t < tE);
			score{iExp,1}(iSegment) = find_shift_and_alignment(posCamJoints2D(tInds,:), experimentStruct(iExp).dataIMU.(IMUlocation).quat(tInds,:), experimentStruct(iExp).dataCam.params.cam.intrinsicMat, 0);
			score{iExp,2}(iSegment) = computeSimilarityScoreBaselineAccel(experimentStruct(iExp).dataIMU.(IMUlocation), posCamJoints3D, baselineAccelAlphaJoint, tInds, 15);
			score{iExp,3}(iSegment) = computeSimilarityScoreBaselineOrientation(experimentStruct(iExp).dataIMU.(IMUlocation).quat(tInds,:), posCamJoints3D(tInds,:));
			iSegment = iSegment+1;
		end
	end
	h = figure('Name', xLabelStr);
	xTicks = cell2mat({experimentStruct.(experimentVariable)});
	if endsWith(experimentVariable, 'CamAngle'), xTicks = cellstr(string(xTicks)); end  % This allows us to plot [90, 60, 30, 0] instead of in ascending order
	plotBoxPlot(score, xTicks, legendStr);
	xlabel(xLabelStr); ylabel('Score');
	save([FIGURE_FOLDER '/' experimentFilename '_figData.mat'], 'score', 'xTicks', 'xLabelStr', 'legendStr', 'defaultTimeWindow', 'tWindowOverlap', 'baselineAccelAlphaJoint', 'joints', 'IMUlocation');
	savefig(h, [FIGURE_FOLDER '/' experimentFilename '.fig']);
	saveas(h, [FIGURE_FOLDER '/' experimentFilename '.eps'], 'epsc');
end
return;

%% Process time vs score
experimentFilenames = {'varyingLinearAmplitude', 'varyingCircularAmplitude'};
outputFigFilenames = {'varyingTimeWindowLinear', 'varyingTimeWindowCircular'};
xLabelStrs = {'[straight line]', '[circular arc]'};
iExperimentStruct = {5, 3};
windowLengths = {2, 4, 6, 8, 10, 15, 20, 30};
for iExperimentType = 1:length(experimentFilenames)
	experimentFilename = experimentFilenames{iExperimentType};
	outputFigFilename = outputFigFilenames{iExperimentType};
	xLabelStr = ['Time window length (s) ' xLabelStrs{iExperimentType}];
	iExp = iExperimentStruct{iExperimentType};

	load([DATA_FOLDER '/' experimentFilename '.mat']);
	posCamJoints2D = [experimentStruct(iExp).dataCam.camPos.(joints{1}).pos2D experimentStruct(iExp).dataCam.camPos.(joints{2}).pos2D];
	posCamJoints3D = [experimentStruct(iExp).dataCam.camPos.(joints{1}).pos3D experimentStruct(iExp).dataCam.camPos.(joints{2}).pos3D];
	t = experimentStruct(iExp).dataCam.t_frames - experimentStruct(iExp).dataCam.t_frames(1);
	
	score = cell(length(windowLengths),3);
	for iWin=1:length(windowLengths)
		tW = windowLengths{iWin};
		
% 		N = floor(t(end)/tW);
% 		score{iWin,1} = NaN(1,N); score{iWin,2} = NaN(1,N); score{iWin,3} = NaN(1,N);
% 		for iSegment = 1:N
		iSegment = 1;
		while true
			tS = (iSegment-1)*tW*(1-tWindowOverlap);
			tE = tS + tW;
			if tE > t(end), break; end

			tInds = (tS <= t) & (t < tE);
			score{iWin,1}(iSegment) = find_shift_and_alignment(posCamJoints2D(tInds,:), experimentStruct(iExp).dataIMU.(IMUlocation).quat(tInds,:), experimentStruct(iExp).dataCam.params.cam.intrinsicMat, 0);
			score{iWin,2}(iSegment) = computeSimilarityScoreBaselineAccel(experimentStruct(iExp).dataIMU.(IMUlocation), posCamJoints3D, baselineAccelAlphaJoint, tInds, 15);
			score{iWin,3}(iSegment) = computeSimilarityScoreBaselineOrientation(experimentStruct(iExp).dataIMU.(IMUlocation).quat(tInds,:), posCamJoints3D(tInds,:));		
			iSegment = iSegment+1;
		end
	end
	h = figure('Name', xLabelStr);
	xTicks = windowLengths;
	plotBoxPlot(score, xTicks, legendStr);
	xlabel(xLabelStr); ylabel('Score');
	save([FIGURE_FOLDER '/' outputFigFilename '_figData.mat'], 'score', 'xTicks', 'xLabelStr', 'legendStr', 'tWindowOverlap', 'baselineAccelAlphaJoint', 'joints', 'IMUlocation');
	savefig(h, [FIGURE_FOLDER '/' outputFigFilename '.fig']);
	saveas(h, [FIGURE_FOLDER '/' outputFigFilename '.eps'], 'epsc');
end
