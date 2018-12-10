function [scoreIMUtoCamBodyPart, bodyPartToJointsAssoc] = computeSimilarityMatrix(dataIMU, dataCam, personID, useResamp, whichMatchingMethod, indsIMU, indsBodyJointPairs, tInds)
	if nargin<3 || isempty(personID), personID = 1; end
	if nargin<4 || isempty(useResamp), useResamp = false; end
	if nargin<5 || isempty(whichMatchingMethod), whichMatchingMethod = 1; end  % 1 = Our method; 2 = 3D accel; 3 = 3D orientation
	
	bodyPartToJointsAssoc = cell2struct([{'Head', 'nose','neck'}; {'Sternum', 'rShoulder','lShoulder'}; {'Pelvis', 'rHip','lHip'}; {'L_UpArm', 'lShoulder','lElbow'}; {'R_UpArm', 'rShoulder','rElbow'}; {'L_LowArm', 'lElbow','lWrist'}; {'R_LowArm', 'rElbow','rWrist'}; {'L_UpLeg', 'lHip','lKnee'}; {'R_UpLeg', 'rHip','rKnee'}; {'L_LowLeg', 'lKnee','lAnkle'}; {'R_LowLeg', 'rKnee','rAnkle'}; {'L_Foot', 'lAnkle','lSmallToe'}; {'R_Foot', 'rAnkle','rSmallToe'}], {'bodyPart', 'joint1', 'joint2'}, 2);
	namesIMUs = setdiff(fieldnames(dataIMU), 'params', 'stable');  % Take all the IMUs (remove the field 'params'). 'stable' keeps the original ordering of fieldnames(dataIMU).
	if nargin<6 || isempty(indsIMU), indsIMU = 1:length(namesIMUs); end
	if nargin<7 || isempty(indsBodyJointPairs), indsBodyJointPairs = 1:length(bodyPartToJointsAssoc); end
	if nargin<8 || isempty(tInds), tInds = 1:length(dataIMU.(namesIMUs{1}).quat); end
	
	extraPairs = [{'Ankle', 'SmallToe'}; {'Heel', 'BigToe'}; {'Heel', 'SmallToe'}; {'BigToe', 'SmallToe'}];
	scoreIMUtoCamBodyPart = ones(length(indsIMU), length(indsBodyJointPairs));
	w = waitbar(0, 'Computing similarity matrix, this might take a while...');
	for iIMU = 1:length(indsIMU)
		nameIMU = namesIMUs{indsIMU(iIMU)};
		for iBodyJointPair = 1:length(indsBodyJointPairs)
			jointAssoc = bodyPartToJointsAssoc(indsBodyJointPairs(iBodyJointPair));
			if useResamp
				pos2Dname = 'pos2Dresamp';
				pos3Dname = 'pos3Dresamp';
				orientationIMU = dataIMU.(nameIMU).quatResamp(tInds,:);
			else
				pos2Dname = 'pos2D';
				pos3Dname = 'pos3D';
				orientationIMU = dataIMU.(nameIMU).quat(tInds,:);
			end
			posCamJoints2D = [dataCam.camPos(personID).(jointAssoc.joint1).(pos2Dname) dataCam.camPos(personID).(jointAssoc.joint2).(pos2Dname)];
			posCamJoints3D = [dataCam.camPos(personID).(jointAssoc.joint1).(pos3Dname) dataCam.camPos(personID).(jointAssoc.joint2).(pos3Dname)];
			
			if whichMatchingMethod == 1
	  			scoreIMUtoCamBodyPart(iIMU, iBodyJointPair) = find_shift_and_alignment(posCamJoints2D(tInds,:), orientationIMU(tInds,:), dataCam.params.cam.intrinsicMat, 0);
			elseif whichMatchingMethod == 2
				scoreIMUtoCamBodyPart(iIMU, iBodyJointPair) = computeSimilarityScoreBaselineAccel(dataIMU.(nameIMU), posCamJoints3D, [], tInds);
			else  % whichMatchingMethod == 3
				scoreIMUtoCamBodyPart(iIMU, iBodyJointPair) = computeSimilarityScoreBaselineOrientation(dataIMU.(nameIMU).quat(tInds,:), posCamJoints3D(tInds,:));
			end
		end
		
		% Computing the matrix takes up to a minute, show progress :)
		waitbar(iIMU/length(indsIMU), w, sprintf('IMUs processed: %2d/%d', iIMU, length(indsIMU)));
	end
	
% 	normalizedScore = sinkhornKnopp(scoreIMUtoCamBodyPart);
	close(w);
end
