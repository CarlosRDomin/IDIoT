function [scoreIMUtoCamBodyPart, bodyPartToJointsAssoc] = computeSimilarityMatrix(dataIMU, dataCam, personIDs, useResamp, whichMatchingMethod, indsIMU, indsBodyJointPairs, tInds)
	if nargin<3 || isempty(personIDs), personIDs = 1; end
	if nargin<4 || isempty(useResamp), useResamp = false; end
	if nargin<5 || isempty(whichMatchingMethod), whichMatchingMethod = 1; end  % 1 = Our method; 2 = 3D accel; 3 = 3D orientation
	
	bodyPartToJointsAssoc = cell2struct([{'Head', 'nose','neck', -1}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
	namesIMUs = setdiff(fieldnames(dataIMU), 'params', 'stable');  % Take all the IMUs (remove the field 'params'). 'stable' keeps the original ordering of fieldnames(dataIMU).
	if nargin<6 || isempty(indsIMU), indsIMU = 1:length(namesIMUs); end
	if nargin<7 || isempty(indsBodyJointPairs), indsBodyJointPairs = 1:length(bodyPartToJointsAssoc); end
	if nargin<8 || isempty(tInds), tInds = 1:length(dataIMU.(namesIMUs{1}).quat); end

	scoreIMUtoCamBodyPart = ones(length(indsIMU), length(personIDs)*length(indsBodyJointPairs));
	w = waitbar(0, 'Computing similarity matrix, this might take a while...');
	for iIMU = 1:length(indsIMU)
		nameIMU = namesIMUs{indsIMU(iIMU)};
		
		for iPerson = 1:length(personIDs)
			personID = personIDs(iPerson);
			
			for iBodyJointPair = 1:length(indsBodyJointPairs)
				jointAssoc = bodyPartToJointsAssoc(indsBodyJointPairs(iBodyJointPair));
				iInd = (iPerson-1)*length(indsBodyJointPairs) + iBodyJointPair;
				
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
				validInds = all(~isnan(posCamJoints2D(tInds,:)), 2);

				if sum(validInds) < length(validInds)/4
					scoreIMUtoCamBodyPart(iIMU, iInd) = 1;
					if false, fprintf('Too few data points in this segment (%d out of %d). Score: 1\n', sum(validInds), length(validInds)); end
				elseif whichMatchingMethod == 1
					scoreIMUtoCamBodyPart(iIMU, iInd) = find_shift_and_alignment(posCamJoints2D(tInds,:), orientationIMU, dataCam.params.cam.intrinsicMat, 0);
				elseif whichMatchingMethod == 2
					scoreIMUtoCamBodyPart(iIMU, iInd) = computeSimilarityScoreBaselineAccel(dataIMU.(nameIMU), posCamJoints3D, jointAssoc.alpha, tInds);
				else  % whichMatchingMethod == 3
					scoreIMUtoCamBodyPart(iIMU, iInd) = computeSimilarityScoreBaselineOrientation(dataIMU.(nameIMU).quat(tInds,:), posCamJoints3D(tInds,:));
				end
			end
		end
		
		% Computing the matrix takes up to a minute, show progress :)
		waitbar(iIMU/length(indsIMU), w, sprintf('IMUs processed: %2d/%d', iIMU, length(indsIMU)));
	end
	
% 	normalizedScore = sinkhornKnopp(scoreIMUtoCamBodyPart);
	close(w);
end
