function [scoreIMUtoCamBodyPart, bodyPartToJointsAssoc] = computeSimilarityMatrix(dataIMU, dataCam, personID, useResamp)
	if nargin<3 || isempty(personID), personID = 1; end
	if nargin<4 || isempty(useResamp), useResamp = false; end
	bodyPartToJointsAssoc = cell2struct([{'Head', 'nose','neck'}; {'Sternum', 'rShoulder','lShoulder'}; {'Pelvis', 'rHip','lHip'}; {'L_UpArm', 'lShoulder','lElbow'}; {'R_UpArm', 'rShoulder','rElbow'}; {'L_LowArm', 'lElbow','lWrist'}; {'R_LowArm', 'rElbow','rWrist'}; {'L_UpLeg', 'lHip','lKnee'}; {'R_UpLeg', 'rHip','rKnee'}; {'L_LowLeg', 'lKnee','lAnkle'}; {'R_LowLeg', 'rKnee','rAnkle'}; {'L_Foot', 'lAnkle','lSmallToe'}; {'R_Foot', 'rAnkle','rSmallToe'}], {'bodyPart', 'joint1', 'joint2'}, 2);
	
	namesIMUs = setdiff(fieldnames(dataIMU), 'params', 'stable');  % Take all the IMUs (remove the field 'params'). 'stable' keeps the original ordering of fieldnames(dataIMU).
	extraPairs = [{'Ankle', 'SmallToe'}; {'Heel', 'BigToe'}; {'Heel', 'SmallToe'}; {'BigToe', 'SmallToe'}];
	scoreIMUtoCamBodyPart = ones(length(namesIMUs), length(bodyPartToJointsAssoc)+2*length(extraPairs)+1);
	w = waitbar(0, 'Computing similarity matrix, this might take a while...');
	for iIMU = 1:length(namesIMUs)
		for iBodyJointPair = 1:length(bodyPartToJointsAssoc)
			if useResamp
				pos2Dname = 'pos2Dresamp'; pos2Dname = 'pos2D';
				orientationIMU = dataIMU.(namesIMUs{iIMU}).forwardResamp;
			else
				pos2Dname = 'pos2D';
				orientationIMU = rotatepoint(dataIMU.(namesIMUs{iIMU}).quat, [1 0 0]);
			end
			scoreIMUtoCamBodyPart(iIMU, iBodyJointPair) = find_shift_and_alignment([dataCam.camPos(personID).(bodyPartToJointsAssoc(iBodyJointPair).joint1).(pos2Dname) dataCam.camPos(personID).(bodyPartToJointsAssoc(iBodyJointPair).joint2).(pos2Dname)], orientationIMU, dataCam.params.cam.intrinsicMat, 0);
		end
		
		% Test: which joints are better for the feet?
		lr = {'l', 'r'};
		for j = 1:length(lr)
			lrStr = lr{j};
			for i = 1:length(extraPairs)
				scoreIMUtoCamBodyPart(iIMU, length(bodyPartToJointsAssoc)+(j-1)*length(extraPairs)+i) = find_shift_and_alignment([dataCam.camPos(personID).([lrStr extraPairs{i,1}]).(pos2Dname) dataCam.camPos(personID).([lrStr extraPairs{i,2}]).(pos2Dname)], orientationIMU, dataCam.params.cam.intrinsicMat, 0);
			end
		end
		
		% Test: is neck-midHip better for sternum/waist?
		scoreIMUtoCamBodyPart(iIMU, end) = find_shift_and_alignment([dataCam.camPos(personID).neck.(pos2Dname) dataCam.camPos(personID).midHip.(pos2Dname)], orientationIMU, dataCam.params.cam.intrinsicMat, 0);
		
		% Computing the matrix takes up to a minute, show progress :)
		waitbar(iIMU/length(namesIMUs), w, sprintf('IMUs processed: %2d/%d', iIMU, length(namesIMUs)));
	end
	
% 	normalizedScore = sinkhornKnopp(scoreIMUtoCamBodyPart);
	close(w);
end
