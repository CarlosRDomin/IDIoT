function [bestScore, R, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted, deltaT] = find_shift_and_alignment(posCamJoints, orientationIoT, camIntrinsicMat, deltaTsweep)
% posCamJoints should be Nx4 containing u_joint1, v_joint1, u_joint2, v_joint2 as columns
	if nargin<4 || isempty(deltaTsweep), deltaTsweep = -30:30; end
	score = zeros(size(deltaTsweep));
	R = zeros(3,3,length(deltaTsweep));
	camFocalLength = mean(diag(camIntrinsicMat(1:2,1:2)));
	camF = repmat(camFocalLength, length(posCamJoints),1);
	camCenterOffset = [camIntrinsicMat(1:2,3)' 0];  % [cx cy 0]
	
% 	nCamOrientation = [camFocalLength.*(posCamJoints(:,2)-posCamJoints(:,4)), ...
% 		camFocalLength.*(posCamJoints(:,3)-posCamJoints(:,1)), ...
% 		(posCamJoints(:,1)-camCenter(1)).*(posCamJoints(:,4)-camCenter(2)) - (posCamJoints(:,3)-camCenter(1)).*(posCamJoints(:,2)-camCenter(2))];
	nCamOrientation = cross([posCamJoints(:,1:2) camF]-camCenterOffset, [posCamJoints(:,3:4) camF]-camCenterOffset, 2);
	nCamOrientation = nCamOrientation./sqrt(sum(nCamOrientation.^2, 2)); % Normalize each n (row) to modulo 1
	
	% Some files in the TotalCapture dataset, contain one less frame/IMU sample or vice versa
	if length(nCamOrientation) ~= length(orientationIoT)
		fprintf('Warning: nCamOrientation has length %d and orientationIoT has length %d! Trimming the extra sample(s) and continuing!', length(nCamOrientation), length(orientationIoT));
		l = min(length(nCamOrientation), length(orientationIoT));
		nCamOrientation = nCamOrientation(1:l,:);
		orientationIoT = orientationIoT(1:l,:);
	end

	% Try every deltaT requested
	for iDeltaT = 1:length(deltaTsweep)
		deltaT = deltaTsweep(iDeltaT);
		[score(iDeltaT), R(:,:,iDeltaT)] = shift_findR_and_rotate(deltaT, nCamOrientation, orientationIoT);
	end
	
	% Finally, recover the best deltaT (lowest score) and return the shifted and rotated values
	[bestScore, deltaTind] = min(score);
	deltaT = deltaTsweep(deltaTind);
	[~, R, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted] = shift_findR_and_rotate(deltaT, nCamOrientation, orientationIoT, R(:,:,deltaTind));
end

function [nCamOrientationShifted, orientationIoTshifted] = shiftOrientationsBy(deltaT, nCamOrientation, orientationIoT)
	nCamOrientationShifted = shiftSignalBy(deltaT, nCamOrientation, false);
	orientationIoTshifted = shiftSignalBy(deltaT, orientationIoT, true);
end

function [score, R, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted] = shift_findR_and_rotate(deltaT, nCamOrientation, orientationIoT, R)
	[nCamOrientationShifted, orientationIoTshifted] = shiftOrientationsBy(deltaT, nCamOrientation, orientationIoT);	% Shift by deltaT
	if nargin<4 || isempty(R)  % Normal use case: we don't specify R and use optimization to find R for each deltaT. Then, we pick the deltaT with lowest score and pass R to this function to shift by the best deltaT and compute orientationIoTtoCamShifted etc.
		R = findBest3Drotation(nCamOrientationShifted, orientationIoTshifted);	% Find best rotation matrix R (that minimizes score)
	end
	orientationIoTtoCamShifted = (R*orientationIoTshifted')';	% Apply projection R to fIoT to get fIoTtoCam
	score = computeSimilarityScore(nCamOrientationShifted, orientationIoTtoCamShifted);	% Compute similarity score (the lower the better)
end
