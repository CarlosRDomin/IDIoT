function [bestScore, q, v, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted, deltaT] = find_shift_and_alignment(posCamJoints, qIoT, camIntrinsicMat, deltaTsweep)
% posCamJoints should be Nx4 containing u_joint1, v_joint1, u_joint2, v_joint2 as columns
	if nargin<4 || isempty(deltaTsweep), deltaTsweep = -30:30; end
	score = zeros(size(deltaTsweep));
	v = zeros(length(deltaTsweep),3);
	q = quaternion();
	camFocalLength = mean(diag(camIntrinsicMat(1:2,1:2)));
	camF = repmat(camFocalLength, length(posCamJoints),1);
	camCenterOffset = [camIntrinsicMat(1:2,3)' 0];  % [cx cy 0]

	nCamOrientation = cross([posCamJoints(:,1:2) camF]-camCenterOffset, [posCamJoints(:,3:4) camF]-camCenterOffset, 2);
	nCamOrientation = nCamOrientation./sqrt(sum(nCamOrientation.^2, 2)); % Normalize each n (row) to modulo 1

	% Some files in the TotalCapture dataset, contain one less frame/IMU sample or vice versa
	if length(nCamOrientation) ~= length(qIoT)
		fprintf('Warning: nCamOrientation has length %d and orientationIoT has length %d! Trimming the extra sample(s) and continuing!', length(nCamOrientation), length(qIoT));
		l = min(length(nCamOrientation), length(qIoT));
		nCamOrientation = nCamOrientation(1:l,:);
		qIoT = qIoT(1:l,:);
	end

	% Try every deltaT requested
	for iDeltaT = 1:length(deltaTsweep)
		deltaT = deltaTsweep(iDeltaT);
		[score(iDeltaT), q(iDeltaT,:), v(iDeltaT,:)] = shift_findR_and_rotate(deltaT, nCamOrientation, qIoT);
	end
	
	% Finally, recover the best deltaT (lowest score) and return the shifted and rotated values
	[bestScore, deltaTind] = min(score);
	deltaT = deltaTsweep(deltaTind);
	[~, q,v, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted] = shift_findR_and_rotate(deltaT, nCamOrientation, qIoT, q(deltaTind,:), v(deltaTind,:));
end

function [nCamOrientationShifted, orientationIoTshifted] = shiftOrientationsBy(deltaT, nCamOrientation, qIoT)
	nCamOrientationShifted = shiftSignalBy(deltaT, nCamOrientation, false);
	orientationIoTshifted = shiftSignalBy(deltaT, qIoT, true);
end

function [score, q,v, nCamOrientationShifted, qIoTshifted, qIoTtoCamShifted, vIoTtoCamShifted] = shift_findR_and_rotate(deltaT, nCamOrientation, qIoT, q,v)
	[nCamOrientationShifted, qIoTshifted] = shiftOrientationsBy(deltaT, nCamOrientation, qIoT);	% Shift by deltaT
	if nargin<4 || isempty(q)  % Normal use case: we don't specify R and use optimization to find R for each deltaT. Then, we pick the deltaT with lowest score and pass R to this function to shift by the best deltaT and compute orientationIoTtoCamShifted etc.
		[q,v] = findBest3Drotation(nCamOrientationShifted, qIoTshifted);	% Find best rotation matrix R (that minimizes score)
	end
	qIoTtoCamShifted = q*qIoTshifted;
 	vIoTtoCamShifted = rotatepoint(qIoTtoCamShifted, v);	% Apply quaternion orientationIoTtoCamShifted to fIoT to get fIoTtoCam
	score = computeSimilarityScore(nCamOrientationShifted, qIoTtoCamShifted, v);	% Compute similarity score (the lower the better)
end
