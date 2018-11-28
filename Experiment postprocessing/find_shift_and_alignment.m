function [deltaT, R, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted, score] = find_shift_and_alignment(posCamJoints, orientationIoT, camFocalLength, camCenter, deltaTsweep)
% posCamJoints should be Nx4 containing u_joint1, v_joint1, u_joint2, v_joint2 as columns
	if nargin<5 || isempty(deltaTsweep), deltaTsweep = -30:30; end
	score = zeros(size(deltaTsweep));
	
	nCamOrientation = [camFocalLength.*(posCamJoints(:,2)-posCamJoints(:,4)), ...
		camFocalLength.*(posCamJoints(:,3)-posCamJoints(:,1)), ...
		(posCamJoints(:,1)-camCenter(1)).*(posCamJoints(:,4)-camCenter(2)) - (posCamJoints(:,3)-camCenter(1)).*(posCamJoints(:,2)-camCenter(2))];
	nCamOrientation = nCamOrientation./sqrt(sum(nCamOrientation.^2, 2)); % Normalize each n (row) to modulo 1

	% Try every deltaT requested
	for iDeltaT = 1:length(deltaTsweep)
		deltaT = deltaTsweep(iDeltaT);
		score(iDeltaT) = shift_findR_and_rotate(deltaT, nCamOrientation, orientationIoT);
	end
	
	% Finally, recover the best deltaT (lowest score) and return the shifted and rotated values
	[~, deltaTind] = min(score);
	deltaT = deltaTsweep(deltaTind);
	[score, R, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted] = shift_findR_and_rotate(deltaT, nCamOrientation, orientationIoT);
end

function [nCamOrientationShifted, orientationIoTshifted] = shiftOrientationsBy(deltaT, nCamOrientation, orientationIoT)
	nCamOrientationShifted = shiftSignalBy(deltaT, nCamOrientation, false);
	orientationIoTshifted = shiftSignalBy(deltaT, orientationIoT, true);
end

function [score, R, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted] = shift_findR_and_rotate(deltaT, nCamOrientation, orientationIoT)
	[nCamOrientationShifted, orientationIoTshifted] = shiftOrientationsBy(deltaT, nCamOrientation, orientationIoT);	% Shift by deltaT
	R = findBest3Drotation(nCamOrientationShifted, orientationIoTshifted);	% Find best rotation matrix R
	orientationIoTtoCamShifted = (R*orientationIoTshifted')';	% Apply projection R to fIoT to get fIoTtoCam
	score = sum(asin(dot(nCamOrientationShifted, orientationIoTtoCamShifted, 2)).^2);	% Compute similarity score (the lower the better)
end
