function score = computeSimilarityScoreBaselineOrientation(qIoT, posCamJoints3D)
	deviceOriInCam3D = [1 -1 1] .* (posCamJoints3D(:,4:6) - posCamJoints3D(:,1:3));  % Flip the y
	deviceOriInCam3D = deviceOriInCam3D./sqrt(sum(deviceOriInCam3D.^2, 2));
	validInds = ~isnan(deviceOriInCam3D(:,1));
	if sum(validInds) < length(validInds)/4
		score = 1;
		fprintf('Too few orientation data points (%d out of %d). Score: 1\n', sum(validInds), length(validInds));
	else
		[q,v] = findBest3Drotation(deviceOriInCam3D(validInds,:), qIoT(validInds));
		orientationIoTtoCam = rotatepoint(q*qIoT(validInds), v);
		score = mean(abs(dot(orientationIoTtoCam, deviceOriInCam3D(validInds,:), 2)));
	end
end
