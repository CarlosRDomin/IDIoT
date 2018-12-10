function score = computeSimilarityScore(nCamOrientation, qIoTtoCam, vIoT, boolMaximize)
	if nargin<4, boolMaximize = false; end
	vIoTtoCam = rotatepoint(qIoTtoCam, vIoT);
	score = mean(abs(dot(nCamOrientation, vIoTtoCam, 2)), 'omitNaN');
	if boolMaximize, score = 1 - score; end  % Maximize instead of minimize (if requested)
end
