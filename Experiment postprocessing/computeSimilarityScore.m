function score = computeSimilarityScore(nCamOrientation, orientationIoTtoCam)
	score = mean(asin(dot(nCamOrientation, orientationIoTtoCam, 2)).^2, 'omitNaN');
end
