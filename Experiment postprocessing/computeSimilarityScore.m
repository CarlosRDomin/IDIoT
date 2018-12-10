function score = computeSimilarityScore(nCamOrientation, qIoTtoCam, vIoT, boolMaximize)
	if nargin<4, boolMaximize = false; end
	vIoTtoCam = rotatepoint(qIoTtoCam, vIoT);
	score = mean(abs(dot(nCamOrientation, vIoTtoCam, 2)), 'omitNaN');
	if boolMaximize, score = 1 - score; end % Maximize instead of minimize (if requested)

% 	else
% % 		camFocalLength = mean(diag(camIntrinsicMat(1:2,1:2)));
% % 		camCenterOffset = [camIntrinsicMat(1:2,3)' 0];  % [cx cy 0]
% % 		v1 = -([posCamJoints(:,1:2) camF]-camCenterOffset)';
% % 		v2 = -([posCamJoints(:,3:4) camF]-camCenterOffset)';
% 		v1 = v1./sqrt(sum(v1.^2, 2));
% 		v2 = v2./sqrt(sum(v2.^2, 2));
% 		v1 = -v1';
% 		v2 = -v2';
% 		l = zeros(length(orientationIoTtoCam),1);
% 		
% 		for i = 1:length(orientationIoTtoCam)
% 			fProj = projectVectPlane(xAxis(i,:), nCamOrientation(i,:));
% 			fProj = fProj./sqrt(sum(fProj.^2, 2));
% 			aux = [fProj' v1(:,i)]\v2(:,i);
% 			l(i) = abs(aux(1));
% 		end
% % 		score = sum((l - mean(l, 'omitNaN')).^2, 'omitNaN');
% 		m = mean(l, 'omitNaN');
% 		score = sum(abs(l - m) > m/2, 'omitNaN');
% % 		figure; plot(l);
% % 		aux = xAxis(:,1:2); aux = aux./sqrt(sum(aux.^2, 2));
% % 		deltaP = deltaP./sqrt(sum(deltaP.^2, 2));
% % 		score = mean(acosd(abs(dot(deltaP, aux, 2))), 'omitNaN');
% 	end
end
