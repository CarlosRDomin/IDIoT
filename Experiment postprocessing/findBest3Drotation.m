% This file contains the math and optimization solver code to find the best rotation matrix R that makes fIoT be as parallel as possible to the plane whose normal is defined by nCamOrientation

function R = findBest3Drotation(nCamOrientation, orientationIoT)
	validInds = ~isnan(nCamOrientation(:,1));  % Only take into account measurements where the camera sees the body part
	nCamOrientation = nCamOrientation(validInds,:);
	orientationIoT = orientationIoT(validInds,:);
	[q,bestScore,exitFlag,output] = fmincon(@(q) computeSimilarityScore(nCamOrientation, rotatepoint(quaternion(q), orientationIoT)), ...  % sum(abs(dot(nCamOrientation, rotateframe(quaternion(q), orientationIoT), 2).^2), 'omitNaN')
		[1 0 0 0],[],[],[],[],[],[], ...
		@getNonLinearConstraints, optimoptions('fmincon', 'FunctionTolerance',1e-9, 'StepTolerance',1e-9, 'OptimalityTolerance',1e-10, 'SpecifyConstraintGradient',false, 'Display','off')); %, 'StepTolerance',1e-3, 'Algorithm','sqp', 'FunctionTolerance',1e-3, 'UseParallel',true, 'Display','iter-detailed'));
	
	R = rotmat(quaternion(q), 'point');
end

function [C, Ceq, gradC, gradCeq] = getNonLinearConstraints(q)
	C = []; % <= 0
	gradC = [];
	
	Ceq = sum(q.^2) - 1; % = 0
	gradCeq = 2*reshape(q, [],1);
end

function A = findBest3DrotationWrong(orientationCam, orientationIoT)
	% Problem definition:
	% x: [a11 a12 a13 a21 a22 a23] such that A*aIoT = aCam
	%  -> For each point in aIoT and aCam, we get two equations:
	%   (x coord): a11*aIoT(i,1) + a12*aIoT(i,2) + a13*aIoT(i,3) = aCam(i,1)
	%   (y coord): a21*aIoT(i,1) + a22*aIoT(i,2) + a23*aIoT(i,3) = aCam(i,2)
	%
	% Therefore, we can build matrices C,d such that C*x = d by having:
	%  C: [ aIoT zeros(N,3)		d: [ aCam(:,1)
	%		zeros(N,3) aIoT ]		 aCam(:,2) ]
	
% 	A = [1 0 0; 0 1 0] * rigid_transform_3D(orientationIoT, [orientationCam zeros(length(orientationCam),1)]);
% 	return;
	
% 	C = [orientationIoT zeros(size(orientationIoT)); zeros(size(orientationIoT)) orientationIoT];
% 	d = reshape(orientationCam, [],1);
% 	
% 	x = lsqlin(C, d, [],[],[],[], -1,1);
% 	A = reshape(x, 3,2)';

	[q,bestScore,exitFlag,output] = fmincon(@(q) funToMinimizeWrong(q, orientationCam, orientationIoT), ...
		[1 0 0 0],[],[],[],[],[],[], ...
		@getNonLinearConstraints, optimoptions('fmincon', 'FunctionTolerance',1e-6, 'StepTolerance',1e-6, 'OptimalityTolerance',1e-8, 'SpecifyConstraintGradient',false)); %, 'StepTolerance',1e-3, 'Algorithm','sqp', 'FunctionTolerance',1e-3, 'UseParallel',true, 'Display','iter-detailed'));
	
	A = quatTo2Dmatrix(q);
end

function A = quatTo2Dmatrix(q)
	a = q(1); b = q(2); c = q(3); d = q(4);
	A = [2*(a^2+b^2)-1  2*(b*c+a*d)    2*(b*d-a*c);
		 2*(b*c-a*d)    2*(a^2+c^2)-1  2*(c*d+a*b)];
end

function v = funToMinimizeWrong(q, orientationCam, orientationIoT)
% Note: orientationCam should be Nx2 and orientationIoT Nx3
	FR_IoTtoCam_2D = quatTo2Dmatrix(q);
	orientationIoTinCam = (FR_IoTtoCam_2D * orientationIoT')';	% Nx2
	orientationIoTinCam = orientationIoTinCam./sqrt(sum(orientationIoTinCam.^2, 2)); % Normalize to modulo 1
	v = -sum(dot(orientationCam, orientationIoTinCam, 2)); % Minus sign because we're maximizing but fmincon expects to minimize the output
end