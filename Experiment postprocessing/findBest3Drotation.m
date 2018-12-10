% This file contains the math and optimization solver code to find the best rotation matrix R that makes fIoT be as parallel as possible to the plane whose normal is defined by nCamOrientation

function [q,v] = findBest3Drotation(nCamOrientation, qIoT, boolMaximize)
	if nargin<3 || isempty(boolMaximize), boolMaximize = false; end

	validInds = ~isnan(nCamOrientation(:,1));  % Only take into account measurements where the camera sees the body part
	nCamOrientation = nCamOrientation(validInds,:);
	qIoT = qIoT(validInds,:);

	[x,bestScore,exitFlag,output] = fmincon(@(x) computeSimilarityScore(nCamOrientation, quaternion(x(1:4))*qIoT, x(5:7), boolMaximize), ...
		[1 0 0 0 1 0 0],[],[],[],[],[],[], ...
		@getNonLinearConstraints, optimoptions('fmincon', 'FunctionTolerance',1e-6, 'StepTolerance',1e-8, 'SpecifyConstraintGradient',false, 'Display','off')); %, 'StepTolerance',1e-3, 'Algorithm','sqp', 'FunctionTolerance',1e-3, 'UseParallel',true, 'Display','iter-detailed'));

	q = quaternion(x(1:4));
	v = x(5:7);
end

function [C, Ceq, gradC, gradCeq] = getNonLinearConstraints(x)
	C = []; % <= 0
	gradC = [];
	
	Ceq = [sum(x(1:4).^2) - 1;  % = 0
		   sum(x(5:7).^2) - 1]; % = 0
	gradCeq = [2*reshape(x(1:4), [],1) zeros(4,1);
			   zeros(3,1) 2*reshape(x(5:7), [],1)];
end
