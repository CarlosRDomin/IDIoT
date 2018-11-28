function [deltaT, A, aCamShifted, aIoTshifted, aIoTtransfShifted] = find_shift_and_alignment(aCam, aIoTresamp)
	aIoT = double(aIoTresamp);	% Change name for easier reading
	
	deltaTsweep = -30:30;
	score = zeros(size(deltaTsweep));

	for iDeltaT = 1:length(deltaTsweep)
		deltaT = deltaTsweep(iDeltaT);
		
		[aCamShifted, aIoTshifted] = shiftAccelsBy(deltaT, aCam, aIoT);	% Shift by deltaT
		A = findBestAlignment(aCamShifted, aIoTshifted);	% Find best projection matrix A
		aIoTtransfShifted = (A*aIoTshifted')';	% Apply projection A to aIoT
		score(iDeltaT) = sum(diag(pdist2(aCamShifted', aIoTtransfShifted', 'correlation')));	% Compute similarity score
		continue;
		
		figure(1); for j=1:2; subplot(2,1,j); cla; hold on; plot(aCamShifted(:,j)); plot(aIoTtransfShifted(:,j)); end
		
		% Prepare next deltaT
		aIoTtransf = (A*aIoT')';
		x=xcorr2(aCam, aIoTtransf);
		if false, figure(2); plot(-size(aIoTtransf,1)+1:size(aCam,1)-1, x(:,2)); end
		[bestScore, deltaTind] = max(x(:,2));	% Since both accel signals are 2D (Nx2), x will be (2*N-1)x3, and the middle column is the one we're interested
		bestScore, deltaT = -(deltaTind-size(aIoTtransf,1))
	end
	[bestScore, deltaTind] = min(score);
	deltaT = deltaTsweep(deltaTind);
	
	[aCamShifted, aIoTshifted] = shiftAccelsBy(deltaT, aCam, aIoT);	% Shift by deltaT
	A = findBestAlignment(aCamShifted, aIoTshifted);	% Find best projection matrix A
	aIoTtransfShifted = (A*aIoTshifted')';	% Apply projection A to aIoT
end

function A = findBestAlignment(aCam, aIoT)
	% Problem definition:
	% x: [a11 a12 a13 a21 a22 a23] such that A*aIoT = aCam
	%  -> For each point in aIoT and aCam, we get two equations:
	%   (x coord): a11*aIoT(i,1) + a12*aIoT(i,2) + a13*aIoT(i,3) = aCam(i,1)
	%   (y coord): a21*aIoT(i,1) + a22*aIoT(i,2) + a23*aIoT(i,3) = aCam(i,2)
	%
	% Therefore, we can build matrices C,d such that C*x = d by having:
	%  C: [ aIoT zeros(N,3)		d: [ aCam(:,1)
	%		zeros(N,3) aIoT ]		 aCam(:,2) ]
	
	C = [aIoT zeros(size(aIoT)); zeros(size(aIoT)) aIoT];
	d = reshape(aCam, [],1);
	
	x = lsqlin(C, d);
	A = reshape(x, 3,2)';
end

function [aCamShifted, aIoTshifted] = shiftAccelsBy(deltaT, aCam, aIoT)
	if deltaT >= 0	% Positive offset
		aIoTshifted = aIoT(1+deltaT:end,:);
		aCamShifted = aCam(1:end-deltaT,:);
	else  % Negative offset
		aCamShifted = aCam(1-deltaT:end,:);
		aIoTshifted = aIoT(1:end+deltaT,:);
	end
end
