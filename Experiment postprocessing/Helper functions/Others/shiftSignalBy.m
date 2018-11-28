function [sShifted] = shiftSignalBy(deltaT, s, isIoT, timeDimension)
% This function takes a time domain input signal s and crops deltaT samples from either
% the begining or the end of the timeDimension'th dimension [e.g. if timeDimension=2, then s(:,this dim would be cropped,:)].
% If isIoT=true, then deltaT>0 crops from the begining; If isIoT=false, deltaT>0 removes the last deltaT samples.
	if nargin<3 || isempty(isIoT), isIoT = false; end
	if nargin<4 || isempty(timeDimension), timeDimension = 1; end
	
	L = size(s,timeDimension);
	inds = repmat({':'}, 1,ndims(s));
	if isIoT
		inds{timeDimension} = max(1+deltaT,1):min(L, L+deltaT); % Positive deltaT means removing from the beggining
	else
		inds{timeDimension} = max(1,1-deltaT):min(L-deltaT, L); % Positive deltaT means removing from the end
	end
	sShifted = s(inds{:});
end