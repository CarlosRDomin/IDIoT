function signalOut = derivFilter(signalIn, deriv, dt, poly_order, win_size)
	% signalIn is a Nxlength(t)xdims, where N indicates independent signals (eg from different drones), length(t) is the time duration, dims is the signal dimensions (eg 3D)
	if nargin<4 || isempty(poly_order)
		poly_order = 2;
	end
	if nargin<5 || isempty(win_size)
		win_size = 21;
	end
	
	[~, diffMatrix] = sgolay(poly_order, win_size);
	signalOut = zeros(size(signalIn));
	for iSignal = 1:size(signalIn,1)
		for iDim = 1:size(signalIn,3)
			signalOut(iSignal,:,iDim) = conv(signalIn(iSignal,:,iDim), factorial(deriv)/(-dt)^deriv * diffMatrix(:,deriv+1), 'same');
		end
	end
end

