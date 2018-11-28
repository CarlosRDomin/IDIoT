function [hGroup, hSlider, hPlayPause]  = addAnimationTimeControls(t, onSetTime, onPlayPause, fig)
	if nargin<2 || isempty(onSetTime)
		onSetTime = @(x)zeros;
	end
	if nargin<3 || isempty(onPlayPause)
		onPlayPause = @(x)zeros;
	end
	if nargin<4 || isempty(fig)
		fig = gcf;
	end
	
	u = get(fig, 'Units'); set(fig, 'Units','Pixels'); p = get(fig, 'Position'); set(fig, 'Units', u);  % Get figure width and height
	W = 400; H = 50;  % [Width Height]
	hGroup = uipanel(fig, 'Title','Animation controls', 'FontSize',12, 'BackgroundColor','white', 'Units','Pixels', 'Position',[(p(3)-W)/2 25 W H], 'TitlePosition','centertop');
	hPlayPause = uicontrol(hGroup, 'Style', 'pushbutton', 'String', 'Play', 'Position', [10 10 50 20], 'Callback', @(src,event)onPlayPauseHandler(src, onPlayPause));
	hSliderLabel = uicontrol(hGroup, 'Style', 'text', 'BackgroundColor','white', 'Position', [280 10 110 20]);
	hSlider = uicontrol(hGroup, 'Style', 'slider', 'Value', 1, 'Min', 1, 'Max', length(t), 'SliderStep', [1 10]/(length(t)-1), 'Position', [70 7 200 20], 'Callback', @(src,event)onSetTimeHandler(src, t, hSliderLabel, onSetTime));
	hSlider.notify('Action');	% Trigger an action event so sliderLabel gets updated
end

function onSetTimeHandler(src, t, hSliderLabel, onSetTime)
	tInd = round(get(src, 'Value'));
	set(src, 'Value', tInd);  % Enforce int value
	set(hSliderLabel, 'String', sprintf('t=%5.2f (%5d/%d)', t(tInd), tInd, length(t)));
	onSetTime(tInd);
end

function onPlayPauseHandler(src, onPlayPause)
	initLabel = get(src, 'String');
	playLabel = 'Play'; pauseLabel = 'Pause';
	if strcmp(initLabel, playLabel)
		set(src, 'String', pauseLabel);
		onPlayPause(true);
	else
		set(src, 'String', playLabel);
		onPlayPause(false);
	end
end
