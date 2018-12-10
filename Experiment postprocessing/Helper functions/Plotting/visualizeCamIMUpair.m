function visualizeCamIMUpair(t, qIoT, nCamOrientation, posCamJoints, videoFilename, test)
	if nargin<5, videoFilename = []; end
	if nargin<6, test = []; end

	% Setup 3D orientation figure
	figure('Name', 'IMU orientation');
	hold on; grid on; 

	if ~isempty(videoFilename)
		vid = VideoReader(videoFilename);
		subplot(1,2,1);
	end

	S = [5 2 3]/5; % Cube dimensions
	[hIoT3D, Pcube] = plotCube(S, -S/2, 0.8, [0 1 0]); view(0,-90);
	hIoT3DforwardArrow = plot3(0,0,0, 'Color',[0 0.5 0], 'LineWidth', 3);
	hCam2Dlimb = plot(0,0, 'b', 'LineWidth', 5);
	hCam3Dnormal = plot3(0,0,0, 'r', 'LineWidth', 5);
	axis(repmat([-1 1], 1,3)); xlabel('x'); ylabel('y'); zlabel('z');

	timerViz = timer('BusyMode','drop', 'ExecutionMode','fixedRate', 'Period',round(1000*mean(diff(t)))/1000);
	[~, hSlider, hPlayPause] = addAnimationTimeControls(t, @(tInd)updateFigAtT(tInd, hIoT3D, hIoT3DforwardArrow, hCam2Dlimb, hCam3Dnormal, qIoT, nCamOrientation, posCamJoints(:,3:4)-posCamJoints(:,1:2), Pcube, test), @(shouldPlay)onPlayPause(shouldPlay, timerViz));
	timerViz.TimerFcn = @(src,event)vizTick(src, hSlider, hPlayPause);
	hPlayPause.notify('Action');  % 'Play' animation
end


function updateFigAtT(tInd, hIoT3D, hIoT3DforwardArrow, hCam2Dlimb, hCam3Dnormal, qIoT, nCam, dirLimbCam2D, Pcube, test)
	aux=vrrotvec([1 0 0], test);
	qIffs = quaternion(aux(1:3).*aux(4), 'rotvec');
	for i = 1:length(hIoT3D)
		P = rotatepoint(qIoT(tInd)*qIffs, Pcube(:,:,i));
		set(hIoT3D(i), 'XData', P(:,1));
		set(hIoT3D(i), 'YData', P(:,2));
		set(hIoT3D(i), 'ZData', P(:,3));
	end
	P = rotatepoint(qIoT(tInd), [1 0 0]);
  	P = rotateframe(qIoT(tInd), [0 0 0.75]);
 	P = rotatepoint(qIoT(tInd), test);
	set(hIoT3DforwardArrow, 'XData', [0 P(1)]);
	set(hIoT3DforwardArrow, 'YData', [0 P(2)]);
	set(hIoT3DforwardArrow, 'ZData', [0 P(3)]);
	set(hCam2Dlimb, 'XData', [0 dirLimbCam2D(tInd,1)]);
	set(hCam2Dlimb, 'YData', [0 dirLimbCam2D(tInd,2)]);
	set(hCam3Dnormal, 'XData', [0 nCam(tInd,1)]);
	set(hCam3Dnormal, 'YData', [0 nCam(tInd,2)]);
	set(hCam3Dnormal, 'ZData', [0 nCam(tInd,3)]);
end

function vizTick(timerViz, hSlider, hPlayPause)
	if hSlider.Value < hSlider.Max  % Show next frame
		hSlider.Value = hSlider.Value + 1;
		hSlider.notify('Action');
	end
	if hSlider.Value >= hSlider.Max  % Stop the timer if we got to the end
		stop(timerViz);
		hPlayPause.notify('Action');
	end
end

function onPlayPause(shouldPlay, timerViz)
	if shouldPlay == true
		timerViz.start();
	else
		stop(timerViz);
	end
end

function img = readFrame(vid, frameNumber)
	vid.CurrentTime = (frameNumber-1)/vid.FrameRate;
	img = vid.readFrame;
end
