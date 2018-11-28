addpath(genpath('.'));	% Make sure all folders and subfolders are added to the path
cdToThisScriptsDirectory();	% Change directory to the folder containing this script
DATA_FOLDER = '../DataCollection/data';
experimentStartT = '2018-11-01 16-53-56';
camPrefix = 'cam_1';

%% Load h5 files and process wrist positions from camera
if ~exist('aIoT', 'var') || ~exist('hfIoT', 'var')
	hfIoT = readHDF5([DATA_FOLDER '/' experimentStartT '/BNO055_' experimentStartT '.h5']);
	qIoT = quaternion(double(hfIoT.orientation.w), double(hfIoT.orientation.x), double(hfIoT.orientation.y), double(hfIoT.orientation.z));
	aIoT = rotatepoint(qIoT, double([hfIoT.linearAccel.x, hfIoT.linearAccel.y, hfIoT.linearAccel.z]));
end
if ~exist('pCam', 'var') || ~exist('hfCam', 'var')
	[pCam,hfCam] = get2DposFromCam([DATA_FOLDER '/' experimentStartT '/' camPrefix '_' experimentStartT '.h5']);
end
dt = diff(hfCam.t_frames);
t = 0:mean(dt):mean(dt)*length(dt); t = 10:mean(dt):105;
pCamResamp = interp1(hfCam.t_frames-hfCam.t_frames(1), pCam, t, 'pchip');
aIoTresamp = interp1(0:1/hfIoT.F_samp:(length(aIoT)-1)/hfIoT.F_samp, aIoT, t, 'pchip');

pElbowCam = get2DposFromCam(hfCam, [], 3);
pElbowCamResamp = interp1(hfCam.t_frames-hfCam.t_frames(1), pElbowCam, t, 'pchip');
forwardIoT = rotatepoint(qIoT, [1 0 0]);
forwardIoTResamp = interp1(0:1/hfIoT.F_samp:(length(qIoT)-1)/hfIoT.F_samp, forwardIoT, t, 'pchip');

%% Test

% derivFilter expects an Nxlen(t)xD signal (D is 2 because our position is 2D, and N is 1 because we are only tracking one target)
aCam = reshape(derivFilter(reshape(pCamResamp, [1 size(pCamResamp)]), 2, mean(dt)), size(pCamResamp));	% Reshape pCam to 1xlen(t)x2 and then reshape the result back to len(t)x2
subplot(2,1,1); T=mean(dt); plot(0:T:(length(aCam)-1)*T, aCam); xlim([0, (length(aCam)-1)*T]); (length(aCam)-1)*T
subplot(2,1,2); T=1/hfIoT.F_samp; plot(0:T:(length(aIoT)-1)*T, aIoT); xlim([0, (length(aIoT)-1)*T]); (length(aIoT)-1)*T

%% Find optimal alignment of frames of reference
sweepStep = 0.025;
azSweep = 0:sweepStep:2*pi;
elSweep = 0:sweepStep:pi;
rSweep = 0:sweepStep:pi;
corrScore = zeros(length(azSweep), length(elSweep), length(rSweep), 2);
[~, lag] = xcorr(aCam(:,1), aIoTresamp(:,1), 'coeff');
n = zeros(1,3);

numSweeps = length(azSweep)*length(elSweep)*length(rSweep);
cntSweeps = 0;
w = waitbar(0, 'Finding best alignment...');

for iAz = 1:length(azSweep)
	az = azSweep(iAz);
	for iEl = 1:length(elSweep)
		el = azSweep(iEl);
		[n(1) n(2) n(3)] = sph2cart(az, el, 1);
		aIoTproj3D = projectVectPlane(aIoTresamp, n);
		
		% Select an orthogonal 2D basis for the plane
		[b1(1) b1(2) b1(3)] = sph2cart(az, el+pi/2, 1);	% Unit vector at 90º extra elevation so it's perpendicular to n
		b2 = cross(b1,n);	% Make sure b2 is perpendicular to both n and b1
		aIoTproj = aIoTproj3D * [b1; b2]';	% Since b1 and b2 are unit vectors, no need to divide by their norm, so proj_b1(a) = a•b/|b| = a•b = a*b'
		
		% Camera could be rotated, try all rotations
		for iR = 1:length(rSweep)
			r = azSweep(iR);
			R = [cos(r) -sin(r);sin(r) cos(r)];	% 2D rotation matrix
			aIoTprojRot = (R*aIoTproj')';
			cX = xcorr(aCam(:,1), aIoTprojRot(:,1), 'coeff');
			cY = xcorr(aCam(:,2), aIoTprojRot(:,2), 'coeff');
			c = mean(abs([cX cY]), 2);	% Pick the one with highest average xcorr
			[cMax, iMax] = max(c);
			
			corrScore(iAz, iEl, iR, 1) = cMax;
			corrScore(iAz, iEl, iR, 2) = lag(iMax);
			cntSweeps = cntSweeps + 1;
		end
		if mod(cntSweeps, 100) == 0
			waitbar(cntSweeps/numSweeps, w, sprintf('Finding best alignment... %5d/%d', cntSweeps, numSweeps));
		end
	end
end
close(w);
return

%% 2D tracked position animation
% Setup 2D position figure
figure('Name', 'Wrist positions');
h = scatter(0,0, 'r', 'SizeData', 500);
xlim([0 hfCam.width]); ylim([0 hfCam.height]);

% Animate the position over time
for i = 1:length(pCam)
	set(h, 'XData', pCam(i,1)+1);
	set(h, 'YData', hfCam.height-pCam(i,2));
	pause(dt(i));
end

%% 2D tracked position animation
% Setup 2D position figure
figure('Name', 'Wrist positions'); hold on;
hWrist = scatter(0,0, 'SizeData', 500);
hElbow = scatter(0,0, 'SizeData', 500);
axis([0 hfCam.width 0 hfCam.height]);

% Animate the position over time
for i = 1:length(pCam)
	set(hWrist, 'XData', pCam(i,1)+1);
	%set(hWrist, 'YData', hfCam.height-pCam(i,2));
	set(hWrist, 'YData', pCam(i,2)+1);
	set(hElbow, 'XData', pElbowCam(i,1)+1);
	set(hElbow, 'YData', pElbowCam(i,2)+1);
	pause(dt(i));
end

%% Find 3D rotation and animate orientations
% Find best time shift and rotation (align frames of coordinates)
if true
	joints = [3 4]; % 3=Relbow, 4=Rwrist
	pCam = get2DposFromCam(hfCam, [], joints(1));
	pElbowCam = get2DposFromCam(hfCam, [], joints(2));
	pCamResamp = interp1(hfCam.t_frames-hfCam.t_frames(1), pCam, t, 'pchip');
	pElbowCamResamp = interp1(hfCam.t_frames-hfCam.t_frames(1), pElbowCam, t, 'pchip');
end

[deltaT, R, nCamOrientationShifted, orientationIoTshifted, orientationIoTtoCamShifted, score] = find_shift_and_alignment([pElbowCamResamp pCamResamp], forwardIoTResamp, 1440, [1920 1080]/2);%[976, 434]);
tShifted = shiftSignalBy(deltaT, t, false, 2);
limbLengthInCam = sqrt(sum((pElbowCamResamp-pCamResamp).^2, 2));
limbLengthInCamShifted = shiftSignalBy(deltaT, limbLengthInCam);
proj = dot(nCamOrientationShifted, orientationIoTtoCamShifted, 2);

figure('Name', 'CamPlane vs IoT orientation angle');
subplot(2,1,1); plot(tShifted, asind(dot(nCamOrientationShifted, orientationIoTtoCamShifted, 2))); hold on; plot(tShifted, asind(abs(dot(nCamOrientationShifted, orientationIoTtoCamShifted, 2))), '--'); stem(tShifted, 90.*(limbLengthInCamShifted<100), '--'); title(['Score: ' num2str(score)]);
subplot(2,1,2); plot(tShifted, limbLengthInCamShifted);

% Setup 3D orientation figure
figure('Name', 'IMU orientation');

subplot(1,2,1); hold on; grid on;
hIoT3D = plot3(0,0,0, 'LineWidth', 5);
hCam3D = plot3(0,0,0, 'LineWidth', 5);
axis([-1 1 -1 1 -1 1]);

subplot(1,2,2); hold on; grid on;
hIoT2D = plot([0 1],[0 0], 'LineWidth', 5);
hCam2D = plot(0,0, 'LineWidth', 5);
axis([-1 1 -1 1]);

% Animate the orientation over time
for i = 1:length(orientationIoTtoCamShifted)
	set(hIoT3D, 'XData', [0 orientationIoTtoCamShifted(i,1)]);
	set(hIoT3D, 'YData', [0 orientationIoTtoCamShifted(i,2)]);
	set(hIoT3D, 'ZData', [0 orientationIoTtoCamShifted(i,3)]);
	set(hCam3D, 'XData', [0 nCamOrientationShifted(i,1)]);
	set(hCam3D, 'YData', [0 nCamOrientationShifted(i,2)]);
	set(hCam3D, 'ZData', [0 nCamOrientationShifted(i,3)]);
	set(hCam2D, 'XData', [0 proj(i)]);
	set(hCam2D, 'YData', [0 sqrt(1 - proj(i).^2)]);
	pause(dt(i));
end
