function [dataCam, dataIMU] = resampleCamAndIMU(t, dataCam, dataIMU, overwriteData)
	if nargin<4 || isempty(overwriteData), overwriteData = false; end  % If overwriteData is true, replace pos2D, pos3D, etc. instead of creating new fields pos2Dresamp, pos3Dresamp, etc.
	if overwriteData
		pos2Dname = 'pos2D';
		pos3Dname = 'pos3D';
	else
		pos2Dname = 'pos2Dresamp';
		pos3Dname = 'pos3Dresamp';
	end

	% Resample the cam positions to match a fixed sampling freq
	if ~isempty(dataCam)  % No need to do this anymore, simply resample IMU at camera times
		jointNames = fieldnames(dataCam.camPos);
		warning('off', 'MATLAB:interp1:NaNstrip');  % Silence the annoying "NaN values ignored" warning
		for n = 1:length(dataCam.camPos)
			for j = 1:length(jointNames)
				if sum(~isnan(dataCam.camPos(n).(jointNames{j}).pos2D(:,1))) < 2  % Nothing to interpolate if less than 2 points are NaN (prevents interp1 from throwing an exception)
					dataCam.camPos(n).(jointNames{j}).(pos2Dname) = nan(length(t), 2);
					dataCam.camPos(n).(jointNames{j}).(pos3Dname) = nan(length(t), 3);
				else
					dataCam.camPos(n).(jointNames{j}).(pos2Dname) = interp1(dataCam.t_frames-dataCam.t_frames(1), dataCam.camPos(n).(jointNames{j}).pos2D, t, 'pchip');
					dataCam.camPos(n).(jointNames{j}).(pos3Dname) = interp1(dataCam.t_frames-dataCam.t_frames(1), dataCam.camPos(n).(jointNames{j}).pos3D, t, 'pchip');
				end
			end
		end
	end

	% Resample IoT acceleration and forward orientation as well
	if ~isempty(dataIMU)
		namesIMUs = setdiff(fieldnames(dataIMU), 'params', 'stable');
		for iIMU = 1:length(namesIMUs)
			IMUlocation = namesIMUs{iIMU};
			magnitudesIMU = setdiff(fieldnames(dataIMU.(IMUlocation)), 'quat', 'stable');  % Cannot interpolate quaternions using interp1
			for i = 1:length(magnitudesIMU)
				strM = magnitudesIMU{i};
				if overwriteData, newStrM = strM; else, newStrM = [strM 'Resamp']; end
				if endsWith(strM, 'Resamp'), continue; end  % Don't resamp a resamp'd field
				dataIMU.(IMUlocation).(newStrM) = interp1(dataIMU.params.t - dataCam.t_frames(1), dataIMU.(IMUlocation).(strM), t, 'pchip');
			end
			
			% Manually resample the quaternion
			if overwriteData
				strQuat = 'quat';
				strForward = 'forward';
			else
				strQuat = 'quatResamp';
				strForward = 'forwardResamp';
			end
			dataIMU.(IMUlocation).(strQuat) = quaternion();
			f = dataIMU.(IMUlocation).(strForward); % Shorter notation
			for i = 1:length(f)
				if isnan(f(i,1))
					dataIMU.(IMUlocation).(strQuat)(i,:) = quaternion(1,0,0,0);
				else
					dataIMU.(IMUlocation).(strQuat)(i,:) = quaternion(vrrotvec2mat(vrrotvec([1 0 0], f(i,:))), 'rotmat', 'point');
				end
			end
		end
	end
end

