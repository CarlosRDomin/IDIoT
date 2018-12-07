function [dataCam, dataIMU] = resampleCamAndIMU(t, dataCam, dataIMU)
	% Resample the cam positions to match a fixed sampling freq
	if ~isempty(dataCam)
		jointNames = fieldnames(dataCam.camPos);
		warning('off', 'MATLAB:interp1:NaNstrip');  % Silence the annoying "NaN values ignored" warning
		for n = 1:length(dataCam.camPos)
			for j = 1:length(jointNames)
				if sum(~isnan(dataCam.camPos(n).(jointNames{j}).pos2D(:,1))) < 2  % Nothing to interpolate if less than 2 points are NaN (prevents interp1 from throwing an exception)
					dataCam.camPos(n).(jointNames{j}).pos2Dresamp = nan(length(t), 2);
					dataCam.camPos(n).(jointNames{j}).pos3Dresamp = nan(length(t), 3);
				else
					dataCam.camPos(n).(jointNames{j}).pos2Dresamp = interp1(dataCam.t_frames-dataCam.t_frames(1), dataCam.camPos(n).(jointNames{j}).pos2D, t, 'pchip');
					dataCam.camPos(n).(jointNames{j}).pos3Dresamp = interp1(dataCam.t_frames-dataCam.t_frames(1), dataCam.camPos(n).(jointNames{j}).pos3D, t, 'pchip');
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
				if endsWith(strM, 'Resamp'), continue; end  % Don't resamp a resamp'd field
				dataIMU.(IMUlocation).([strM 'Resamp']) = interp1(dataIMU.params.t - dataCam.t_frames(1), dataIMU.(IMUlocation).(strM), t, 'pchip');
			end
		end
	end
end

