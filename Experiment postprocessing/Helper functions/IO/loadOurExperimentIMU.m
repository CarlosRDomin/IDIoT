function dataIMU = loadOurExperimentIMU(filename, IMUlocation)
	if nargin<2 || isempty(IMUlocation), IMUlocation = 'R_LowArm'; end

	hfIoT = readHDF5(filename);
	dataIMU = struct(IMUlocation, [], 'params', []);
	dataIMU.(IMUlocation).quat = quaternion(double(hfIoT.orientation.w), double(hfIoT.orientation.x), double(hfIoT.orientation.y), double(hfIoT.orientation.z));
	dataIMU.(IMUlocation).forward = rotatepoint(dataIMU.(IMUlocation).quat, [1 0 0]);
	for m = {'linearAccel', 'accel', 'gyro', 'mag', 'gravity'}
		strM = m{:};
		dataIMU.(IMUlocation).(strM) = double([hfIoT.(strM).x, hfIoT.(strM).y, hfIoT.(strM).z]);
	end
	dataIMU.params.Fsamp = hfIoT.F_samp;
	dataIMU.params.tStartStr = hfIoT.t_start;
	dataIMU.params.tStart = posixtime(datetime(hfIoT.t_start, 'InputFormat','yyyy-MM-dd HH:mm:ss.S', 'TimeZone','America/Los_Angeles'));
	dataIMU.params.t = dataIMU.params.tStart + (0:(length(dataIMU.(IMUlocation).quat)-1))/dataIMU.params.Fsamp;
end
