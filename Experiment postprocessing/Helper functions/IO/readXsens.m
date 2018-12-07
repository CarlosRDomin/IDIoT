function data = readXsens(filename)
% Reads a file of the Total Capture dataset and parses it. For instance, you can get linear acceleration like this:
%  plot(data.R_LowArm.accel - rotateframe(data.R_LowArm.quat, [0 0 9.8]));  % Subtract gravity from the raw accel
	data = struct();  % Initialize output struct
	f = fopen(filename);  % Open the file
	[~, name, ext] = fileparts(filename);
	filename_disp = [name ext];
	
	% Parse first line: numSensors \t numFrames
	l = strsplit(fgetl(f), '\t');
	numSensors = str2double(l{1});
	numFrames = str2double(l{2});
	w = waitbar(0, 'Reading .sensors file, this might take a while...', 'Name', filename_disp);
	set(get(get(w, 'Children'), 'Title'), 'Interpreter', 'none');  % Make sure underscores are displayed correctly
	
	% Read all frames
	for i = 1:numFrames
		data = readXsensFrame(f, numSensors, data);
		if mod(i, 10) == 0
			waitbar(i/numFrames, w, sprintf('Frames processed: %5d/%d', i, numFrames));
		end
	end
	
	fclose(f);
	close(w);
end

function data = readXsensFrame(f, numSensors, data)
	frameNum = str2double(fgetl(f));  % Read frame number
	
	for i = 1:numSensors  % Read each sensor (e.g. head, R_foreArm, etc.)
		l = strsplit(fgetl(f), '\t');
		v = str2double(l(2:end));  % Convert to double to get the actual values
	
		% Initialize the structure on the first frame (this allows for dynamic detection of sensor locations, l{1})
		if frameNum == 1
			data.(l{1}) = struct('quat', quaternion(), 'accel', [], 'gyro', [], 'mag', []);
		end
		
		% Fill in the values
		data.(l{1}).quat(frameNum,:) = quaternion(v(1:4));
		data.(l{1}).accel(frameNum,:) = v(5:7);
		data.(l{1}).gyro(frameNum,:) = v(8:10);
		data.(l{1}).mag(frameNum,:) = v(11:13);
	end
end
