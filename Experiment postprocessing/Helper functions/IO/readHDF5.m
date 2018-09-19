function data = readHDF5(filename)
	info = h5info(filename);
	data = struct();  % Initialize output struct
	
	% Fill in data from attributes
	for i = 1:length(info.Attributes)
		data.(info.Attributes(i).Name) = info.Attributes(i).Value;
	end
	
	% Fill in data from groups
	for i = 1:length(info.Groups)
		groupName = info.Groups(i).Name(2:end);
		data.(groupName) = struct();
		for j = 1:length(info.Groups(i).Datasets)
			axisName = info.Groups(i).Datasets(j).Name;
			data.(groupName).(axisName) = h5read(filename, ['/' groupName '/' axisName]);
		end
	end
end
