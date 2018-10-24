function camPos = get2DposFromCam(filename, personID, jointNum)
	if nargin<2 || isempty(personID)
		personID = 0;
	end
	if nargin<3 || isempty(jointNum)
		jointNum = 5;	% Right wrist
	end

	w = waitbar(0, 'Reading hdf5 file, this might take a while...');
	hf = readHDF5(filename);
	frame_names = fieldnames(hf.person_to_joint_assoc);	% frame00001, frame00002, and so on...
	camPos = NaN(length(frame_names),2);
	
	waitbar(0.5, w, sprintf('Processing %d frames...', length(frame_names)));
	for i = 1:length(frame_names)
		if mod(i, 10) == 0
			waitbar(0.5 + 0.5*i/length(frame_names), w, sprintf('Frames processed: %5d', i));
		end
		frame_name = frame_names{i};
		p = hf.person_to_joint_assoc.(frame_name);
		jointID = p(jointNum, p(end,:)==personID);	% 1st dimension is which joint, 2nd is which person (and p(end,:) contains the person IDs)
		if ~isempty(jointID) && jointID >= 0
			j = hf.joint_list.(frame_name);
			camPos(i,:) = j(1:2, jointID+1);	% 1st dimension: indices 1-2 are xy pos; 2nd dimension is jointID (matlab indices start at 1 so add 1)
		end
	end

	close(w);
end
