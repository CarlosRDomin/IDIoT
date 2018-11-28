function [camPos, hf] = get2DposFromCam(filenameOrHDF5Contents, personID, jointNum, boolOriginAtBottomLeft)
	if nargin<2 || isempty(personID)
		personID = 0;
	end
	if nargin<3 || isempty(jointNum)
		jointNum = 4;	% Right wrist
	end
	if nargin<4 || isempty(boolOriginAtBottomLeft)
		boolOriginAtBottomLeft = false;
	end
	jointNum = jointNum+1; % Original values range 0-17, but matlab indices start at 1

	% Read the hdf5 file (unless filenameOrHDF5Contents already represents the file contents)
	w = waitbar(0, 'Reading hdf5 file, this might take a while...');
	if ischar(filenameOrHDF5Contents)
		hf = readHDF5(filenameOrHDF5Contents);
		initialProgress = 0.5;
	else
		hf = filenameOrHDF5Contents;
		initialProgress = 0;
	end
	frame_names = fieldnames(hf.person_to_joint_assoc);	% frame00001, frame00002, and so on...
	camPos = NaN(length(frame_names),2);
	
	% Scan every frame looking for jointNum's joint of person personID
	waitbar(initialProgress, w, sprintf('Processing %d frames...', length(frame_names)));
	for i = 1:length(frame_names)
		if mod(i, 10) == 0
			waitbar(initialProgress + (1-initialProgress)*i/length(frame_names), w, sprintf('Frames processed: %5d/%d', i, length(frame_names)));
		end
		frame_name = frame_names{i};
		p = hf.person_to_joint_assoc.(frame_name);
		jointID = p(jointNum, p(end,:)==personID);	% 1st dimension is which joint, 2nd is which person (and p(end,:) contains the person IDs)
		if ~isempty(jointID) && jointID >= 0
			j = hf.joint_list.(frame_name);
			camPos(i,:) = j(1:2, jointID+1);	% 1st dimension: indices 1-2 are xy pos; 2nd dimension is jointID (matlab indices start at 1 so add 1)
		end
	end
	
	% Convert coords so vert axis increases going up and coords are in range [1, W] and [1, H] respectively
	if boolOriginAtBottomLeft
		camPos(:,1) = camPos(:,1) + 1;
		camPos(:,2) = double(hf.height) - camPos(:,2);
	end

	close(w);
end
