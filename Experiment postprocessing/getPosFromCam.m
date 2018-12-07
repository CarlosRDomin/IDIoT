function dataCam = getPosFromCam(filenameOrHDF5Contents, boolOriginAtBottomLeft)
	if nargin<2 || isempty(boolOriginAtBottomLeft)
		boolOriginAtBottomLeft = false;
	end
	dataCam = struct();  % Initialize output structure

	% Read the hdf5 file (unless filenameOrHDF5Contents already represents the file contents)
	w = waitbar(0, 'Reading hdf5 file, this might take a while...');
	if ischar(filenameOrHDF5Contents)
		hf = readHDF5(filenameOrHDF5Contents);
		initialProgress = 0.5;
	else
		hf = filenameOrHDF5Contents;
		initialProgress = 0;
	end
	
	% Figure out how many different person IDs there are in the whole video, and initialize as many entries in the camPos struct
	frame_names = fieldnames(hf.person_to_joint_assoc);	% frame00001, frame00002, and so on...
	maxID = -1;
	for i = 1:length(frame_names)
		p = hf.person_to_joint_assoc.(frame_names{i});  % p will be a (num_joints+2) x num_people_found matrix
		maxID = max([maxID, p(end,:)]);  % p(end,:) contains the ID of each person (each column is a person)
	end
	jointNames = {'nose', 'neck', 'rShoulder', 'rElbow', 'rWrist', 'lShoulder', 'lElbow', 'lWrist', 'midHip', 'rHip', 'rKnee', 'rAnkle', 'lHip', 'lKnee', 'lAnkle', 'rEye', 'lEye', 'rEar', 'lEar', 'lBigToe', 'lSmallToe', 'lHeel', 'rBigToe', 'rSmallToe', 'rHeel'};
 	dataCam.camPos = cell2struct(repmat({struct('pos2D',NaN(length(frame_names),2), 'pos3D',NaN(length(frame_names),3))}, length(jointNames),maxID+1), jointNames);  % IDs start at 0 so add 1
	
	% Scan every frame and update the info of each joint found
	waitbar(initialProgress, w, sprintf('Processing %d frames...', length(frame_names)));
	for i = 1:length(frame_names)
		if mod(i, 10) == 0
			waitbar(initialProgress + (1-initialProgress)*i/length(frame_names), w, sprintf('Frames processed: %5d/%d', i, length(frame_names)));
		end
		frame_name = frame_names{i};
		j2D = hf.joint_list.(frame_name);
		j3D = hf.joint_list_3D.(frame_name);
		p = hf.person_to_joint_assoc.(frame_name);
		
		% Iterate every person found in this frame (each column of p)
		for n = 1:size(p,2)
			personID = p(end,n)+1;  % 2nd dimension is which person (and p(end,:) contains the person IDs, starting at 0 -> Sum 1 to get person index in the struct camPos)
			jointIDs = p(1:length(jointNames), n);	% 1st dimension is which joint -> Get joint IDs (-1 if not found, >=0 if present in hf.joint_list.(Frame_name)
			if ~isempty(jointIDs) && any(jointIDs >= 0)
				for j = 1:length(jointNames)
					if jointIDs(j) < 0, continue, end  % Only find the joint info by jointID if the id is >=0 (means openpose found it)
					dataCam.camPos(personID).(jointNames{j}).pos2D(i,:) = j2D(1:2, jointIDs(j)+1);  % 1st dimension: indices 1-2 are xy pos; 2nd dimension is jointID (matlab indices start at 1 so add 1)
					dataCam.camPos(personID).(jointNames{j}).pos3D(i,:) = j3D(1:3, jointIDs(j)+1);  % Same for 3D
				end
			else
				disp('THIS IS WEIRD');
			end
		end
	end
	
	% Convert coords so vert axis increases going up and coords are in range [1, W] and [1, H] respectively
	if boolOriginAtBottomLeft
		for n = 1:length(dataCam.camPos)
			for j = 1:length(jointNames)
				dataCam.camPos(n).(jointNames{j}).pos2D(:,1) = dataCam.camPos(n).(jointNames{j}).pos2D(:,1) + 1;
				dataCam.camPos(n).(jointNames{j}).pos2D(:,2) = double(hf.height) - dataCam.camPos(n).(jointNames{j}).pos2D(:,2);
			end
		end
	end
	
	% Return any relevant information stored in the hf file (e.g. params, width, height, fps, t_start...)
	hfFields = setdiff(fieldnames(hf), {'joint_list', 'joint_list_3D', 'person_to_joint_assoc'});  % Except for those 3 fields, copy all other fields from the hdf5 file
	for i = 1:length(hfFields)
		f = hfFields{i};
		dataCam.(f) = hf.(f);
	end

	close(w);
end
