function camCalib = readTotalCaptureCamCalib(calibFilename)
	camCalib = struct('width',[], 'height',[], 'intrinsicMat',[], 'extrinsicMat',[], 'distCoeffs',[]);  % Initialize output struct
	f = fopen(calibFilename);  % Open the file
	l = str2double(strsplit(fgetl(f), ' '));  % Parse the first line, containing the number of cameras and the distorsion coeff order
	numCams = l(1);
	numDistCoeffs = l(2);
	
	for i = 1:numCams
		l = str2double(strsplit(fgetl(f), ' '));
		camH = l(2)-l(1)+1;
		camW = l(4)-l(3)+1;
		
		l = str2double(strsplit(fgetl(f), ' '));
		intrinsicMat = [l(1) 0 l(3); 0 l(2) l(4); 0 0 1];
		
		distCoeffs = str2double(strsplit(fgetl(f), ' '));
		
		extrinsicMat = eye(4);
		for j = 1:3
			extrinsicMat(j,1:3) = str2double(strsplit(strtrim(fgetl(f)), ' '));
		end
		extrinsicMat(1:3,4) = str2double(strsplit(strtrim(fgetl(f)), ' '));
		
		camCalib(i) = struct('width',camW, 'height',camH, 'intrinsicMat',intrinsicMat, 'extrinsicMat',extrinsicMat, 'distCoeffs',distCoeffs);
	end
	fclose(f);
end
