function out = processPreProcessedData(activities, iSubj, iWin, methods, cams, DATA_FOLDER, tWindowOverlap)
	folderPrefix = [DATA_FOLDER '/s' num2str(iSubj)];
	for iActivity = 1:length(activities)
		activity = activities{iActivity};
		activityProcessedFile = [folderPrefix '/' activity '_processed.mat'];
		activityPreProcessedFile = [folderPrefix '/' activity '_preprocessed.mat'];
		if exist(activityPreProcessedFile, 'file')~=2  % Make sure dataset has been preprocessed
			fprintf('Couldn''t find file %s for subject s%d\n', activityPreProcessedFile, iSubj);
			continue;
		end
		dataPreProc = load(activityPreProcessedFile);                                                                                                                                                

		% Load or initialize processed
		out = load(activityProcessedFile);
		tW = out.processed(iWin).tW;

		for iCam = cams %1:length(dataPreProc.dataCams)
			if iCam > length(dataPreProc.dataCams), continue; end
			t = (0:(length(dataPreProc.dataCams(iCam).camPos(1).rWrist.pos2D)-1))./dataPreProc.dataCams(iCam).fps;

			iSegment = 1;
			while true
				tS = (iSegment-1)*tW*(1-tWindowOverlap);
				tE = tS + tW;
				if tE > t(end), break; end

				tInds = (tS <= t) & (t < tE);
				for iMethod = 1:length(methods)
					m = methods(iMethod);
					out.processed(iWin).scores{iCam,m}{iSegment} = computeSimilarityMatrix(dataPreProc.dataIMU, dataPreProc.dataCams(iCam), 1:length(dataPreProc.dataCams(iCam).camPos), false, m, [], [], tInds);
				end
				fprintf('\tDone processing segment %d of s%d/%s (cam %d) tW=%d\n', iSegment, iSubj, activity, iCam, tW);
				iSegment = iSegment+1;
			end
			fprintf('Done processing s%d/%s (cam %d) tW=%d\n', iSubj, activity, iCam, tW);
		end

		% Save temp results
		fprintf('Saving!\n');
		save(activityProcessedFile, '-struct', 'out');
	end
end
