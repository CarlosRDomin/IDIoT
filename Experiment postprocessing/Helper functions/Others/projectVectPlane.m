function projVect = projectVectPlane(vect, planeNormalVect)
	% proj_plane(u) = u - proj_n(u) = u - (u•n)/(n•n)*n
	n = reshape(planeNormalVect, 1,3);
	u = reshape(vect, [],3);
	projVect = u - (u*n')./(n*n')*n;
end
