function [available] = QueryCartoAvailability(x,y,orientation,cartoId)
% multiple cartoId to be developped
% taking into account robot size and rotation to be developped
load carto1
	available=0;
if (carto1(x,y)<=1)
	available=1;
endif
endfunction