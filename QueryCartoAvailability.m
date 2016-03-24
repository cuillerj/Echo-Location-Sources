function [available] = QueryCartoAvailability(x,y,cartoId)
% multiple cartoId to be developped
% taking into account robot size and rotation to be developped
load carto1
	available=false;
if (carto1(x,y)<=10)
	available=true;
endif
endfunction