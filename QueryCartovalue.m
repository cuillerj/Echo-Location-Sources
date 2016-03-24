function [cartoValue] = QueryCartovalue(x,y,orientation,cartoId)
% multiple cartoId to be developped
% taking into account robot size and rotation to be developped
load carto1

cartoValue=carto1(x,y)

endfunction