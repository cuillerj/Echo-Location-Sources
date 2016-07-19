function [cartoValue] = QueryCartovalue(x,y,orientation,cartoId)

load carto1

cartoValue=carto1(x,y)

endfunction