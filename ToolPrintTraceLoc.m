function [] = ToolPrintTraceLoc(traceLoc)
  for (i=1:size(traceLoc,1))
    printf("Actual position:(%d,%d) delta VS determined:%dcm, VS expected:%dcm, Robot computed:%dcm\n",traceLoc(i,15),traceLoc(i,16), round(traceLoc(i,18)),round(traceLoc(i,19)),round(traceLoc(i,11)))
    
  endfor
endfunction