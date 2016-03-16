function [] = PlotEcho(echoX,echoY)
figure()
hold on;
for i=1:size(echoX,2)
	plot(echoX(i),echoY(i),'g');
end

endfunction