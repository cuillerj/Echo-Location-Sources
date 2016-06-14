function [roundX] = RoundTo(X,R)
r=mod(X,R);
q=floor(X/R);
if (r>=2.5)
	roundX=(q+1)*R;
else
	roundX=q*R;
endif
endfunction