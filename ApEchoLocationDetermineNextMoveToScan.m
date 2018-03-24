function [apRobot,robot,rotation,moveDist] = ApEchoLocationDetermineNextMoveToScan(apRobot,robot,plotOn,random)
  printf(mfilename);
  printf("  random(1) or max(0):%d ***  ",random);
  printf(ctime(time()))
  if (!exist("random"))  % flat logistic regression is default mode 
      random=false;
  endif
	nbPingByScan=apGet(apRobot,"nbPulse");
  nbPingByScan=nbPingByScan-1;
	frontLenght=apGet(apRobot,"frontLenght");
  securityLenght=(apGet(apRobot,"securityLenght")+frontLenght)*1.1;
	shiftAngle=pi()/(nbPingByScan);
	minDistance=apGet(apRobot,"minDistToBeDone");
	validMoveList=[];
	maxDistance=60;
	if (plotOn)
		ApPlotEchoScanRobot(apRobot,robot);
	endif
  i=0;
  for (j=0:nbPingByScan-2)
    angleR=(i)*shiftAngle;
    if (i==0)
        echo0=robot.GetScanDistBack(nbPingByScan-1);
      else
        echo0=robot.GetScanDistFront(i-1);
    endif
    echo1=robot.GetScanDistFront(i);
    echo2=robot.GetScanDistFront(i+1);
 %   coefEcho1=unifrnd(0.5,0.6);
    coefEcho1=.75;
    if (echo0>=minDistance && echo1>=minDistance && echo2>=minDistance)
      validMoveList(i+1,1)=2*echo1+echo0+echo2;
      validMoveList(i+1,2)=min([echo0-securityLenght,echo1*coefEcho1,echo1-securityLenght,echo2-securityLenght]);
      validMoveList(i+1,3)=angleR;
      %{
      if (i==0)
        echo0=robot.GetScanDistFront(nbPingByScan-1);
      else
        echo0=robot.GetScanDistBack(i-1);
      endif
      %}

    endif

    echo1=robot.GetScanDistBack(i);
    echo2=robot.GetScanDistBack(i+1);
 %   if ((echo0>=minDistance) & (echo1>=minDistance) & (echo2>=minDistance))
    validMoveList(i+1,4)=2*echo1+echo0+echo2;
    validMoveList(i+1,5)=min([echo0-securityLenght,echo1*coefEcho1,echo1-securityLenght,echo2-securityLenght]);
    i++;
  %		validMoveList(i+1,6)=angleR;
  %  else  % no possible move
 %     moveDist=0;
  %    rotation=0;
   %   return;   
  %  endif
  end

  if(!random)
    [maxValues,idxMax]=max(validMoveList);
  else
    [a,b]=size(validMoveList);
    idx=randi(a);
    maxValues=validMoveList(idx,:);
 endif
 maxFront=maxValues(1)/4;
 maxBack=maxValues(4)/4;
 rotation=-pi()/2+maxValues(3);
  if (maxFront>=maxBack)
      moveDist=maxFront;
  else
      moveDist=-maxBack;
  endif
  rotation=round(rotation*180/pi());
  if (abs(rotation) < apGet(apRobot,"minRotToBeDone"))
    rotation=0;
  endif
  dist=min(abs(moveDist),maxDistance);
  if (moveDist!=0)
    moveDist=round(dist*(moveDist/abs(moveDist)));
  endif
  [rotation,moveDist,forward] = ApOptimizeMoveToDo(rotation,moveDist,1,0);
  printf(mfilename);
  printf(" according to scan optimal move todo :(%d,%d)  *** ",rotation,moveDist);
  printf(ctime(time()))
  return
endfunction