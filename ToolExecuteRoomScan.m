 function [apRobot,robot,scanStatus] = ToolExecuteRoomScan(apRobot,robot)
   if (!exist('apRobot'))
    [apRobot,robot] = ApStartRobotRealmode(1,1)
   endif
   load ('casesNumber.txt');
   load ('zonesXY.txt');
   scanId=robot.GetMaxScanID()+1;
   minDistToBeDone=apGet(apRobot,'minDistToBeDone');
   nbScansToDo=5;
   forward=true;
   if (robot.GetMaxScanID()==0)
     printf('error scanId:%d \n',scanId)
     return
   endif
   robot.SetScanId(scanId);
   trainBatch=input("enter trainBatch number (0 to stop) ");
   if (trainBatch==0)
      printf('stop \n')
     return
   endif
   robot.SetTrainBatch(trainBatch);
   if (robot.trainBatch!=trainBatch)
     printf('error \n')
     return
   endif
   fname=['carto1Scan' num2str(trainBatch) 'Status.txt'];
   if(exist(fname,'file'))
    scanStatus=csvread(fname);
    caseId=max(scanStatus(:,1)+1);
   else
    scanStatus=[];
    caseId=1;
   endif
   stopRequest=false;
   while (!stopRequest)
     idx=find(casesNumber(:,3)==caseId);
     nextX=casesNumber(idx,1);
     nextY=casesNumber(idx,2);
     printf ('put robot in (X:%d,Y,%d,0) \n',nextX,nextY)
     validate=false;
     printf("do you confirm this location: X=%d Y=%d H=0 case:%d / no to stop ? \n",nextX,nextY,caseId);
     validate=yes_or_no("yes or no");
     if (!validate)
       stopRequest=true
       return
     endif
     count=0;
     [apRobot,robot,result,quality] = ToolQualifyEchoPrediction(apRobot,robot,nextX,nextY,nbScansToDo);
     count=count+nbScansToDo;
     [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,minDistToBeDone,forward,false);
     if (retCodeMove==0)
      [apRobot,robot,result,quality] = ToolQualifyEchoPrediction(apRobot,robot,nextX,nextY,nbScansToDo);
       count=count+nbScansToDo;
     endif
     [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,-minDistToBeDone,!forward,false);
     if (retCodeMove==0)     
      [apRobot,robot,result,quality] = ToolQualifyEchoPrediction(apRobot,robot,nextX,nextY,nbScansToDo);
       count=count+nbScansToDo;
     endif      
     [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,-minDistToBeDone,!forward,false);   
     if (retCodeMove==0)  
      [apRobot,robot,result,quality] = ToolQualifyEchoPrediction(apRobot,robot,nextX,nextY,nbScansToDo);
        count=count+nbScansToDo;
     endif     
     scanStatus=[scanStatus;[caseId,count]];
     csvwrite(fname,scanStatus);
     caseId++;
   endwhile
 endfunction