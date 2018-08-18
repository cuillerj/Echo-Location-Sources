function [apRobot,robot,result,quality] = ToolQualifyEchoPrediction(apRobot,robot)
      validate=false;
      initialLocation=[0,0,0];
      result=[];
      nbPredicionsToDo=5;
      quality=[];
      while (validate==false)
        initialLocation(1)=input("enter current location X (0 to stop): ");
        initialLocation(2)=input("enter current location Y (0 to stop): ");
        printf("do you confirm this location: X=%d Y=%d H=%d ? \n",initialLocation(1),initialLocation(2),0);
        validate=yes_or_no("yes or no");
        if (initialLocation(1)==0 && initialLocation(2)==0 &&initialLocation(3)==0)
          printf(" stop requested  *** ");
          stopRequested=true;
          return
        endif
        [apRobot,robot,scanRef,distance] = ApGetClosestScanReference(apRobot,robot,initialLocation);
        if (distance!=0)
            printf("locations is not a scan reference (zonesXY.txt) \n ");
          validate=false;
        endif
      end
      apRobot = setfield(apRobot,"location",initialLocation);
      robot.SetPosX(initialLocation(1));
      robot.SetPosY(initialLocation(2));
      robot.SetHeading(initialLocation(3));
      printf(" predict position \n");
      for (i=1:nbPredicionsToDo)        
        scanId=robot.GetMaxScanID()+1;   
        robot.SetScanId(scanId);
        [apRobot,robot,posX,posY,posH,posProb,retCode] = ApEchoLocalizeRobotWithTensorFlow(apRobot,robot,false,scanId);
        result=[scanId,scanRef,posX,posY,posH,posProb,robot.GetScanNOOrientation(),retCode];
        quality = [quality;[ToolQualifyComputeQuality(result)]];
      endfor
      filename=["quality",mat2str(min(quality(:,1))),"-",mat2str(max(quality(:,1))),".txt"]
      csvwrite(filename,quality);
endfunction