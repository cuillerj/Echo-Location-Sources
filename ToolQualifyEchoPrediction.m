function [apRobot,robot,result,quality] = ToolQualifyEchoPrediction(apRobot,robot,X,Y,nbScanToDo)
  %{
    for a specific location compute the tensor flow prediction quality
      location can be set as parameter or enter with keyboard
      
  %}
      validate=false;
      initialLocation=[0,0,0];
      result=[];
       minDistToBeDone=apGet(apRobot,'minDistToBeDone');
      if (!exist("nbScanToDo"))  % flat or rotated IA echo location 
        nbPredicionsToDo=5;
      else
        nbPredicionsToDo=nbScanToDo;
      endif
      quality=[];
      if (exist("X"))
        initialLocation(1)=X;
        initialLocation(2)=Y;
        [apRobot,robot,scanRef,distance] = ApGetClosestScanReference(apRobot,robot,initialLocation);
        if (distance>minDistToBeDone)
             printf("locations is not a scan reference (zonesXY.txt) \n ");
            return;
        endif
      else
        while (validate==false)
          initialLocation(1)=input("enter current location X (0 to stop): ");
          initialLocation(2)=input("enter current location Y (0 to stop): ");
          printf("do you confirm this location: X=%d Y=%d H=%d ? \n",initialLocation(1),initialLocation(2),0);
          validate=yes_or_no("yes or no");
          if (initialLocation(1)==0 && initialLocation(2)==0)
            printf(" stop requested  *** ");
            stopRequested=true;
            return
          endif
          [apRobot,robot,scanRef,distance] = ApGetClosestScanReference(apRobot,robot,initialLocation);
          if (distance!=0)
              printf("locations is not a scan reference (zonesXY.txt) \n ");
            validate=false;
          endif
        endwhile
      endif
      
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
        quality = [quality;[ToolQualifyComputeQuality(result),posX(1),posY(1),posH(1)]];
      endfor

      %  1) scan identificator
      %  2) position inside the predictions list: 1 is the best result - 0 means not found
      %  3) the abslolute value of the difference between the north orientation mesurment and the north oreintation reference for the current position
      %  4) the closest prediction to the actual position
      %    4 - X
      %    5 - Y
      %  6) distance between he closest prediction to the actual position 
      %  7) the first prediction (more likely) in the predictions list
      %    7 - X
      %    8 - Y
      %    9 - Heading

      dlmwrite("qualityPredictions.txt",quality,'delimiter',',','-append');
         %   filename=["quality",mat2str(min(quality(:,1))),"-",mat2str(max(quality(:,1))),".txt"]
        %   csvwrite(filename,quality);
endfunction