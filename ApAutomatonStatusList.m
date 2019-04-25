function [mStatus,lStatus,aStatus] = ApAutomatonStatusList(apRobot)
      mStatus={"initial";"localizing";"targeting";"gotTarget";"locked";"lost"};
      lStatus={"notLocalized";"localized";"localisationLost";"determining"};
      aStatus={"atRest";"NOrient";"moving";"scanned"};
 endfunction