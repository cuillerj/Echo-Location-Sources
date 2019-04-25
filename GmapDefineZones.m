function [apRobot] = GmapDefineZones(apRobot)
      zones=struct();
      zones(1).fct="GmapZone1";
      zones(1).act="GmapAction1";
      zones(2).fct="GmapZone2";
      zones(2).act="GmapAction2";
      zones(3).fct="GmapZone3";
      zones(3).act="GmapAction3";
      zones(4).fct="GmapZone4";
      zones(4).act="GmapAction4";
      apRobot=setfield(apRobot,"zonesList",zones);
 endfunction