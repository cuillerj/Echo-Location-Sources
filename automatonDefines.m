  %{
  main status
  %}
  initial=1;
  localizing=2;
  targeting=3;
  gotTarget=4;
  locked=5;
  lost=6;
  mStatus=["initial";"localizing";"targeting";"gotTarget";"locked";"lost"];
   %{
  localization status
  %} 
  notLocalized=1;
  localized=2;
  localisationLost=3;
  determining=4;
  lStatus=["notLocalized";"localized";"localisationLost";"determining"];
  %{
  action status
  %}
  atRest=1;
  NOrient=2;
  moving=3;
  scanned=4;
  aStatus=["atRest";"NOrient";"moving";"moving";"scanned"];