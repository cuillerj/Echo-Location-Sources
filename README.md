# Echo-Location-Sources

Logistic regression tools to be used with echo localization
The aim is to be able to localize using echo in a previously learnt space.
The space is divided by zones.
For each zone the robot makes echo scanner all around itself. This operation is repeated in order to get measurement diversity enough.
The measurements are used by a logistic regression system to learn the environment.
After that it is possible to localize the robot in a zone and define the orientation.
