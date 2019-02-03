function [apRobot] = ApCreateParticlesForGoToTarget(apRobot,plotOn)
  % create particles auround robot position according to location probalibity (must be over 50%) to make sens
    locProbRange=apGet(apRobot,"locProbRange");  % probability location range in wich robot is considered as located (under first value = not located between first and second location to be checked
    sigmaLocation=[5,0.72*apGet(apRobot,"radiusMargin")];  % sigma range values of the normal distribution
    sigmaHeading=[1,0.72*apGet(apRobot,"headingMargin")]; % sigma range values of the normal distribution
    particlesNumber=[1000,3000]; % particles numner range
    locProb=apGet(apRobot,"locationProb"); % get the location probability of the robot (0-100)
    sigmaLoc=sigmaLocation(1)+(sigmaLocation(2)-sigmaLocation(1))*sqrt(1-locProb);
    sigmaHead=sigmaHeading(1)+(sigmaHeading(2)-sigmaHeading(1))*sqrt(1-locProb);
    partNumber=particlesNumber(1)+(particlesNumber(2)-particlesNumber(1))*(1-locProb);
    apRobot = setfield(apRobot,"locationProb",100); % force probability to for particles creation
    apRobot = ApCreateLocatedParticles(apRobot,partNumber,plotOn,sigmaLoc,sigmaHead); % create particles for particle filter
    apRobot = setfield(apRobot,"locationProb",locProb);  % restore current probability
endfunction