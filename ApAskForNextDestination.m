function [isThereNewDestination] = ApAskForNextDestination();
         printf("do you want to enter an other destination ?");
         isThereNewDestination=!(yes_or_no("yes or no"));
 endfunction