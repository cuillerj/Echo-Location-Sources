			load actionSteps
			  stepSize=10; 
			actions=[
		[stepSize,0];[-stepSize,0];
		[0,stepSize];[0,-stepSize];
		[stepSize,stepSize]; [stepSize,-stepSize];  % actions list (deltaX,deltaY)
		[-stepSize,stepSize];[-stepSize,-stepSize];
		];
		a=175
		b=255
		c=5
%		c1=5
		origin=false
		AStarStep=[]
			while (origin==false)
%					AStarPath=[AStarPath,[(actionSteps(a,b,c))]]				
%					as=a;
					printf("a:%d b:%d c:%d ",a,b,c)
					printf("actionSteps:%d ",actionSteps(a,b,c))
					printf("actions: %d %d.\n",actions(actionSteps(a,b,c),1),actions(actionSteps(a,b,c),2))
					a1=a-actions(c,1)
					b1=b-actions(c,2)
					c=actionSteps(a,b,c)
					a=a1;
					b=b1;
					AStarStep=[AStarStep;[a1,b1]]


					if (c==0)
						AStarStep=[AStarStep;[initPos(1:2)]]
						AStarPath=fliplr(AStarPath)
						origin=true;
					endif
				end