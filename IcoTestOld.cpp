// IcoTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


int _tmain(int argc, _TCHAR* argv[])
{
	  FILE * pFile;
	  FILE * pRaw;

	    pFile = fopen ( "iconew.csv" , "wb" );
		pRaw = fopen ( "praw.csv" , "wb" );
		float f=0.01;
		float q=0.501;
	    Uico controller(f,q);

		int N=10000;
		int proximal;
		int distal;
		printf("Running test ico\n");
		fprintf (pFile, "Time,X0,X1,U0,U1\n");
		for(int i=0; i<N; ++i)
		    {

		      // Every 200 time steps there will be a peak pair
		      // After 6000 the reflexive input vanishes
				if(i<6000)
				{
					if(i%200==20 || i%200==21 || i%200==22  || i%200==23  || i%200==24)
						proximal=-1;
					else
						proximal=0;
				}
				else
					proximal=0;

				if(i>6000 && i<6010)
				{
					controller.left_bump=1;
					controller.right_bump=1;
				}
				if(i==6000)
					controller.avoid(1.0,0.0);
				else if(i==8000)
					controller.avoid(0.0,1.0);
				else controller.avoid(0.0,0.0);
		      //proximal= (i<6000 ? (i%200==20 ? 1 : 0) : 0);
		      controller.setProximal(proximal);

			  if(i%200==14 || i%200==15 || i%200==16 || i%200==17 || i%200==18)
				  distal=-1;
			  else
				  distal=0;

			  if(i%200==18 || i%200==19 || i%200==20)
				  distal=-1;
			  else
				  distal=0;

		      //distal= (i%200==10 ? 1 : 0);
		      controller.setDistal(distal);
		      controller.filterBP();
		      controller.calculate();
			  fprintf (pFile,"%d,%d,%d,%f,%f,%f,%f\n",i,proximal,distal,controller.getU0(),controller.getU1(),controller.ul,controller.ur);
			  fprintf (pRaw,"%d,%d,%d,%d,%d,%d,%d\n",i,proximal,distal,controller.getLeftOutput(),controller.getRightOutput(),(int)controller.getDistalLeft(),(int)controller.getDistalRight());

		  }
    printf("Left syn %f Right syn %f \n",controller.getDistalLeft(),controller.getDistalRight());
	printf("Sum pos %f Sum neg %f \n",controller.sumpos,controller.sumneg);
    fclose (pFile);
	fclose (pRaw);
	return 0;
}

