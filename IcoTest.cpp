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

		int N=400;
		int proximal;
		int distal;
		int offset=20;
		printf("Running test ico\n");
		fprintf (pFile, "Time,X0,X1,U0,U1\n");
		for(int i=0; i<N; ++i)
		    {

		      proximal=(i%100==offset ? 1 : 0);
			  //if(proximal==1)
				//  offset+=10;
		      controller.setProximal(proximal);

		      distal= (i%100==10 ? 1 : 0);
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

