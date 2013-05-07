#include <stdio.h>

#include "bsplines/BSpline.h"

using namespace splines;


int main()
{
	Point S[5] = {
		Point(1,-10), 
		Point(-1,2),
		Point(1,4),
		Point(4,3),
		Point(7,5)
	};

	BSpline spline(S,5);

	spline.printControl("control");
	spline.printPolygon("polygon");
	spline.printHull("hull");

	double t;
	FILE * fp = fopen("spline","w");
	for (t=0;t<=1.01;t+=0.01) {
		Point P = spline(t);
		fprintf(fp,"%f %f %f\n",t,P.x,P.y);
	}
	fclose(fp);

	return 0;
}



