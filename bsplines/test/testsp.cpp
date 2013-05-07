
#include "bsplines/SplinePath.h"

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

	SplinePath spline(S,5,1e-3);

	spline.printControl("control");
	spline.printPolygon("polygon");
	spline.printHull("hull");

	double s,l;


	s = spline.getControlPointAbscissa(2);
	Point Ps = spline(s);

	FILE * fp = fopen("path","w");
	l = spline.length();
	printf("Spline length: %.2f\n",l);
	for (s=0;s<=l+0.01;s+=0.1) {
		Point P = spline(s);
		Point V = spline.speed(s);
		double a = atan2(V.y,V.x);
		Point A = spline.acceleration(s);
		fprintf(fp,"%f %f %f %f %f %f %f %f %f\n",s,P.x,P.y,V.x,V.y,A.x,A.y,
				spline.orientation(s),a);
	}
	fclose(fp);

	fp = fopen("cp2","w");
	fprintf(fp,"%f %f \n",Ps.x,Ps.y);
	fclose(fp);

	return 0;
}



