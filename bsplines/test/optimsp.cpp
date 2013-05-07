#include <assert.h>

#include <list>
#include "bsplines/SplinePath.h"

#define N 1000
#define DIST_INTER_CTRL_MAX 3.0
#define DIST_INTER_CTRL_MIN 0.5

using namespace std;
using namespace splines;

void outputSpline(const SplinePath & sp, const string & prefix)
{
	sp.printControl((prefix+"control").c_str());
	sp.printPolygon((prefix+"polygon").c_str());
	sp.printHull((prefix+"hull").c_str());
	double s,l;
	FILE * fp = fopen((prefix + "path").c_str(),"w");
	l = sp.length();
	printf("Spline size %d length: %.2f\n",sp.getControlSize(),l);
	for (s=0;s<=l+0.01;s+=0.1) {
		Point P = sp(s);
		fprintf(fp,"%f %f %f\n",s,P.x,P.y);
	}
	fclose(fp);
}

int main(int argc, char * argv[])
{
	vector<Point> S;
	if (argc>1) {
		bool first = true;
		unsigned int k = 0;
		FILE * input;
		input = fopen(argv[1],"r");
		Point P,Pprev(0,0);
		assert(input);
		while (!feof(input)) {
			double x,y;
			int n = fscanf(input," %le %le ",&x,&y);
			if (n != 2) break;
			k += 1;
			P = Point(x,y);
			if (first || ((P-Pprev).norm()>0.25)) {
				S.push_back(P);
				Pprev = P;
				first = false;
			}
		}
		if ((P-Pprev).norm() > 1e-3) {
			S.push_back(P);
		}
		fclose(input);
		printf("Read %d points, stored %d\n",k,(int)S.size());
	} else {
		unsigned int i;
		S.resize(N);
		for (i=0;i<N;i++) {
			double x = 50 * cos(2*i*M_PI/N);
			double y = 25 * sin(4*i*M_PI/N);
			S[i].x = x; S[i].y = y;
		}
	}
	SplinePath orig(S,1e-3);
	outputSpline(orig,"o");

	SplinePath rescur = orig.resampleByCurvature(DIST_INTER_CTRL_MIN,DIST_INTER_CTRL_MAX,0.1);
	outputSpline(rescur,"c");

	double error;
	SplinePath decim = orig.decimate(0.15,&error);
	outputSpline(decim,"d");
	printf("Final error: %f\n",error);

	return 0;
}



