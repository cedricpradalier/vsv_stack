#include <math.h>
#include <stdio.h>

#include "bsplines/BSpline.h"


using namespace std;
using namespace splines;

//#define DEBUG_SPLINE

Point Bezier::operator()(double u) const
{
	if (u < 0) u = 0;
	if (u > 1) u = 1;

	double p = u;
	double p2 = u*p;
	double p3 = u*p2;
	double q = (1-u);
	double q2 = (1-u)*q;
	double q3 = (1-u)*q2;
#ifdef DEBUG_SPLINE
	Point O = (P[0]*q3) + (P[1]*(3*q2*p)) + (P[2]*(3*q*p2)) + (P[3]*p3);
	printf("\tt %f O %f %f\n",u,O.x,O.y);
	return O;
#else
	return (P[0]*q3) + (P[1]*(3*q2*p)) + (P[2]*(3*q*p2)) + (P[3]*p3);
#endif
}

Point Bezier::speed(double u) const
{
	double t2 = u*u;
#ifdef DEBUG_SPLINE
	Point O = (P[0]*(6*u-3*t2-3)) + (P[1]*(9*t2-12t+3)) + (P[2]*(6*u-9*t2)) + (P[3]*(3*t2));
	printf("\tt %f O %f %f %f\n",u,O.x,O.y,O.z);
	return O;
#else
	return (P[0]*(6*u-3*t2-3)) + (P[1]*(9*t2-12*u+3)) + (P[2]*(6*u-9*t2)) + (P[3]*(3*t2));
#endif
}

Point Bezier::acceleration(double u) const
{
#ifdef DEBUG_SPLINE
	Point O = (P[0]*(6-6*u)) + (P[1]*(18*u-12)) + (P[2]*(-18*u+6)) + (P[3]*(6*u));
	printf("\tt %f O %f %f %f\n",u,O.x,O.y,O.z);
	return O;
#else
	return (P[0]*(6-6*u)) + (P[1]*(18*u-12)) + (P[2]*(-18*u+6)) + (P[3]*(6*u));
#endif
}

BSpline::BSpline(const Point *s, unsigned int n) 
{
	unsigned int i;
	S.resize(n);
	for (i=0;i<n;i++) S[i] = s[i];
	B.resize(S.size());
	bezier.resize(S.size()-1);

	computeBezier();
}

BSpline::BSpline(const std::vector<Point> & s) :
	S(s)
{
	// printf("Creating spline from %f %f to %f %f\n",S[0].x,S[0].y,S.back().x,S.back().y);
	B.resize(S.size());
	bezier.resize(S.size()-1);

	computeBezier();
}

Point BSpline::operator()(double u) const
{
	unsigned int i,n;
	if (u < 0) return S[0];
	if (u >= 1) return S[S.size()-1];
#ifdef DEBUG_SPLINE
	printf("u %f ",u);
#endif
	n = bezier.size();
	u *= n;
	i = (int)floor(u);
	u = (u-i);
#ifdef DEBUG_SPLINE
	printf("i %d u %f ",i,u);
#endif
	return bezier[i](u);
}

Point BSpline::speed(double u) const
{
	unsigned int i,n;
	if (u < 0) u = 0;
	if (u >= 1) u = 1;
#ifdef DEBUG_SPLINE
	printf("u %f ",u);
#endif
	n = bezier.size();
	u *= n;
	i = (int)floor(u);
	u = (u-i);
#ifdef DEBUG_SPLINE
	printf("i %d u %f ",i,u);
#endif
	return bezier[i].speed(u);
}

double BSpline::orientation(double u) const
{
	Point V = speed(u);
	return atan2(V.y,V.x);
}

double BSpline::velocity(double u) const
{
	Point V = speed(u);
	return sqrt(V*V);
}

double BSpline::curvature(double u) const
{
	Point V = speed(u);
	Point A = acceleration(u);
	double vel = V.norm();
    if (fabs(vel)>1e-5) {
        return (V.cross(A)).norm() / (vel*vel*vel);
    } else {
        return 0.0;
    }
}


Point BSpline::acceleration(double u) const
{
	unsigned int i,n;
	if (u < 0) u = 0;
	if (u >= 1) u = 1;
	n = bezier.size();
	u *= n;
	i = (int)floor(u);
	u = (u-i);
	return bezier[i].acceleration(u);
}

//#define THE_HARD_WAY
#ifndef THE_HARD_WAY
bool BSpline::computeBezier()
{
	/** From: http://www.math.ucla.edu/~baker/149.1.02w/handouts/dd_splines.pdf
	 * **/
	unsigned int i,n;
	n = S.size();
	if (n < 4) {
		return false;
	}
	n = n-1;

	/****
	 * Solving the following equation
	 *
	 * ( 1 0 0 ...              ) ( B[0] )   ( D[0] )   (      S[0]      )
	 * ( 4 1 0 ...              ) ( B[1] )   ( D[1] )   (  6S[1] - S[0]  )
	 * ( 1 4 1 0 ...            ) (  ..  )   (  ..  )   (     6S[2]      )
	 * ( 0 1 4 1 0 ...          ) (  ..  )   (  ..  )   (      ...       )
	 * ( . 0 1 4 1 0 ...        ) (  ..  ) = (  ..  ) = (      ...       ) 
	 * ( .  . . . . . ...       ) (  ..  )   (  ..  )   (      ...       )
	 * ( ......           1 4 1 ) (  ..  )   (  ..  )   (    6S[n-2]     )
	 * ( ......             1 4 ) (  ..  )   (  ..  )   ( 6S[n-1] - S[n] )
	 * (                      1 ) ( B[n] )   ( D[n] )   (      S[n]      )
	 *
	 * Using the method from the tri-diagonal page from wikipedia
	 * *****/
	vector<Point> D(n);
	vector<double> b(n);
	D[1] = S[1]*6 - S[0];
	for (i=2;i<n-1;i++) {
		D[i] = S[i]*6;
	}
	D[n-1] = S[n-1]*6 - S[n];

	b[1] = 4;
	for (i=2;i<n;i++) {
		D[i] = D[i] - D[i-1]/b[i-1];
		b[i]=4 - 1/b[i-1];
	}
	B[n] = S[n];
	B[n-1] = D[n-1] / b[n-1];
	for (i=n-2;i>0;i--) {
		B[i] = (D[i] - B[i+1])/b[i];
	}
	B[0] = S[0];

	for (i=0;i<n;i++) {
		bezier[i].P[0] = S[i];
		bezier[i].P[1] = B[i]*(2.0/3.0) + B[i+1]*(1.0/3.0);
		bezier[i].P[2] = B[i]*(1.0/3.0) + B[i+1]*(2.0/3.0);
		bezier[i].P[3] = S[i+1];
	}
	return true;
}

#else // The hard way
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>

#ifdef DEBUG_SPLINE
static
void gsl_matrix_print(gsl_matrix * M, FILE * fp=stdout) {
    unsigned int i,j;
    fprintf(fp,"\n");
    for (i=0;i<M->size1;i++) {
        fprintf(fp,"[\t");
        for (j=0;j<M->size2;j++) {
            fprintf(fp,"%+.2f\t",gsl_matrix_get(M,i,j));
        }
        fprintf(fp,"]\n");
    }
}

static
void gsl_vector_print(gsl_vector * M, FILE * fp=stdout) {
    unsigned int j;
    fprintf(fp,"\n");
    fprintf(fp,"[\t");
    for (j=0;j<M->size;j++) {
        fprintf(fp,"%+.2e\t",gsl_vector_get(M,j));
    }
    fprintf(fp,"]\n");
}
#endif


bool BSpline::computeB() 
{

	unsigned int i,n;
	gsl_matrix *mM;
	gsl_vector *mCx, *mCy;
	gsl_permutation *p;
	n = S.size()-1;
	B[0] = S[0];
	B[n] = S[n];

	p = gsl_permutation_alloc(n-1);
	mM = gsl_matrix_alloc(n-1,n-1);
	mCx = gsl_vector_alloc(n-1);
	mCy = gsl_vector_alloc(n-1);
	gsl_matrix_set_zero(mM);
	gsl_matrix_set(mM,0,0,4);
	for (i=1;i<n-1;i++) {
		gsl_matrix_set(mM,i,i,4);
		gsl_matrix_set(mM,i,i-1,1);
		gsl_matrix_set(mM,i-1,i,1);
	}
#ifdef DEBUG_SPLINE
	printf("Matrix M\n");
	gsl_matrix_print(mM);
#endif
	for (i=1;i<n-2;i++) {
		gsl_vector_set(mCx,i,6*S[i+1].x);
		gsl_vector_set(mCy,i,6*S[i+1].y);
	}
	gsl_vector_set(mCx,0,6*S[1].x-S[0].x);
	gsl_vector_set(mCy,0,6*S[1].y-S[0].y);
	gsl_vector_set(mCx,n-2,6*S[n-1].x-S[n].x);
	gsl_vector_set(mCy,n-2,6*S[n-1].y-S[n].y);

#ifdef DEBUG_SPLINE
	printf("Vector Cx\n");
	gsl_vector_print(mCx);
	printf("Vector Cy\n");
	gsl_vector_print(mCy);
#endif

	int signum;
	gsl_linalg_LU_decomp(mM,p,&signum);
	gsl_linalg_LU_svx(mM,p,mCx);
	gsl_linalg_LU_svx(mM,p,mCy);

#ifdef DEBUG_SPLINE
	printf("Vector Bx\n");
	gsl_vector_print(mCx);
	printf("Vector By\n");
	gsl_vector_print(mCy);
#endif

	for (i=1;i<=n-1;i++) {
		B[i].x = gsl_vector_get(mCx,i-1);
		B[i].y = gsl_vector_get(mCy,i-1);
	}
#ifdef DEBUG_SPLINE
	for (i=0;i<n+1;i++) {
		printf("B[i]: %f %f\n",B[i].x,B[i].y);
	}
#endif

	gsl_vector_free(mCx);
	gsl_vector_free(mCy);
	gsl_matrix_free(mM);
	gsl_permutation_free(p);
	return true;
}

bool BSpline::computeBezier()
{
	unsigned int i,n;
	computeB();
	n = S.size();
	for (i=0;i<n-1;i++) {
		bezier[i].P[0] = S[i];
		bezier[i].P[1] = B[i]*(2.0/3.0) + B[i+1]*(1.0/3.0);
		bezier[i].P[2] = B[i]*(1.0/3.0) + B[i+1]*(2.0/3.0);
		bezier[i].P[3] = S[i+1];
#ifdef DEBUG_SPLINE
		printf("Control %d: (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f)\n",i,
				bezier[i].P[0].x, bezier[i].P[0].y,
				bezier[i].P[1].x, bezier[i].P[1].y,
				bezier[i].P[2].x, bezier[i].P[2].y,
				bezier[i].P[3].x, bezier[i].P[3].y);
#endif
	}
	return true;
}
#endif

void BSpline::printControl(const std::string & fname) const
{
	FILE * fp = fopen(fname.c_str(),"w");
	unsigned int i;
	for (i=0;i<S.size();i++) {
		fprintf(fp,"%e %e\n",S[i].x,S[i].y);
	}
	fclose(fp);
}

void BSpline::printPolygon(const std::string & fname) const
{
	FILE * fp = fopen(fname.c_str(),"w");
	unsigned int i;
	for (i=0;i<B.size();i++) {
		fprintf(fp,"%e %e\n",B[i].x,B[i].y);
	}
	fclose(fp);
}

void BSpline::printHull(const std::string & fname) const
{
	FILE * fp = fopen(fname.c_str(),"w");
	unsigned int i;
	for (i=0;i<bezier.size();i++) {
		fprintf(fp,"%e %e\n",bezier[i].P[0].x,bezier[i].P[0].y);
		fprintf(fp,"%e %e\n",bezier[i].P[1].x,bezier[i].P[1].y);
		fprintf(fp,"%e %e\n",bezier[i].P[2].x,bezier[i].P[2].y);
		fprintf(fp,"%e %e\n\n",bezier[i].P[3].x,bezier[i].P[3].y);
	}
	fclose(fp);
}

